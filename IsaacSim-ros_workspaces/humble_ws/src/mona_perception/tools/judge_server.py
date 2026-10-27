#!/usr/bin/env python3
"""Qwen3.5 option-logprob judge server (runs on the lab server, no ROS dependency).

    python judge_server.py [--model <HF dir>] [--host 0.0.0.0] [--port 8092]

Endpoints
  GET  /health -> {"status": "ok", "model": <name>, "adapter": <LoRA dir or null>}
  POST /score  -> request  {image_jpeg_b64, context, questions:[{id, text ("{A}" and "{B}" slots), options:[positive, negative],
                            readout: "word" (default) | "letter"}]}
                  response {results:{id:{p, p_ab, p_ba, d, d_ab, d_ba, slot_mass, readout}}, latency_s, model, adapter}
readout "word": the option words fill {A}/{B}; P(option) = mass of the option's first tokens. readout "letter": the text
lists "A) {A}\nB) {B}"-style option words in both orders and the answer is read from the slot letters A / B.
d = log-odds(pos) - log-odds(neg) per order (d_ab, d_ba), d their mean; slot_mass = mean over orders of the full-vocab
probability mass on the two read-out candidates (low = the model wants to say something else).
p = P(positive) = sigmoid(logprob(first option token) difference) at the next token after the prompt;
p_ab / p_ba are the two option orders ("{A} or {B}" = positive/negative, then negative/positive), p is their mean.
The image + context prefix is encoded once per request; each question/order only runs its suffix on a copy of the
prefix cache. Bad request -> HTTP 400; model failure or prefix token mismatch, non-finite p -> HTTP 500 (no silent fallback).

Single-threaded on purpose: the GPU model is not thread-safe, one request at a time.
torch/transformers/PIL are imported only inside JudgeEngine so the pure helpers test without them.
"""
import argparse
import base64
import json
import math
import os
import time
from http.server import BaseHTTPRequestHandler, HTTPServer



class BadRequest(ValueError):
    """The client's request is wrong -> HTTP 400. Any other exception is a server failure -> HTTP 500."""


DEFAULT_MODEL = '/home/ciderlab-server1/project/mona-scenes/weights/hf/Qwen3.5-4B'


def option_ids(vocab, word, other):
    """Token ids that start `word` and not `other`: the whole word, or a >=3-char prefix of it that is not also
    a prefix of `other` (Qwen writes 'movable' as 'mov'+'able'). Case/space/'Ġ'-insensitive."""
    word, other = word.lower(), other.lower()
    ids = []
    for t, i in vocab.items():
        s = t.replace('Ġ', ' ').strip().lower()
        if s == word or (len(s) >= 3 and word.startswith(s) and not other.startswith(s)):
            ids.append(i)
    return sorted(ids)


def p_from_lp(la, lb):
    """P(a) from the two options' log-probs (two-way softmax)."""
    d = la - lb
    if not math.isfinite(d):
        raise ValueError(f'non-finite log-prob difference: {la} - {lb}')
    return 1.0 / (1.0 + math.exp(-d)) if d > -700 else 0.0


def combine(p_ab, p_ba):
    return {'p': (p_ab + p_ba) / 2, 'p_ab': p_ab, 'p_ba': p_ba}


READOUTS = ('word', 'letter')


def letter_ids(vocab):
    """Token ids of the slot letters: {'A': [id('A'), id('ĠA')], 'B': [...]}. Missing -> ValueError (startup assert)."""
    out = {}
    for L in ('A', 'B'):
        missing = [k for k in (L, 'Ġ' + L) if k not in vocab]
        if missing:
            raise ValueError(f'tokenizer vocab lacks letter tokens {missing}')
        out[L] = [vocab[L], vocab['Ġ' + L]]
    return out


def order_result(order, readout, lp_a, lp_b):
    """One prompt order -> {p, d, mass}.
    word:   lp_a / lp_b = log-prob of the positive / negative option words.
    letter: lp_a / lp_b = log-prob of slot letter A / B; ab puts pos in slot A, ba puts pos in slot B."""
    if readout == 'letter' and order == 'ba':
        lp_pos, lp_neg = lp_b, lp_a
    else:
        lp_pos, lp_neg = lp_a, lp_b
    d = lp_pos - lp_neg
    return {'p': p_from_lp(lp_pos, lp_neg), 'd': d, 'mass': math.exp(min(0.0, max(lp_pos, lp_neg) +
                                                                       math.log1p(math.exp(-abs(d)))))}


def combine_orders(r_ab, r_ba, readout='word'):
    return {'p': (r_ab['p'] + r_ba['p']) / 2, 'p_ab': r_ab['p'], 'p_ba': r_ba['p'],
            'd': (r_ab['d'] + r_ba['d']) / 2, 'd_ab': r_ab['d'], 'd_ba': r_ba['d'],
            'slot_mass': (r_ab['mass'] + r_ba['mass']) / 2, 'readout': readout}


def pad_suffixes(rows, pad_id):
    """Right-pad token-id lists to the max length -> (padded rows, last real index per row). Causal attention makes
    right padding harmless as long as logits are read at the last real token."""
    if not rows or any(len(r) == 0 for r in rows):
        raise ValueError('every suffix needs at least one token')
    width = max(len(r) for r in rows)
    return [list(r) + [pad_id] * (width - len(r)) for r in rows], [len(r) - 1 for r in rows]


def parse_request(req):
    """Validate a request dict -> (jpeg bytes, context, questions). Raises BadRequest."""
    if not isinstance(req, dict):
        raise BadRequest('body must be a JSON object')
    try:
        raw = base64.b64decode(req['image_jpeg_b64'], validate=True)
    except (KeyError, TypeError, ValueError) as e:
        raise BadRequest(f'image_jpeg_b64 missing/invalid: {e}')
    context = req.get('context', '')
    if not isinstance(context, str):
        raise BadRequest('context must be a string')
    qs = req.get('questions')
    if not isinstance(qs, list) or not qs:
        raise BadRequest('questions must be a non-empty list')
    seen = set()
    for q in qs:
        if not isinstance(q, dict) or not isinstance(q.get('id'), str) or not isinstance(q.get('text'), str):
            raise BadRequest('each question needs string id and text')
        if q['id'] in seen:
            raise BadRequest(f"duplicate question id {q['id']}")
        seen.add(q['id'])
        if '{A}' not in q['text'] or '{B}' not in q['text']:
            raise BadRequest(f"question {q['id']}: text needs both {{A}} and {{B}}")
        opts = q.get('options')
        if not isinstance(opts, list) or len(opts) != 2 or not all(isinstance(o, str) and o.strip() for o in opts):
            raise BadRequest(f"question {q['id']}: options must be two non-empty strings")
        if q.get('readout', 'word') not in READOUTS:
            raise BadRequest(f"question {q['id']}: readout must be one of {READOUTS}")
    return raw, context, qs


def variants(context, questions):
    """Two prompts per question: ab = (positive, negative) in the {A}/{B} slots, ba = swapped. str.replace, not
    format: the wording may contain other braces."""
    out = []
    for q in questions:
        pos, neg = q['options']
        for order, a, b in (('ab', pos, neg), ('ba', neg, pos)):
            out.append({'id': q['id'], 'order': order, 'pos': pos, 'neg': neg, 'readout': q.get('readout', 'word'),
                        'text': context + q['text'].replace('{A}', a).replace('{B}', b)})
    return out


class JudgeEngine:
    def __init__(self, model_dir=DEFAULT_MODEL, suffix_mode='sequential', adapter=None):
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor
        if suffix_mode not in ('sequential', 'batched'):
            raise ValueError(f'suffix_mode {suffix_mode!r}')
        self.suffix_mode = suffix_mode
        self.adapter = adapter
        self.name = os.path.basename(os.path.normpath(model_dir))
        self.proc = AutoProcessor.from_pretrained(model_dir)
        self.model = AutoModelForImageTextToText.from_pretrained(
            model_dir, dtype=torch.bfloat16, device_map='cuda').eval()
        if adapter:
            from peft import PeftModel  # lazy: only needed (and installed) when serving a LoRA
            self.model = PeftModel.from_pretrained(self.model, adapter).merge_and_unload().eval()
            self.name = f'{self.name}+{os.path.basename(os.path.normpath(adapter))}'
        self.vocab = self.proc.tokenizer.get_vocab()
        self.letters = letter_ids(self.vocab)
        self.opt_ids = {}
        # warm-up: first forward is slow (kernels); keep that out of the first real request
        from PIL import Image
        img = Image.new('RGB', (64, 64), (128, 128, 128))
        self.score(img, 'warm-up.\n', [{'id': 'w', 'text': 'Pick {A} or {B}.', 'options': ['yes', 'no']}])
        self.score(img, 'warm-up.\n', [{'id': 'w', 'text': 'A) {A}\nB) {B}\nAnswer with A or B.',
                                        'options': ['yes', 'no'], 'readout': 'letter'}])

    def decode(self, jpeg):
        import io
        from PIL import Image
        try:
            return Image.open(io.BytesIO(jpeg)).convert('RGB')
        except Exception as e:  # PIL raises several types for bad data
            raise BadRequest(f'image_jpeg_b64 is not a decodable image: {e}')

    def _ids(self, word, other):
        key = (word, other)
        if key not in self.opt_ids:
            ids = option_ids(self.vocab, word, other)
            if not ids:
                raise BadRequest(f'no vocab token matches option {word!r} (vs {other!r})')
            self.opt_ids[key] = ids
        return self.opt_ids[key]

    def _chat(self, user_text):
        msgs = [{'role': 'user', 'content': [{'type': 'image'}, {'type': 'text', 'text': user_text}]}]
        return self.proc.apply_chat_template(msgs, tokenize=False, add_generation_prompt=True,
                                             enable_thinking=False)

    def _suffix_logprobs(self, kv, n, sufs):
        """Log-softmax at the last real token of each suffix (list of 1-D id tensors) -> list of 1-D float tensors."""
        import copy
        import torch
        if self.suffix_mode == 'sequential':
            out = []
            for suf in sufs:
                with torch.no_grad():
                    lg = self.model(input_ids=suf[None], past_key_values=copy.deepcopy(kv),
                                    cache_position=torch.arange(n, n + len(suf), device='cuda'),
                                    use_cache=True).logits[0, -1]
                out.append(torch.log_softmax(lg.float(), -1))
            return out
        rows, last = pad_suffixes([s.tolist() for s in sufs], 0)
        ids = torch.tensor(rows, dtype=torch.long, device='cuda')
        with torch.no_grad():
            # expand the 1-row prefix cache to B rows (index_select copies KV and conv/recurrent states); no
            # attention_mask: with one the 3-D rope positions come from a cumsum from 0, wrong after a prefix.
            kv.reorder_cache(torch.zeros(len(rows), dtype=torch.long, device='cuda'))
            lg = self.model(input_ids=ids, past_key_values=kv,
                            cache_position=torch.arange(n, n + ids.shape[1], device='cuda'), use_cache=True).logits
        return [torch.log_softmax(lg[i, j].float(), -1) for i, j in enumerate(last)]

    def score(self, img, context, questions):
        """-> {id: {p, p_ab, p_ba, d, d_ab, d_ba, slot_mass, readout}}. Raises BadRequest (bad options/prompt) or
        RuntimeError (prefix mismatch)."""
        import torch
        vs = variants(context, questions)
        for v in vs:
            if v['readout'] == 'word':
                self._ids(v['pos'], v['neg'])
                self._ids(v['neg'], v['pos'])
        texts = [self._chat(v['text']) for v in vs]
        pre = os.path.commonprefix(texts)
        pre = pre[:pre.rfind('\n') + 1]  # cut on a line boundary -> stable tokenisation
        if '<|vision_end|>' not in pre:
            raise BadRequest('context + shared question text needs a line break after the image to cut a prefix')
        pin = self.proc(text=[pre], images=[img], return_tensors='pt').to('cuda')
        n = pin['input_ids'].shape[1]
        with torch.no_grad():
            kv = self.model(**pin, use_cache=True).past_key_values
        sufs = []
        for t in texts:
            ids = self.proc(text=[t], images=[img], return_tensors='pt')['input_ids'][0]
            if not torch.equal(ids[:n].cpu(), pin['input_ids'][0].cpu()):
                raise RuntimeError('prefix tokens differ from full-prompt tokens')
            sufs.append(ids[n:].to('cuda'))
        lps = self._suffix_logprobs(kv, n, sufs)
        r = {}
        for v, lp in zip(vs, lps):
            if v['readout'] == 'letter':
                la = torch.logsumexp(lp[self.letters['A']], 0).item()
                lb = torch.logsumexp(lp[self.letters['B']], 0).item()
            else:
                la = torch.logsumexp(lp[self._ids(v['pos'], v['neg'])], 0).item()
                lb = torch.logsumexp(lp[self._ids(v['neg'], v['pos'])], 0).item()
            r[(v['id'], v['order'])] = order_result(v['order'], v['readout'], la, lb)
        return {q['id']: combine_orders(r[(q['id'], 'ab')], r[(q['id'], 'ba')], q.get('readout', 'word'))
                for q in questions}


def handle_score(engine, req):
    t0 = time.time()
    raw, context, questions = parse_request(req)
    results = engine.score(engine.decode(raw), context, questions)
    return {'results': results, 'latency_s': round(time.time() - t0, 4), 'model': engine.name,
            'adapter': engine.adapter}


def make_server(engine, host, port):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/health':
                self._send(200, {'status': 'ok', 'model': engine.name, 'adapter': engine.adapter})
            else:
                self._send(404, {'error': 'not found'})

        def do_POST(self):
            if self.path != '/score':
                self._send(404, {'error': 'not found'})
                return
            try:
                req = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
                self._send(200, handle_score(engine, req))
            except (BadRequest, json.JSONDecodeError) as e:
                self._send(400, {'error': str(e)})
            except Exception as e:  # model failure / prefix mismatch: report, keep serving
                self._send(500, {'error': f'{type(e).__name__}: {e}'})

        def log_message(self, fmt, *args):
            print('%s %s' % (self.address_string(), fmt % args), flush=True)

    return HTTPServer((host, port), Handler)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8092)
    ap.add_argument('--suffix-mode', choices=('sequential', 'batched'), default='sequential')
    ap.add_argument('--adapter', default=None, help='LoRA dir (peft); merged into the weights at load')
    a = ap.parse_args()
    engine = JudgeEngine(a.model, a.suffix_mode, a.adapter)
    srv = make_server(engine, a.host, a.port)
    print(f'judge_server: {engine.name} suffix={engine.suffix_mode} on {a.host}:{a.port}', flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
