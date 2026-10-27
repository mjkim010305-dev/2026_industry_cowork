#!/usr/bin/env python3
"""Qwen3.5 option-logprob judge server (runs on the lab server, no ROS dependency).

    python judge_server.py [--model <HF dir>] [--host 0.0.0.0] [--port 8092]

Endpoints
  GET  /health -> {"status": "ok", "model": <model dir name>}
  POST /score  -> request  {image_jpeg_b64, context, questions:[{id, text ("{A}" and "{B}" slots), options:[positive, negative]}]}
                  response {results:{id:{p, p_ab, p_ba}}, latency_s, model}
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
    return raw, context, qs


def variants(context, questions):
    """Two prompts per question: ab = (positive, negative) in the {A}/{B} slots, ba = swapped. str.replace, not
    format: the wording may contain other braces."""
    out = []
    for q in questions:
        pos, neg = q['options']
        for order, a, b in (('ab', pos, neg), ('ba', neg, pos)):
            out.append({'id': q['id'], 'order': order, 'pos': pos, 'neg': neg,
                        'text': context + q['text'].replace('{A}', a).replace('{B}', b)})
    return out


class JudgeEngine:
    def __init__(self, model_dir=DEFAULT_MODEL):
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor
        self.name = os.path.basename(os.path.normpath(model_dir))
        self.proc = AutoProcessor.from_pretrained(model_dir)
        self.model = AutoModelForImageTextToText.from_pretrained(
            model_dir, dtype=torch.bfloat16, device_map='cuda').eval()
        self.vocab = self.proc.tokenizer.get_vocab()
        self.opt_ids = {}
        # warm-up: first forward is slow (kernels); keep that out of the first real request
        from PIL import Image
        self.score(Image.new('RGB', (64, 64), (128, 128, 128)),
                   'warm-up.\n', [{'id': 'w', 'text': 'Pick {A} or {B}.', 'options': ['yes', 'no']}])

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

    def score(self, img, context, questions):
        """-> {id: {p, p_ab, p_ba}}. Raises BadRequest (bad options/prompt) or RuntimeError (prefix mismatch)."""
        import copy
        import torch
        vs = variants(context, questions)
        for v in vs:
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
        p = {}
        for v, t in zip(vs, texts):
            ids = self.proc(text=[t], images=[img], return_tensors='pt')['input_ids'][0]
            if not torch.equal(ids[:n].cpu(), pin['input_ids'][0].cpu()):
                raise RuntimeError('prefix tokens differ from full-prompt tokens')
            suf = ids[n:].to('cuda')
            with torch.no_grad():
                lg = self.model(input_ids=suf[None], past_key_values=copy.deepcopy(kv),
                                cache_position=torch.arange(n, n + len(suf), device='cuda'),
                                use_cache=True).logits[0, -1]
            lp = torch.log_softmax(lg.float(), -1)
            la = torch.logsumexp(lp[self._ids(v['pos'], v['neg'])], 0).item()
            lb = torch.logsumexp(lp[self._ids(v['neg'], v['pos'])], 0).item()
            p[(v['id'], v['order'])] = p_from_lp(la, lb)
        return {q['id']: combine(p[(q['id'], 'ab')], p[(q['id'], 'ba')]) for q in questions}


def handle_score(engine, req):
    t0 = time.time()
    raw, context, questions = parse_request(req)
    results = engine.score(engine.decode(raw), context, questions)
    return {'results': results, 'latency_s': round(time.time() - t0, 4), 'model': engine.name}


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
                self._send(200, {'status': 'ok', 'model': engine.name})
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
    a = ap.parse_args()
    engine = JudgeEngine(a.model)
    srv = make_server(engine, a.host, a.port)
    print(f'judge_server: {engine.name} on {a.host}:{a.port}', flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
