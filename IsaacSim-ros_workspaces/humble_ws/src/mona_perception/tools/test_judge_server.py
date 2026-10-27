"""Tests for judge_server.

Pure helpers (no GPU):  python3 -m pytest tools/test_judge_server.py -q
GPU smoke (loads Qwen3.5-4B, in-process server): mona-judge python  tools/test_judge_server.py --gpu
The GPU checks are named gpu_* so pytest does not collect them.
"""
import base64
import json
import math
import os
import sys
import threading
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import judge_server as J  # noqa: E402

VOCAB = {'mov': 1, 'Ġmovable': 2, 'movable': 3, 'Mov': 4, 'mo': 5, 'move': 6, 'fix': 7, 'Ġfixed': 8,
         'no': 9, 'Ġno': 10, 'n': 11, 'not': 12, 'yes': 13, 'ye': 14, 'Ġyes': 15}
B64 = base64.b64encode(b'jpegbytes').decode()


def _q(**kw):
    q = {'id': 'm', 'text': 'Pick {A} or {B}.', 'options': ['movable', 'fixed']}
    q.update(kw)
    return q


def _req(**kw):
    r = {'image_jpeg_b64': B64, 'context': 'ctx\n', 'questions': [_q()]}
    r.update(kw)
    return r


def test_option_ids_prefix_and_case():
    # whole word, >=3-char prefix (with Ġ / case), but not 2-char 'mo' nor non-prefix 'move'
    assert J.option_ids(VOCAB, 'movable', 'fixed') == [1, 2, 3, 4]
    assert J.option_ids(VOCAB, 'fixed', 'movable') == [7, 8]


def test_option_ids_excludes_prefix_of_other():
    vocab = {'not': 1, 'no': 2, 'nor': 3, 'norm': 4}
    assert J.option_ids(vocab, 'north', 'normal') == []
    assert J.option_ids({'nor': 1, 'nort': 2, 'north': 3}, 'north', 'normal') == [2, 3]


def test_option_ids_short_word_exact_only():
    assert J.option_ids(VOCAB, 'no', 'yes') == [9, 10]  # 'n' (1 char), 'not' (not a prefix of 'no') excluded
    assert J.option_ids(VOCAB, 'yes', 'no') == [13, 15]  # 'ye' is 2 chars -> excluded


def test_p_from_lp():
    assert math.isclose(J.p_from_lp(-1.0, -1.0), 0.5)
    for la, lb in ((-0.3, -2.1), (-5.0, -0.1)):
        assert math.isclose(J.p_from_lp(la, lb) + J.p_from_lp(lb, la), 1.0)
    assert J.p_from_lp(0.0, -1000.0) == 1.0
    assert J.p_from_lp(-1000.0, 0.0) == 0.0
    assert J.p_from_lp(-1e9, 0.0) == 0.0


def test_p_from_lp_rejects_non_finite():
    for la, lb in ((float('nan'), -1.0), (-1.0, float('nan')), (float('inf'), float('inf'))):
        try:
            J.p_from_lp(la, lb)
        except ValueError:
            continue
        raise AssertionError((la, lb))


def test_bad_request_is_distinct_from_engine_value_error():
    assert issubclass(J.BadRequest, ValueError)
    assert not isinstance(ValueError('engine'), J.BadRequest)
    try:
        J.parse_request([])
    except J.BadRequest:
        pass
    else:
        raise AssertionError


def test_combine():
    r = J.combine(0.8, 0.4)
    assert math.isclose(r['p'], 0.6) and r['p_ab'] == 0.8 and r['p_ba'] == 0.4


def test_parse_request_ok():
    raw, ctx, qs = J.parse_request(_req())
    assert raw == b'jpegbytes' and ctx == 'ctx\n' and qs[0]['id'] == 'm'


def _bad(req):
    try:
        J.parse_request(req)
    except ValueError:
        return True
    return False


def test_parse_request_400_cases():
    assert _bad([])
    assert _bad({'context': '', 'questions': [_q()]})                       # no image
    assert _bad(_req(image_jpeg_b64='!!notb64'))
    assert _bad(_req(context=5))
    assert _bad(_req(questions=[]))
    assert _bad(_req(questions='x'))
    assert _bad(_req(questions=[_q(), _q()]))                               # duplicate id
    assert _bad(_req(questions=[_q(text='Pick {A}.')]))                      # slot missing
    assert _bad(_req(questions=[_q(text='Pick {B}.')]))
    assert _bad(_req(questions=[_q(options=['movable'])]))
    assert _bad(_req(questions=[_q(options=['a', 'b', 'c'])]))
    assert _bad(_req(questions=[_q(options=['movable', ''])]))
    assert _bad(_req(questions=[_q(id=3)]))


def test_variants_slots_and_order():
    vs = J.variants('ctx ', [_q(text='{"k": 1} {A} or {B}')])  # other braces must survive
    assert [(v['id'], v['order']) for v in vs] == [('m', 'ab'), ('m', 'ba')]
    assert vs[0]['text'] == 'ctx {"k": 1} movable or fixed'
    assert vs[1]['text'] == 'ctx {"k": 1} fixed or movable'
    assert vs[1]['pos'] == 'movable' and vs[1]['neg'] == 'fixed'


def test_letter_ids_from_vocab():
    v = {'A': 32, 'ĠA': 357, 'B': 33, 'ĠB': 417, 'x': 1}
    assert J.letter_ids(v) == {'A': [32, 357], 'B': [33, 417]}
    for drop in ('A', 'ĠB'):
        bad = {k: i for k, i in v.items() if k != drop}
        try:
            J.letter_ids(bad)
        except ValueError:
            continue
        raise AssertionError(drop)


def test_variants_letter_readout():
    q = _q(text='A) {A}\nB) {B}\nAnswer with A or B.', readout='letter')
    vs = J.variants('ctx\n', [q])
    assert vs[0]['readout'] == 'letter' and vs[0]['text'] == 'ctx\nA) movable\nB) fixed\nAnswer with A or B.'
    assert vs[1]['text'] == 'ctx\nA) fixed\nB) movable\nAnswer with A or B.'
    assert J.variants('', [_q()])[0]['readout'] == 'word'


def test_order_result_letter_both_orders():
    lpa, lpb = math.log(0.6), math.log(0.2)
    ab = J.order_result('ab', 'letter', lpa, lpb)   # pos in slot A
    ba = J.order_result('ba', 'letter', lpa, lpb)   # pos in slot B
    assert math.isclose(ab['p'], 0.75) and math.isclose(ba['p'], 0.25)
    assert math.isclose(ab['d'], math.log(3)) and math.isclose(ba['d'], -math.log(3))
    assert math.isclose(ab['mass'], 0.8) and math.isclose(ba['mass'], 0.8)
    r = J.combine_orders(ab, ba, 'letter')
    assert math.isclose(r['p'], 0.5) and math.isclose(r['d'], 0.0) and math.isclose(r['slot_mass'], 0.8)
    assert r['d_ab'] == ab['d'] and r['d_ba'] == ba['d'] and r['p_ab'] == ab['p'] and r['readout'] == 'letter'


def test_order_result_word_ignores_order():
    for o in ('ab', 'ba'):
        r = J.order_result(o, 'word', math.log(0.3), math.log(0.1))
        assert math.isclose(r['p'], 0.75) and math.isclose(r['mass'], 0.4)


def test_order_result_rejects_non_finite():
    try:
        J.order_result('ab', 'letter', float('-inf'), float('-inf'))
    except ValueError:
        return
    raise AssertionError


def test_pad_suffixes():
    rows, last = J.pad_suffixes([[1, 2, 3], [4], [5, 6]], 0)
    assert rows == [[1, 2, 3], [4, 0, 0], [5, 6, 0]] and last == [2, 0, 1]
    assert J.pad_suffixes([[7]], 9) == ([[7]], [0])
    for bad in ([], [[1], []]):
        try:
            J.pad_suffixes(bad, 0)
        except ValueError:
            continue
        raise AssertionError(bad)


def test_parse_request_readout():
    J.parse_request(_req(questions=[_q(readout='letter')]))
    J.parse_request(_req(questions=[_q(readout='word')]))
    assert _bad(_req(questions=[_q(readout='digit')]))
    assert _bad(_req(questions=[_q(readout=None)]))
    assert _bad(_req(questions=[_q(readout='letter', text='no slots')]))


# ---------------------------------------------------------------- GPU smoke
def _post(url, body):
    data = body if isinstance(body, bytes) else json.dumps(body).encode()
    req = urllib.request.Request(url + '/score', data=data, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _rect_images():
    """(movable image bgr, fixed image bgr) rect-highlighted from the mona-scenes A/B logs; None if absent."""
    ab = '/home/ciderlab-server1/project/mona-scenes/perc/ab'
    if not os.path.isdir(ab):
        return None
    sys.path.insert(0, ab)
    import ab_run
    import common as C
    got = {}
    for tag in ('s0b', 'a272', 'a281', 'f25'):
        logdir = os.path.join(C.LOGS, ab_run.LOGDIRS[tag])
        rx, ry = C.robot_xy(logdir)
        for rnd, _, _, _ in C.frame_paths(logdir):
            bgr, _, meta = C.load_frame(logdir, rnd)
            for cand in meta['candidates']:
                cls, gtm = C.gt_class(cand, rx, ry)
                if cls != C.perc_report.OTHER and gtm is not None and gtm not in got:
                    got[gtm] = C.make_highlight('rect', bgr, bbox=cand['bbox_px'])
            if len(got) == 2:
                return got[True], got[False]
    return None


def gpu_smoke(model_dir):
    import time

    import cv2
    sys.path.insert(0, '/home/ciderlab-server1/project/mona-scenes/perc/ab')
    t0 = time.time()
    srv = J.make_server(J.JudgeEngine(model_dir), '127.0.0.1', 0)
    print(f'load+warm-up {time.time() - t0:.1f}s')
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{srv.server_address[1]}'

    with urllib.request.urlopen(url + '/health') as r:
        h = json.loads(r.read())
    assert h['status'] == 'ok' and h['model'] == os.path.basename(model_dir), h
    print('PASS health', h)

    assert _post(url, b'not json')[0] == 400
    assert _post(url, _req(image_jpeg_b64=base64.b64encode(b'junk').decode()))[0] == 400  # undecodable
    assert _post(url, _req(questions=[_q(options=['zzzqx', 'fixed'])]))[0] == 400          # no matching token
    print('PASS 400 paths')

    imgs = _rect_images()
    if imgs is None:
        print('SKIP direction/latency (mona-scenes rect images unavailable)')
        srv.shutdown()
        return
    box, rack = imgs

    def b64(im):
        return base64.b64encode(cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()).decode()

    import common as C
    ctx = C.PROMPT_HEAD
    movable = {'id': 'movable', 'options': ['movable', 'fixed'], 'text': (
        'Decide whether the robot can PUSH the outlined object aside with its small arm:\n'
        '- movable : light, free-standing, small enough to shove\n'
        '            (cardboard box, empty crate, small carton, cone, bin)\n'
        '- fixed   : heavy machinery, vehicles, or fixed structure\n'
        '            (forklift, pallet jack, loaded pallet, rack, wall, pillar)\n'
        '\n'
        'Answer with exactly one word: {A} or {B}.')}
    yn = 'Answer with exactly one word: {A} or {B}.'
    five = [movable] + [{'id': f'q{i}', 'options': ['yes', 'no'], 'text': t + '\n' + yn} for i, t in enumerate((
        'Is the outlined object resting directly on the floor (not on a shelf or another object)?',
        'Could a small robot gripper (opening 10 cm) grasp and lift the outlined object?',
        'Is the outlined object a vehicle or a machine?',
        'Is the outlined object part of a shelf, rack, wall or other building structure?'))]

    def score(im, qs):
        code, resp = _post(url, {'image_jpeg_b64': b64(im), 'context': ctx, 'questions': qs})
        assert code == 200, resp
        for q in qs:
            r = resp['results'][q['id']]
            assert math.isclose(r['p'], (r['p_ab'] + r['p_ba']) / 2) and 0 <= r['p'] <= 1
        return resp

    pb = score(box, [movable])['results']['movable']['p']
    pr = score(rack, [movable])['results']['movable']['p']
    print(f'p(movable) box={pb:.3f} rack={pr:.3f}')
    assert pb > 0.5 > pr, (pb, pr)
    print('PASS direction')

    for name, qs in (('1 question', [movable]), ('5 questions', five)):
        ts = [score(box, qs)['latency_s'] for _ in range(3)]
        print(f'latency /score {name}: ' + ' '.join(f'{t:.3f}' for t in ts) + f' s (median {sorted(ts)[1]:.3f})')
    srv.shutdown()


if __name__ == '__main__':
    if '--gpu' in sys.argv:
        gpu_smoke(J.DEFAULT_MODEL)
    else:
        for name, fn in list(globals().items()):
            if name.startswith('test_'):
                fn()
                print('PASS', name)
