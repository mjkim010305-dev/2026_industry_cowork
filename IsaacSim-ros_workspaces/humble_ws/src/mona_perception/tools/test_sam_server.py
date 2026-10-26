"""Tests for sam_server (in-process server on a free port; needs GPU/weights).

Run with the mona-detect python:  python tools/test_sam_server.py   (or pytest if installed)
"""
import base64
import json
import os
import sys
import threading
import urllib.error
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sam_server  # noqa: E402

_state = {}


def _base():
    if 'url' not in _state:
        srv = sam_server.make_server(sam_server.SamEngine(), '127.0.0.1', 0)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        _state['srv'] = srv
        _state['url'] = f'http://127.0.0.1:{srv.server_address[1]}'
    return _state['url']


def _post(body):
    data = body if isinstance(body, bytes) else json.dumps(body).encode()
    req = urllib.request.Request(_base() + '/segment', data=data, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _scene():
    """A = red 100x120 at x 60..160, B = green 100x120 touching at x 160..260."""
    im = np.zeros((240, 320, 3), np.uint8)
    im[60:180, 60:160] = (0, 0, 255)
    im[60:180, 160:260] = (0, 255, 0)
    a = np.zeros((240, 320), bool)
    a[60:180, 60:160] = True
    b = np.zeros((240, 320), bool)
    b[60:180, 160:260] = True
    return im, a, b


def _body(im, **kw):
    jpg = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    return {'image_jpeg_b64': base64.b64encode(jpg).decode(), **kw}


def _masks(resp):
    return [cv2.imdecode(np.frombuffer(base64.b64decode(m), np.uint8), cv2.IMREAD_UNCHANGED)
            for m in resp['masks_png_b64']]


def _iou(m, ref):
    m = m > 127
    return (m & ref).sum() / max((m | ref).sum(), 1)


def _check_contract(resp, shape):
    assert set(resp) >= {'masks_png_b64', 'scores', 'latency_s', 'model'}
    ms = _masks(resp)
    assert len(ms) == len(resp['scores']) >= 1
    for m in ms:
        assert m.shape == shape[:2] and m.dtype == np.uint8 and m.ndim == 2
        assert set(np.unique(m)) <= {0, 255}
    assert resp['latency_s'] >= 0 and isinstance(resp['model'], str)
    return ms


def test_health():
    with urllib.request.urlopen(_base() + '/health') as r:
        assert json.loads(r.read())['status'] == 'ok'


def test_point_prompt_selects_rect_a():
    im, a, b = _scene()
    code, resp = _post(_body(im, points=[[110, 120]], point_labels=[1], box=None))
    assert code == 200
    ms = _check_contract(resp, im.shape)
    best = ms[int(np.argmax(resp['scores']))]
    assert _iou(best, a) > 0.9
    assert ((best > 127) & b).sum() < 0.05 * b.sum()


def test_point_plus_box():
    im, a, _ = _scene()
    code, resp = _post(_body(im, points=[[110, 120]], point_labels=[1], box=[55, 55, 165, 185]))
    assert code == 200
    ms = _check_contract(resp, im.shape)
    assert max(_iou(m, a) for m in ms) > 0.9


def test_box_only():
    im, a, _ = _scene()
    code, resp = _post(_body(im, points=[], point_labels=[], box=[58, 58, 162, 182]))
    assert code == 200
    ms = _check_contract(resp, im.shape)
    assert max(_iou(m, a) for m in ms) > 0.9


def test_bad_requests_400():
    im, _, _ = _scene()
    assert _post(b'not json')[0] == 400
    assert _post({'points': [[1, 1]], 'point_labels': [1]})[0] == 400  # no image
    assert _post({'image_jpeg_b64': 'AAAA', 'points': [[1, 1]], 'point_labels': [1]})[0] == 400  # undecodable
    assert _post(_body(im, points=[[1, 1]], point_labels=[1, 1]))[0] == 400  # length mismatch
    assert _post(_body(im, points=[], point_labels=[], box=None))[0] == 400  # no prompt
    assert _post(_body(im, box=[1, 2, 3]))[0] == 400  # bad box


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
    _state['srv'].shutdown()
