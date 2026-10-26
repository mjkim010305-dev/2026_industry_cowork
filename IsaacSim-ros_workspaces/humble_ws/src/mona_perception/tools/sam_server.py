#!/usr/bin/env python3
"""SAM 2.1 HTTP server (runs on the lab server, no ROS dependency).

    python sam_server.py [--model sam2.1_s.pt] [--host 0.0.0.0] [--port 8091]

Endpoints
  GET  /health   -> {"status": "ok", "model": <name>}
  POST /segment  -> request  {image_jpeg_b64, points:[[u,v],..], point_labels:[1,..], box:[x0,y0,x1,y1]|null}
                    response {masks_png_b64:[..], scores:[..], latency_s, model}
Masks are original-resolution single-channel PNG (0/255); SAM's 3 multimask
candidates are all returned (same order as scores). Bad request -> HTTP 400.

Single-threaded on purpose: the GPU model is not thread-safe, one request at a time.
"""
import argparse
import base64
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np


class SamEngine:
    def __init__(self, model_name='sam2.1_s.pt'):
        from ultralytics import SAM
        self.name = model_name
        self.sam = SAM(model_name)
        # The public SAM.predict() cannot pass multimask_output and drops masks with
        # score < conf, so run one warm-up predict (builds the predictor) and then drive
        # the predictor's inference() directly.
        self.sam.predict(np.zeros((64, 64, 3), np.uint8), points=[[32, 32]], labels=[1], verbose=False)
        self.pred = self.sam.predictor

    def segment(self, bgr, points, labels, box):
        """Return (masks uint8 0/255 list at bgr resolution, scores list)."""
        import torch
        from ultralytics.utils import ops
        kw = {}
        if points:
            # one object with N points: shape (1, N, 2) / (1, N). A flat (N, 2) means N objects.
            kw['points'] = [points]
            kw['labels'] = [labels]
        if box is not None:
            kw['bboxes'] = [box]  # box + points are merged by SAM2Predictor (box corners get labels 2, 3)
        self.pred.features = None  # force re-encoding of the new image
        self.pred.batch = (['img'], [bgr])  # predictor.src_shape (prompt scaling) is read from batch[1][0]
        with torch.inference_mode():
            im = self.pred.preprocess([bgr])
            masks, scores = self.pred.inference(im, multimask_output=True, **kw)
            masks = ops.scale_masks(masks[None].float(), bgr.shape[:2], padding=False)[0]
            masks = (masks > self.pred.model.mask_threshold).cpu().numpy()
        return [m.astype(np.uint8) * 255 for m in masks], [float(s) for s in scores.cpu().tolist()]


def parse_request(req):
    """Validate a request dict -> (bgr, points, labels, box). Raises ValueError."""
    if not isinstance(req, dict):
        raise ValueError('body must be a JSON object')
    try:
        raw = base64.b64decode(req['image_jpeg_b64'], validate=True)
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f'image_jpeg_b64 missing/invalid: {e}')
    bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('image_jpeg_b64 is not a decodable image')
    points = req.get('points') or []
    labels = req.get('point_labels') or []
    box = req.get('box')
    try:
        points = [[float(u), float(v)] for u, v in points]
        labels = [int(x) for x in labels]
        box = None if box is None else [float(x) for x in box]
    except (TypeError, ValueError) as e:
        raise ValueError(f'points/point_labels/box malformed: {e}')
    if len(points) != len(labels):
        raise ValueError('points and point_labels length differ')
    if box is not None and len(box) != 4:
        raise ValueError('box must be [x0,y0,x1,y1]')
    if not points and box is None:
        raise ValueError('need at least one point or a box')
    return bgr, points, labels, box


def handle_segment(engine, req):
    t0 = time.time()
    bgr, points, labels, box = parse_request(req)
    masks, scores = engine.segment(bgr, points, labels, box)
    pngs = [base64.b64encode(cv2.imencode('.png', m)[1].tobytes()).decode() for m in masks]
    return {'masks_png_b64': pngs, 'scores': scores,
            'latency_s': round(time.time() - t0, 4), 'model': engine.name}


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
            if self.path != '/segment':
                self._send(404, {'error': 'not found'})
                return
            try:
                req = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
                self._send(200, handle_segment(engine, req))
            except (ValueError, json.JSONDecodeError) as e:  # JSONDecodeError is a ValueError
                self._send(400, {'error': str(e)})
            except Exception as e:  # model failure: report, keep serving
                self._send(500, {'error': f'{type(e).__name__}: {e}'})

        def log_message(self, fmt, *args):
            print('%s %s' % (self.address_string(), fmt % args), flush=True)

    return HTTPServer((host, port), Handler)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='sam2.1_s.pt')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8091)
    a = ap.parse_args()
    engine = SamEngine(a.model)
    srv = make_server(engine, a.host, a.port)
    print(f'sam_server: {a.model} on {a.host}:{a.port}', flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
