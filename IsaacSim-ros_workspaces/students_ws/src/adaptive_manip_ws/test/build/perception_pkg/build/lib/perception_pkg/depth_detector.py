#!/usr/bin/env python3
"""
[PROD] depth 기반 배경 차분 + blob clustering 물체 검출.
color/RGB를 전혀 쓰지 않는다 - depth 이미지에서 "배경보다 카메라 쪽으로 가까운
픽셀 덩어리"를 물체로 인식한다.

[안정성 처리 1] 배경 학습 시 여러 프레임에 걸쳐 값이 들쭉날쭉한 픽셀(경계선 등)은
"신뢰 불가"로 보고 배경에서 제외한다.
[안정성 처리 2] distance_threshold_m을 depth 센서의 자연스러운 노이즈보다 크게
잡아서(8cm), 바닥처럼 비스듬히 보이는 평면에서 생기는 노이즈를 "물체"로 오인식
하지 않게 한다 (3cm는 너무 예민해서 바닥 노이즈까지 잡혔던 문제 발견).
[안정성 처리 3] 너무 큰 덩어리(화면의 넓은 영역)는 실제 물체일 수 없으므로
max_blob_area_px로 걸러낸다.
"""

import numpy as np
import cv2


class DepthBlobDetector:
    def __init__(self, distance_threshold_m=0.10, min_blob_area_px=500,
                 max_blob_area_px=60000, background_frames=15, warmup_frames=30,
                 max_background_std_m=0.05, min_valid_ratio=0.6):
        """
        distance_threshold_m: 배경보다 이만큼(m) 더 가까우면 "물체"로 판단
        min_blob_area_px: 이보다 작은 덩어리는 노이즈로 무시
        max_blob_area_px: 이보다 큰 덩어리는 물체가 아니라 노이즈/배경 오차로 보고 무시
        background_frames: 배경 학습에 쓸 프레임 수
        warmup_frames: 배경 학습을 시작하기 전에 그냥 흘려보낼 프레임 수
        max_background_std_m: 배경 학습 프레임들 사이에서 이보다 값이 더
            들쭉날쭉한 픽셀은 "불안정"으로 보고 배경에서 제외(경계선 등)
        min_valid_ratio: 배경 학습 프레임 중 이 비율 이상이 유효(0 아님)해야
            그 픽셀을 배경으로 인정
        """
        self.distance_threshold_m = distance_threshold_m
        self.min_blob_area_px = min_blob_area_px
        self.max_blob_area_px = max_blob_area_px
        self.background_frames = background_frames
        self.warmup_frames = warmup_frames
        self.max_background_std_m = max_background_std_m
        self.min_valid_ratio = min_valid_ratio
        self._warmup_count = 0
        self._bg_accum = []
        self.background = None  # 학습된 배경 depth (m, np.float32). 0=신뢰 불가/제외

    @property
    def is_background_ready(self):
        return self.background is not None

    def update_background(self, depth_m: np.ndarray):
        """배경 학습 단계에서 프레임을 하나씩 누적. 워밍업이 끝나고 프레임이
        다 모이면, 값이 안정적인 픽셀만 골라서 배경을 확정한다."""
        if self._warmup_count < self.warmup_frames:
            self._warmup_count += 1
            return

        self._bg_accum.append(depth_m.copy())
        if len(self._bg_accum) >= self.background_frames:
            stacked = np.stack(self._bg_accum, axis=0)  # (N, H, W)
            valid_mask = stacked > 0
            valid_count = valid_mask.sum(axis=0)

            median_bg = np.median(stacked, axis=0)
            std_bg = np.std(stacked, axis=0)

            unreliable = (valid_count < self.background_frames * self.min_valid_ratio) | \
                         (std_bg > self.max_background_std_m)
            median_bg[unreliable] = 0

            self.background = median_bg
            self._bg_accum = []

    def reset_background(self):
        """카메라를 옮겼거나 장면이 바뀌어 배경을 다시 학습해야 할 때 호출."""
        self.background = None
        self._bg_accum = []
        self._warmup_count = 0

    def detect(self, depth_m: np.ndarray):
        """
        depth_m: 현재 프레임 depth (m, np.float32, 0=측정 불가)
        반환: [{"bbox": (u_min, v_min, u_max, v_max)}, ...]  (없으면 빈 리스트)
        """
        if self.background is None:
            return []

        valid = (depth_m > 0) & (self.background > 0)
        diff = np.where(valid, self.background - depth_m, 0)
        closer_mask = (diff > self.distance_threshold_m).astype(np.uint8)

        num_labels, _, stats, _ = cv2.connectedComponentsWithStats(closer_mask, connectivity=8)

        objects = []
        for label_id in range(1, num_labels):  # 0번은 배경
            area = stats[label_id, cv2.CC_STAT_AREA]
            if area < self.min_blob_area_px or area > self.max_blob_area_px:
                continue
            u_min = int(stats[label_id, cv2.CC_STAT_LEFT])
            v_min = int(stats[label_id, cv2.CC_STAT_TOP])
            w = int(stats[label_id, cv2.CC_STAT_WIDTH])
            h = int(stats[label_id, cv2.CC_STAT_HEIGHT])
            objects.append({"bbox": (u_min, v_min, u_min + w, v_min + h)})

        return objects