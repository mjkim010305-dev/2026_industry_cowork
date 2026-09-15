#!/usr/bin/env python3
"""
10단계: 프레임마다 흔들리는(노이즈 섞인) 물체 위치를 매끄럽게 만드는 안정화 로직.

[왜 필요한가]
depth 센서 값은 프레임마다 조금씩 흔들린다(노이즈). 게다가 가끔 완전히 잘못된 값
(outlier)이 한 프레임 튀어나올 수도 있다. 이 흔들림을 그대로 로봇팔 제어에 넘기면
팔이 떨리듯 움직이거나 잘못된 위치로 갈 수 있다 — 그래서 "최근 몇 프레임"을 같이
보고 이상치를 거르고 평균 내는 안정화 단계가 필요하다.

[이 파일이 하는 일 — 슬롯(slot) 기준]
objects[].id는 아직 "프레임 내 순번"일 뿐 프레임 간 동일 물체를 추적하는 진짜 ID가
아니다(notes/message_spec.md의 열린 이슈 참고). 그래서 이 안정화 로직은 "매 프레임
같은 자리(배열 인덱스)에 오는 물체는 대체로 같은 물체일 것"이라는 임시 가정 위에서
동작한다 — 물체가 하나뿐인 지금 단계(Phase 1)에는 항상 맞지만, 나중에 여러 물체가
뒤섞이는 상황에서는 부정확할 수 있다. 진짜 추적 ID가 생기면 slot_index 대신 그 ID를
키로 쓰도록 바꿔야 한다.

[동작 방식]
슬롯별로 최근 window_size개의 위치를 저장해둔다.
  1) 새 위치가 최근 값들의 중앙값(median)에서 outlier_threshold_m보다 더 멀리
     떨어져 있으면 "이상치"로 보고 버린다 — 그 프레임은 직전까지의 안정된 값을
     그대로 돌려준다(값이 저장되지 않는다).
  2) 이상치가 아니면 히스토리에 추가하고, 최근 값들의 평균(단순 이동평균)을
     돌려준다.
이번 프레임에 없었던 슬롯은 reset_missing()으로 히스토리를 지워서, 나중에 그
자리에 새로운(다른) 물체가 나타났을 때 예전 물체의 히스토리와 섞이지 않게 한다.

[코드 사용 범위 표시]
  [TEST-ONLY · 교체 예정] : slot_index 기준 로직 자체가, objects[].id가 진짜 추적
                            ID로 바뀌면 그 ID 기준으로 교체되어야 한다. 평균/이상치
                            판정 방식(윈도우 크기, 임계값)도 실제 로봇에서 나오는
                            노이즈 수준을 보고 다시 조정될 수 있다.
"""

import collections


class ObjectStabilizer:
    def __init__(self, window_size=5, outlier_threshold_m=0.05, adapt_after_n_outliers=3):
        """
        window_size: 이동평균/중앙값 계산에 쓸 최근 프레임 개수.
        outlier_threshold_m: 최근 중앙값에서 이 거리(m)보다 더 벗어나면 이상치로 본다.
        adapt_after_n_outliers: "이상치로 보이는 값"이 이만큼 연속으로 들어오면,
            그건 노이즈가 아니라 물체가 실제로 다른 자리로 옮겨간 것으로 보고
            히스토리를 새로 시작한다 (아래 "왜 이게 필요한가" 참고).
        """
        self.window_size = window_size
        self.outlier_threshold_m = outlier_threshold_m
        self.adapt_after_n_outliers = adapt_after_n_outliers
        self._history = collections.defaultdict(lambda: collections.deque(maxlen=window_size))
        self._outlier_streak = collections.defaultdict(int)

    def update(self, slot_index, position):
        """
        slot_index: 이번 프레임 objects 배열에서 이 물체의 인덱스(0, 1, 2, ...).
        position: (x, y, z) — 이번 프레임에 새로 계산된 위치.
        -> (x, y, z) — 안정화된(이상치 제거 + 평균) 위치.

        [왜 "연속 이상치 카운트"가 필요한가]
        단순히 "중앙값에서 멀면 무조건 버린다"만 있으면, 물체가 실제로 다른
        위치로 바뀌었을 때(케이스 전환, 혹은 실제로 물체가 옮겨진 경우) 새 값이
        전부 "이상치"로 보여서 안정화 로직이 예전 값에 영원히 멈춰버리는 문제가
        생긴다(실제로 이 파일을 테스트하다가 발견한 문제). 그래서 이상치로 보이는
        값이 여러 프레임 연속으로 들어오면(=계속 같은 새로운 자리를 가리키면),
        그건 노이즈가 아니라 "진짜로 바뀐 값"이라고 보고 히스토리를 리셋해서
        새 위치에 다시 적응한다.
        """
        x, y, z = position
        history = self._history[slot_index]

        if history:
            mx, my, mz = self._median(history)
            dist = ((x - mx) ** 2 + (y - my) ** 2 + (z - mz) ** 2) ** 0.5
            if dist > self.outlier_threshold_m:
                self._outlier_streak[slot_index] += 1
                if self._outlier_streak[slot_index] < self.adapt_after_n_outliers:
                    # 아직은 튀는 값 하나(노이즈)로 보고 버린다 — 직전 안정값을 유지.
                    return self._average(history)
                # 연속으로 충분히 벗어났다 -> 새 위치로 적응(히스토리 리셋).
                history.clear()
                self._outlier_streak[slot_index] = 0
            else:
                self._outlier_streak[slot_index] = 0

        history.append((x, y, z))
        return self._average(history)

    def reset_missing(self, seen_slots):
        """이번 프레임에 없었던(더 이상 검출되지 않는) 슬롯의 히스토리를 지운다."""
        for slot in list(self._history.keys()):
            if slot not in seen_slots:
                del self._history[slot]
                self._outlier_streak.pop(slot, None)

    @staticmethod
    def _median(history):
        xs = sorted(p[0] for p in history)
        ys = sorted(p[1] for p in history)
        zs = sorted(p[2] for p in history)
        mid = len(history) // 2
        return xs[mid], ys[mid], zs[mid]

    @staticmethod
    def _average(history):
        n = len(history)
        sx = sum(p[0] for p in history) / n
        sy = sum(p[1] for p in history) / n
        sz = sum(p[2] for p in history) / n
        return sx, sy, sz


# [TEST-ONLY · 삭제 예정] ObjectStabilizer가 잘 동작하는지 확인하는 테스트 코드.
def main():
    stabilizer = ObjectStabilizer(window_size=5, outlier_threshold_m=0.05)

    # 슬롯 0번 물체가 (1.0, 0.0, 0.3) 근처에서 작게 흔들리다가,
    # 한 프레임(6번째)만 완전히 튀는(outlier) 상황을 흉내낸다.
    noisy_positions = [
        (1.00, 0.00, 0.30),
        (1.01, -0.01, 0.30),
        (0.99, 0.01, 0.29),
        (1.00, 0.00, 0.31),
        (1.02, -0.01, 0.30),
        (5.00, 5.00, 5.00),   # <- outlier (엉뚱한 값)
        (1.00, 0.00, 0.30),
    ]

    print("프레임  입력값                      -> 안정화된 값")
    for i, pos in enumerate(noisy_positions):
        stable = stabilizer.update(0, pos)
        flag = "  <- outlier로 거를 것으로 기대" if pos == (5.00, 5.00, 5.00) else ""
        print(f"  {i}    {pos}  -> ({stable[0]:+.3f}, {stable[1]:+.3f}, {stable[2]:+.3f}){flag}")

    stable_after_outlier = stabilizer.update(0, (1.00, 0.00, 0.30))
    assert abs(stable_after_outlier[0] - 1.00) < 0.05, "outlier가 평균에 섞여 들어감 (버그)"
    print("\nOK — outlier가 평균에 영향을 주지 않았다.")

    # 슬롯이 사라지면(reset_missing) 히스토리가 지워지는지 확인.
    stabilizer.reset_missing(seen_slots=set())
    fresh = stabilizer.update(0, (9.0, 9.0, 9.0))
    assert abs(fresh[0] - 9.0) < 1e-9, "reset_missing 이후 이전 히스토리가 남아있음 (버그)"
    print("OK — reset_missing 이후 새 물체가 이전 히스토리에 영향받지 않았다.")

    # 물체가 "진짜로" 다른 자리로 옮겨간 상황(케이스 전환) — 여러 프레임 연속으로
    # 새 위치가 들어오면, 노이즈로 오인해 예전 값에 영원히 멈추지 않고 적응해야 한다.
    stabilizer2 = ObjectStabilizer(window_size=5, outlier_threshold_m=0.05, adapt_after_n_outliers=3)
    for _ in range(5):
        stabilizer2.update(0, (1.0, 0.0, 0.3))  # 케이스 A: 8프레임 유지된다고 가정한 앞부분
    moved = None
    for _ in range(8):
        moved = stabilizer2.update(0, (1.35, 0.44, 0.3))  # 케이스 B로 전환 (실제 위치 변화)
    assert moved is not None and abs(moved[0] - 1.35) < 0.01 and abs(moved[1] - 0.44) < 0.01, (
        f"케이스 전환 후에도 예전 값에 멈춰 있음 (버그): {moved}"
    )
    print(f"OK — 케이스 전환처럼 위치가 실제로 바뀌었을 때도 새 값({moved})에 적응했다.")


if __name__ == "__main__":
    main()
