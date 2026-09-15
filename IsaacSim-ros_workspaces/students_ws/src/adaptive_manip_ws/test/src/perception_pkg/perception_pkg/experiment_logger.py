#!/usr/bin/env python3
"""
11단계: 실험 결과를 나중에 분석할 수 있도록 CSV 파일로 남기는 로거.

[왜 필요한가]
로봇을 실제로 돌려볼 때 "몇 번째 시도에서 물체를 못 찾았는지", "위치가 시간에
따라 얼마나 흔들렸는지" 같은 걸 나중에 엑셀이나 그래프로 확인하려면, 화면에
찍히고 사라지는 print() 출력 말고 파일로 남는 기록이 필요하다.

[형식 — 한 행 = 한 프레임]
컬럼 수를 프레임마다 고정하기 위해, 물체가 몇 개든 objects_json 컬럼 하나에
그 프레임의 objects 배열 전체를 JSON 문자열로 그대로 담는다. id/detected 필드는
따로 컬럼으로 빼서 엑셀에서 바로 필터링/정렬하기 쉽게 했다.

[코드 사용 범위 표시]
  [TEST-ONLY · 교체 예정] : 로그 "형식"(컬럼 구성) 자체는 그대로 두고, 실제 로봇
                            실험에서는 여기 채워지는 값(가짜 케이스 이름 등)만
                            실제 상황 값으로 바뀐다. 저장 위치(logs/)도 그대로 재사용한다.
"""

import csv
import datetime
import json
import pathlib


class ExperimentLogger:
    COLUMNS = ["timestamp_iso", "frame", "detected", "num_objects", "objects_json"]

    def __init__(self, log_dir, run_name=None):
        """
        log_dir: 로그 파일을 저장할 디렉터리 (없으면 만든다).
        run_name: 파일 이름에 쓸 태그. 안 주면 현재 시각(YYYYMMDD_HHMMSS)을 쓴다.
        """
        log_dir = pathlib.Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        run_name = run_name or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"perception_run_{run_name}.csv"

        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.COLUMNS)
        self._file.flush()

    def log_frame(self, frame_index, detected, objects):
        """
        frame_index: 프레임 번호(1부터 증가하는 정수 권장).
        detected: bool.
        objects: [{"id":..., "position":{...}, "size":{...}}, ...] 형태의 리스트.
        """
        row = [
            datetime.datetime.now().isoformat(timespec="milliseconds"),
            frame_index,
            detected,
            len(objects),
            json.dumps(objects, ensure_ascii=False),
        ]
        self._writer.writerow(row)
        self._file.flush()  # 노드가 중간에 죽어도 그때까지 로그는 파일에 남도록 매번 flush

    def close(self):
        if not self._file.closed:
            self._file.close()


# [TEST-ONLY · 삭제 예정] ExperimentLogger가 잘 동작하는지 확인하는 테스트 코드.
def main():
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        logger = ExperimentLogger(log_dir=tmp, run_name="selftest")
        logger.log_frame(1, True, [{"id": 0, "position": {"x": 1.0, "y": 0.0, "z": 0.3},
                                     "size": {"w": 0.1, "d": 0.0, "h": 0.1}}])
        logger.log_frame(2, False, [])
        logger.close()

        with open(logger.path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))

        assert rows[0] == ExperimentLogger.COLUMNS, f"헤더가 다름: {rows[0]}"
        assert rows[1][1] == "1" and rows[1][2] == "True" and rows[1][3] == "1"
        assert rows[2][1] == "2" and rows[2][2] == "False" and rows[2][3] == "0"

        parsed = json.loads(rows[1][4])
        assert parsed[0]["id"] == 0 and parsed[0]["position"]["x"] == 1.0

        print(f"임시 로그 파일: {logger.path}")
        print("기록된 내용:")
        for row in rows:
            print("  ", row)
        print("\nOK — 헤더/행 구성/JSON 인코딩 모두 정상.")


if __name__ == "__main__":
    main()
