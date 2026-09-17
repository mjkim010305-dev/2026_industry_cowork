# Behavior Tree × Perception 통합 설계 논의

> **주제**: Nav2 BT 기반 장애물 밀어내기 시스템에서, VLM 기반 비동기 perception을 BT 구조에 어떻게 통합할 것인가
> **결론 요약**: perception을 BT 안에 넣지 않고, 독립 ROS2 노드로 분리한 뒤 내부 상태 머신으로 안정화하여 토픽으로 발행한다

---

## 1. 문제 정의 — BT tick과 perception 스트림의 인터페이스 불일치

### 출발점

BT는 동기적 tick 기반으로 동작한다. 매 tick마다 노드가 `SUCCESS` / `FAILURE` / `RUNNING`을 반환하는 구조다. 반면 perception은 ROS 토픽 기반 비동기 스트림이다. 카메라 프레임마다 콜백으로 포즈를 발행하고, 미검출 시에는 값이 없거나 저역필터로 인한 지연이 발생한다. 이 둘의 인터페이스가 근본적으로 맞지 않는다.

일반적인 해결 패턴은 세 가지다.

- **(a)** blackboard에 최신 포즈값을 캐싱하고, BT의 Condition 노드가 매 tick 최신값만 읽게 한다
- **(b)** 미검출 시 이전 추정치를 유지(extrapolation)하여 BT 쪽에 "값이 끊기는" 상황 자체를 만들지 않는다
- **(c)** 신뢰도 낮은 관측치는 애초에 거부(rejection)하여 BT에 노이즈가 전달되지 않게 막는다

흥미롭게도 이 세 가지는 SMMS가 하는 일(외삽, 저역필터, rejection)과 정확히 일치한다. 즉 해당 팀은 "BT와 perception을 어떻게 맞출까"라는 문제를 **BT 쪽을 정교하게 만드는 대신, perception 출력 자체를 미리 안정화시켜 상위 로직이 노이즈를 신경 쓸 필요가 없게 만드는** 전략을 취한 것으로 보인다.

### 내 상황

현재 나는 Nav2의 BT를 직접 설계하고 있어서 매 tick마다 판단이 이루어진다. 여기에 다음 두 가지 제약이 겹친다.

1. 매 tick마다 perception 과정을 넣는 것은 연산상 무리다
2. Perception 과정이 "프레임을 VLM에 제공 → VLM output인 ROI를 삽입"하는 구조라, **매 tick마다 ROI가 변경될 가능성**이 존재한다

---

## 2. 내가 먼저 검토한 세 가지 아이디어와 한계

| # | 아이디어 | 내가 인식한 한계 |
|---|---------|----------------|
| 1 | ROI가 커질 때마다 perception update | ROI를 매 tick 잘 찾는다 해도, 이 과정으로 얻는 **법선 접근 좌표가 계속 변경될 수 있음** |
| 2 | 특정 시간마다 perception 적용 | 휴리스틱한 접근. **특정 시간을 얼마로 정의할지 불명확**하며, 1번의 한계도 그대로 존재 |
| 3 | 특정 시간마다 snapshot/log 저장 후 평균 | 여러 기록의 ROI 정보를 종합하므로 어느정도 안정화 가능하지만, **역으로 값이 튀어버릴 위험성** 존재 |

### 이에 대한 재평가

세 아이디어 모두 **"BT 쪽 로직(트리거 조건)이 perception의 갱신 시점을 결정한다"**는 전제를 공유하고 있다. 이것이 오히려 문제를 만드는 지점이다.

Perception이 이미 토픽 기반 콜백으로 돌고 있으니, "BT tick마다 perception을 새로 부를지 말지"는 애초에 고민할 필요가 없다. Perception은 자기 페이스대로(카메라 프레임 → VLM 추론 지연만큼) 계속 돌고, BT는 최신 상태를 구독만 하면 된다 — **AMCL pose나 costmap을 BT가 소비하는 방식과 동일하다.**

진짜 문제는 "perception이 낸 raw 추정치를 BT가 그대로 믿고 반응해도 되는가"이고, 이는 트리거 시점의 문제가 아니라 **perception 출력 자체의 안정화** 문제다. 즉 세 아이디어 모두 "BT가 **언제 물어볼까**"에 초점이 맞춰져 있는데, 맞춰야 할 지점은 "perception이 **무엇을 발행할까**"다.

**개별 평가**

- **1번**: 방향은 맞다. 로봇이 접근할수록 해상도가 좋아지고 법선 추정이 정확해지는 건 사실이다. 다만 이걸 이산적 트리거로 쓰면 임계값 튜닝 문제가 남는다. → **필터의 신뢰도 가중치(confidence weight)로 흡수**하는 것이 맞다. ROI 크기(=거리 proxy)를 신뢰도 스코어에 넣어, 가까울수록 새 관측치에 더 큰 가중치를 부여.
- **2번**: 임계값이 불명확하고, "언제 유효한 추정치가 나왔는가"와 무관한 인위적 스케줄이라 1번의 한계를 상속한다. VLM 콜백이 들어올 때마다 이벤트 드리븐으로 반영하면 되므로 굳이 쓸 이유가 없다.
- **3번**: 방향성은 셋 중 가장 맞다(SMMS의 저역필터/rejection과 같은 계열). 다만 지적한 위험이 실재한다 — **최종 pose(yaw, 좌표)를 평균내면** outlier 하나가 통계를 크게 흔들고, 특히 yaw는 wrap-around 문제까지 있어 naive averaging이 위험하다.

---

## 3. 제안 — Perception Stabilizer를 BT 밖 별도 노드로 분리

### 핵심 설계 4가지

**① 집계 레벨을 pose가 아니라 point cloud로 내리기**

기존 "ROI → depth unproject → RANSAC + PCA" 파이프라인의 **입력 자체**를 단일 프레임이 아니라, 최근 N개 유효 프레임의 3D 포인트를 누적한 rolling buffer로 만든다. 매 프레임 개별로 yaw를 뽑아 그 값들을 평균내는 게 아니라, **포인트를 누적한 뒤 누적된 클라우드에 RANSAC+PCA를 한 번** 돌린다. RANSAC이 이미 outlier rejection을 내장하고 있으므로 3번 아이디어의 outlier 위험이 크게 줄어든다.

**② Convergence 기반 stability flag**

buffer 내 프레임별 yaw/normal의 분산이 임계값(예: 5도) 이하로 떨어지면 `stable=True`. "특정 시간"이라는 임의 기준 대신 **실제 데이터의 수렴 여부**를 기준으로 삼는다.

**③ Confidence 기반 rejection + extrapolation**

VLM ROI confidence가 낮거나, depth 값이 튀거나(median filter 이상치), 미검출 프레임이면 buffer에 넣지 않고 이전 추정치 유지. (SMMS 논의에서 나온 그대로)

**④ Lock 메커니즘 (가장 중요)**

approach 단계를 지나 실제 push 실행 단계로 들어가면, 그 시점의 추정치를 blackboard에 **freeze**하고 push 실행 중에는 perception 업데이트를 무시한다. 그렇지 않으면 VLM 노이즈로 push 목표점이 실행 중 계속 흔들리고, 이는 **Tang et al.의 point contact 불안정성 문제와 겹쳐 더 나쁜 상황**을 만든다. 이미 확정한 "push phase는 Nav2 controller 밖에서 별도 실행" 아키텍처와도 자연스럽게 맞는다 — push 단계 진입 순간 perception 입력을 끊으면 된다.

### BT 인터페이스

```
[BT 외부 — 독립 노드]
  Perception Stabilizer
    발행: /obstacle_pose_stable, /obstacle_pose_confidence

[BT 내부 — 얇은 래퍼만]
  Condition : IsPerceptionStable      → stability flag 읽기
  Action    : LockApproachPose        → stable 시점에 blackboard로 값 복사, 이후 재구독 X
  Action    : ComputeApproachDirection ┐
  Action    : NavigateToApproachPose   ├ 전부 blackboard 값만 읽음 (perception 직접 접근 X)
  Action    : PushObstacle             ┘
```

이렇게 하면 BT tick 주기와 perception 갱신 주기가 서로를 신경 쓸 필요가 없어지고, "언제 믿을 만한 값인가"라는 판단만 stabilizer에 캡슐화된다.

---

## 4. 확인 질문 — "그럼 별도 python script commander인가?"

> 현재 depth camera로 장애물 표면의 방향을 확인하는 것처럼 PCA는 그대로 사용하되, 이 방식을 behavior tree 구조에 넣는 것이 아니라 **별개의 python script commander로써 작동**시키자는 말인가?

**맞는 부분**: PCA(+RANSAC) 연산 자체는 유지하되, 이 연산을 BT tick 안에서 돌리지 않고 BT와 독립적으로 도는 별도 프로세스로 분리한다는 것 — 정확하다.

**다만 "commander"라는 표현에서 짚을 점**: `BasicNavigator` 같은 커맨더 스크립트(외부에서 필요할 때 함수 호출하듯 부르는 것)로 만들면 안 되고, **자기 페이스대로 계속 도는 독립 ROS2 노드**여야 한다.

| | 방식 | 평가 |
|---|---|---|
| ❌ | BT가 "지금 자세 계산해줘" 요청 → stabilizer가 그때 계산해 응답 | 여전히 BT tick에 종속됨 |
| ✅ | Stabilizer가 카메라 콜백마다 알아서 buffer에 쌓고 PCA를 돌리며, 안정화 결과를 토픽에 계속 발행 → BT는 최신값을 구독만 | tick 주기와 완전 분리 |

**비유**: 지금 팀이 쓰는 AMCL이나 costmap과 완전히 같은 관계다. AMCL도 BT가 "지금 위치 계산해줘"라고 부르는 게 아니라 자기 페이스대로 pose를 계속 발행하고, BT 노드는 최신 TF/토픽 값을 읽기만 한다.

---

## 5. 후속 지적 — "tick 종속성은 풀리지만, 데이터 유입 속도는?"

> 이러면 매 tick 관련 종속성(순차적 확인 절차)은 해소할 수 있겠지만, 정작 독립적으로 들어오는 **데이터의 속도**는 처리가 안 될 것 같은데.

정확한 지적. tick 종속성 제거와 처리량(throughput) 문제는 **완전히 다른 층위**이며, 노드를 독립시켜도 저절로 해결되지 않는다. 오히려 이제 명시적으로 설계해야 하는 상황이 된다.

### 실제로 존재하는 세 가지 속도

| 단계 | 속도 | 비고 |
|---|---|---|
| 카메라 프레임 레이트 | 15~30Hz | RealSense D455 |
| **VLM 추론** | **1~5Hz 수준** | **전체 파이프라인의 실질적 병목** (팀메이트 파트) |
| Depth unprojection + RANSAC + PCA | 포인트 수에 따라 가변 | non-trivial한 연산 비용 |

세 속도가 다르고 심지어 가변적(VLM 추론 시간이 프레임마다 다름)이므로, "매번 다 처리한다"는 전제로 설계하면 반드시 큐가 쌓이고 지연이 누적된다.

### 해결 — 큐(queue)가 아니라 "최신값 슬롯(latest-value slot)"

핵심은 **FIFO 큐를 쓰지 않는 것**. ROS2에서 흔한 실수가 subscriber queue_size를 크게 잡아 "밀린 데이터를 다 처리하려는" 설계인데, 여기서는 반대로 가야 한다.

- VLM ROI subscriber, depth image subscriber 모두 **`queue_size=1`** → 처리가 못 따라가면 오래된 메시지는 버리고 항상 최신 것만 유지
- Stabilizer 내부에 **busy flag** → PCA 연산 중 새 콜백이 들어오면 그 프레임은 스킵. 연산 큐가 절대 쌓이지 않고, 항상 "지금 처리 가능한 가장 최신 데이터"만 처리
- 실질 처리율은 `min(VLM 속도, PCA 연산 속도)`로 자연 수렴. 그보다 빠르게 들어오는 프레임은 버려지지만 이는 **손실이 아니라 의도된 동작**이다. 로봇은 매 프레임을 볼 필요가 없고, 처리 가능한 만큼만 최신 정보로 갱신되면 충분하다.

### 추가 고려 — VLM ROI와 depth 프레임의 시간 동기화

VLM 추론이 느리므로, ROI 결과가 나왔을 때 그 ROI는 "지금"이 아니라 **수백 ms 전 프레임 기준**이다. 이 ROI를 아무 최신 depth 프레임에 적용하면 안 된다.

→ `message_filters::ApproximateTimeSynchronizer`로 header timestamp 기준 매칭을 걸어, "VLM이 보고 있던 순간의 depth"와 정확히 맞춘다. 이걸 안 하면 **로봇이 접근 중일 때 ROI(과거)와 depth(현재)가 어긋나 법선 추정이 흔들리는 원인**이 된다.

### Stability 판단은 카운트가 아니라 시간 기반 window로

VLM 속도가 가변적이므로 "최근 N개 프레임"보다 **"최근 T초 동안의 관측치 분산"**으로 window를 잡는 것이 낫다. VLM이 빠른 구간에는 샘플이 많이, 느린 구간에는 적게 쌓이지만 판단 기준 자체는 흔들리지 않는다.

---

## 6. 마지막 질문 — 판단 시기를 어떻게 정할 것인가

> 어차피 로직은 같으니까, 정확하지 않아도 "법선 방향으로 접근한다"는 궁극적 목표가 VLM이 정확한 ROI를 제공한다는 조건 하에 성립한다는 것까지는 괜찮은 것 같다. 그럼 **판단 시기**는? 생각 중인 것은 시스템 시작과 동시에 모든 것을 켜 놓고, VLM이 탐지 가능한 물체를 인식하면 topic을 발행하게 해서 그걸 신호로 껐다 켜는 것. 근데 그러면 **VLM 인식 시스템이 매우 정확해야 한다는 문제**가 있다.

방향은 맞지만, "탐지되면 킴 → 안 되면 끔"을 **단일 프레임의 유/무로 바로 스위칭**하면 우려한 문제가 그대로 발생한다. 이는 **hysteresis(이력현상) 없는 on/off 스위치의 전형적 문제**다 — VLM이 한 프레임 잠깐 놓치거나 잘못 잡으면 stabilizer가 껐다 켜졌다 하면서 buffer가 계속 리셋되고, 결국 "누적 → 수렴 판단" 로직 자체가 무의미해진다.

### 해결 — 3단계 상태 머신 + 카운터 기반 전이

| 상태 | 의미 | 동작 |
|---|---|---|
| **IDLE** | VLM 미탐지 | Depth/PCA 연산 자체를 돌리지 않음 (연산 낭비 방지) |
| **CANDIDATE** | 탐지 신호는 있으나 아직 미신뢰 | buffer에 쌓기 시작하되 "탐지됨"으로 확정하지 않음 |
| **TRACKING** | 조건 만족, 확정 | BT 쪽에 "perception 값 있음" 통지 |

**전이 조건 (단일 프레임이 아닌 카운트/시간 기반)**

```
IDLE      → CANDIDATE : 탐지 신호 1회라도 발생 시 즉시
                        (연산 시작은 빠르게, 아직 신뢰는 안 하므로 무방)

CANDIDATE → TRACKING  : 최근 T초 탐지 성공률 ≥ 임계값(예: 70%)
                        AND pose 분산 수렴 조건 만족

TRACKING  → CANDIDATE : 연속 미탐 시간 > 임계값(예: 0.5~1초)
(또는 IDLE)             → 한두 프레임 놓치는 건 무시

CANDIDATE → IDLE      : CANDIDATE로 일정 시간 초과했으나 TRACKING 조건 미충족
                        → 애초에 신뢰 불가한 노이즈성 탐지로 판단
```

이렇게 하면 **"VLM이 완벽해야 한다"는 전제 자체가 불필요**해진다. 오히려 VLM의 순간적 오탐/미탐을 흡수하는 것이 이 상태 머신의 존재 이유다.

### "모든 걸 켜놓고 시작" 아이디어와의 결합

시스템 시작 시 모든 노드(카메라 드라이버, TF, stabilizer 프로세스)를 다 띄워놓는 것은 맞고, ROS2 노드 생명주기 관점에서도 자연스럽다. 다만 **stabilizer가 IDLE 상태에서 무거운 depth unprojection/RANSAC/PCA를 건너뛴다**는 조건만 넣으면, 노드는 항상 떠 있되 실제 연산 부하는 탐지 여부에 따라 자연 조절된다.

즉 "노드를 껐다 켰다"가 아니라 **"노드는 항상 살아있고 내부 상태만 전이"**시키는 것이 프로세스 재시작 오버헤드도 없고 훨씬 깔끔하다.

### BT 쪽 인터페이스

`Condition: IsObstacleTracking` 노드가 stabilizer 상태가 `TRACKING`인지만 확인. IDLE/CANDIDATE에서는 BT가 "아직 신뢰할 만한 장애물 정보 없음"으로 처리하고 원래 global path를 따라간다. VLM의 순간적 신뢰도 저하가 **BT 레벨까지 전파되지 않고 stabilizer 내부 상태 머신에서 흡수**된다.

이 구조는 SMMS 계열 논문들이 트래킹 시스템에서 detection confirmation과 track deletion을 카운터/시간 기반 hysteresis로 처리하는 **표준 패턴**과 동일하다.

---

## 최종 아키텍처 정리

```
┌─────────────────────────────────────────────────────────────┐
│  Perception Stabilizer Node  (BT와 독립, 항상 실행)           │
│                                                             │
│  [입력] queue_size=1                                        │
│    /camera/color/image_raw   ─┐                             │
│    /camera/depth/image_raw   ─┼ ApproximateTimeSync         │
│    /vlm/roi                  ─┘                             │
│                                                             │
│  [내부 상태 머신]  IDLE ⇄ CANDIDATE ⇄ TRACKING              │
│    · busy flag로 non-blocking 처리 (밀린 프레임 스킵)         │
│    · IDLE에서는 무거운 연산 skip                              │
│    · confidence 기반 rejection                              │
│    · point cloud rolling buffer 누적 → RANSAC + PCA         │
│    · 최근 T초 window 분산으로 수렴 판정                        │
│    · 미탐 시 extrapolation                                   │
│                                                             │
│  [출력]                                                      │
│    /obstacle_pose_stable                                    │
│    /obstacle_pose_confidence                                │
│    /perception_state  (IDLE/CANDIDATE/TRACKING)             │
└─────────────────────────────────────────────────────────────┘
                            │ subscribe (최신값만)
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Nav2 Behavior Tree                                         │
│                                                             │
│    Condition : IsObstacleTracking                           │
│    Action    : LockApproachPose   ← 여기서 freeze            │
│    Action    : ComputeApproachDirection                     │
│    Action    : NavigateToApproachPose                       │
│    Action    : PushObstacle       ← perception 입력 차단      │
│                (Nav2 controller 밖에서 실행)                  │
└─────────────────────────────────────────────────────────────┘
```

### 핵심 원칙 3줄 요약

1. **BT tick과 perception 주기는 애초에 분리한다** — BT가 perception을 호출하는 것이 아니라, AMCL처럼 독립 노드가 발행하는 최신값을 구독한다
2. **처리량 문제는 큐가 아니라 "최신값 슬롯 + busy flag"로 해결한다** — 밀린 프레임은 버리는 것이 의도된 동작이다
3. **VLM의 불완전함은 상태 머신의 hysteresis로 흡수한다** — VLM이 정확해야 시스템이 동작하는 구조를 만들지 않는다
