# AI Festa 구성의 역할

- `../src/ai-festa-map.usd`, `.png`, `.yaml`: 대회 예상 환경을 단순화한 공통 simulation environment.
- `ai_festa_navigation`: 이 환경에서 일반 ROS2 Humble Nav2 주행을 검증한 **Basic Nav2 baseline**. 메인 기능 개발 프로젝트가 아니다.
- `../src/custom_nav2_bt_plugins` 등: 향후 custom BT/plugin과 메인 기능 개발 위치. 같은 AI Festa 환경에서 검증한다.

baseline은 custom 기능 통합 중 기본 Nav2·Isaac·map 연결을 비교하는 기준으로 유지한다.
폴더와 맵 위치는 옮기지 않는다. 실행 중인 baseline과 custom Nav2를 중복 실행하지 않는다.

환경 설정, 선택 빌드, WebRTC Isaac, Windows X11 RViz, 초기 위치와 Goal 예시는
[ai_festa_cmd_line.txt](../ai_festa_cmd_line.txt)를 참고한다.
