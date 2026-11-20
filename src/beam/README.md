# 빔 스케줄링(`beam`) 모듈

## 디렉터리 구조

```text
beam/
├── include/
│   └── beam/
│       ├── config/
│       ├── controller/      # 모듈 호출 진입점
│       ├── domain/          # 도메인 모델과 규칙
│       └── dto/
│           └── log/
├── src/
│   └── domain/
│       ├── request/         # 요청 빔 데이터와 대기열, 영역 판정
│       │   └── factory/
│       ├── scheduler/       # 자세 프레임과 송신 순서 관리
│       └── search/          # 탐색 빔 격자와 순회 상태
└── test/
    ├── unit/                # 단위 테스트
    │   ├── config/
    │   ├── domain/
    │   │   └── request/
    │   │       └── factory/
    │   └── dto/
    ├── integration/         # 통합 테스트
    │   ├── controller/
    │   └── domain/
    │       ├── request/
    │       ├── scheduler/
    │       └── search/
    ├── scenario/            # 테스트와 시각화 도구가 공유하는 시나리오
    └── tools/               # 데이터 출력과 시각화
```

## 데이터 흐름

```mermaid
flowchart LR
    CommReceiveWorker[수신<br/>스레드]
    PoseWorker[레이더 포즈<br/>스레드]
    TargetWorker[타겟 추적<br/>스레드]
    TickWorker[틱<br/>스레드]
    SimulatorSendWorker[모의기 송신<br/>스레드]

    subgraph BeamController
        reset["reset()"]
        setAntEnuTransform["setAntEnuTransform()"]
        requestBeam["requestBeam()"]
        nextBeamCommand["nextBeamCommand()"]
    end

    CommReceiveWorker -->|ON → OFF| reset
    PoseWorker -->|자세 좌표계 변환| setAntEnuTransform
    TargetWorker -->|빔 요청| requestBeam
    TickWorker -->|10ms 틱| nextBeamCommand
    nextBeamCommand -->|빔 송신 명령| SimulatorSendWorker

    classDef communication fill:#DCFCE7,stroke:#16A34A,color:#166534,stroke-width:2px
    classDef processing fill:#DBEAFE,stroke:#2563EB,color:#1E40AF,stroke-width:2px
    classDef utility fill:#F3E8FF,stroke:#9333EA,color:#6B21A8,stroke-width:2px
    classDef control fill:#F1F5F9,stroke:#64748B,color:#334155,stroke-width:2px

    class CommReceiveWorker,SimulatorSendWorker communication
    class TargetWorker processing
    class PoseWorker utility
    class TickWorker control
    style BeamController fill:#DBEAFE,stroke:#2563EB,color:#1E40AF,stroke-width:2px
```

## 도메인 클래스 구성

```mermaid
classDiagram
    direction TB

    class AntEnuTransform["pose::AntEnuTransform"]

    BeamScheduler *-- "0..1" AttitudeFrame
    BeamScheduler *-- RequestQueue

    AttitudeFrame *-- SearchGrid
    AttitudeFrame *-- RequestCoverage
    SearchGrid *-- "0..*" SearchBeam
    SearchGrid ..> AntEnuTransform : 탐색 빔 방향 변환
    RequestCoverage *-- AntEnuTransform

    RequestQueue *-- "0..*" BeamRequest
```

## 환경 설정

| 환경 변수 | 설정 용도 | 허용 범위 |
|---|---|---|
| `BEAM_SEARCH_AZIMUTH_COUNT` | 탐색 격자의 방위각 개수 | 1 이상 |
| `BEAM_SEARCH_ELEVATION_COUNT` | 탐색 격자의 고각 개수 | 1 이상 |
| `BEAM_SEARCH_AZIMUTH_START_DEG` | 탐색 격자 방위각 시작점 | 제한 없음 |
| `BEAM_SEARCH_AZIMUTH_STEP_DEG` | 방위각 인접 격자점 간 간격 | 제한 없음 |
| `BEAM_SEARCH_ELEVATION_START_DEG` | 탐색 격자 고각 시작점 | [-90°, 90°] |
| `BEAM_SEARCH_ELEVATION_STEP_DEG` | 고각 인접 격자점 간 간격 | 마지막 고각이 [-90°, 90°] |
| `BEAM_SEARCH_WIDTH_DEG` | 탐색 빔 폭 | 0° 초과 |
| `BEAM_CONFIRMATION_WIDTH_DEG` | 확인 빔 폭 | 0° 초과 |
| `BEAM_TRACKING_WIDTH_DEG` | 추적 빔 폭 | 0° 초과 |
| `BEAM_REQUEST_COVERAGE_AZIMUTH_MIN_DEG` | 요청 허용 방위각 시작 경계 | 두 경계 간 차이 0° 제외, 절댓값 ≤ 360° |
| `BEAM_REQUEST_COVERAGE_AZIMUTH_MAX_DEG` | 요청 허용 방위각 끝 경계 | 두 경계 간 차이 0° 제외, 절댓값 ≤ 360° |
| `BEAM_REQUEST_COVERAGE_ELEVATION_MIN_DEG` | 요청 허용 고각 하한 | [-90°, 90°); 하한 < 상한 |
| `BEAM_REQUEST_COVERAGE_ELEVATION_MAX_DEG` | 요청 허용 고각 상한 | (-90°, 90°]; 하한 < 상한 |
