# 레이더 포즈 모듈(`pose`)

## 디렉터리 구조

```text
pose/
├── include/
│   └── pose/
│       ├── config/
│       ├── controller/      # 모듈 호출 진입점
│       ├── domain/          # 도메인 모델과 규칙
│       │   └── factory/
│       └── dto/
├── src/
└── test/
    ├── unit/                # 단위 테스트
    │   ├── config/
    │   └── domain/
    │       └── factory/
    └── integration/         # 통합 테스트
        └── controller/
```

## 데이터 흐름

```mermaid
flowchart LR
    CommReceiveWorker[수신<br/>스레드]
    BeamWorker[빔 스케줄링<br/>스레드]
    TargetWorker[타겟 추적<br/>스레드]
    ConsoleSendWorker[전시기 송신<br/>스레드]

    subgraph PoseController
        setRadarPose["setRadarPose()"]
    end

    CommReceiveWorker -->|위치 + 자세| setRadarPose
    setRadarPose -->|자세 좌표계 변환| BeamWorker
    setRadarPose -->|위치 좌표계 변환| TargetWorker
    setRadarPose -->|위치 + 자세| ConsoleSendWorker

    classDef communication fill:#DCFCE7,stroke:#16A34A,color:#166534,stroke-width:2px
    classDef processing fill:#DBEAFE,stroke:#2563EB,color:#1E40AF,stroke-width:2px

    class CommReceiveWorker,ConsoleSendWorker communication
    class BeamWorker,TargetWorker processing
    style PoseController fill:#F3E8FF,stroke:#9333EA,color:#6B21A8,stroke-width:2px
```

## 도메인 클래스 구성

```mermaid
classDiagram
    direction TB

    AntEnuTransform ..> RadarAttitude : 변환 생성 입력
    EnuGeoTransform *-- GeoPosition : ENU 원점
```

## 환경 설정

| 환경 변수 | 설정 용도 | 허용 범위 |
|---|---|---|
| `POSE_MAX_ABS_PITCH_DEG` | 절대 pitch의 상한 | (0°, 90°] |
| `POSE_MOUNT_ANT_TO_BODY_ROTATION` | 안테나 좌표계에서 몸체 좌표계로의 장착 회전 행렬 | 3×3 회전 행렬 |
