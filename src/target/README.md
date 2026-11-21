# 타겟 추적(`target`) 모듈

## 디렉터리 구조

```text
target/
├── include/
│   └── target/
│       ├── config/
│       ├── controller/      # 모듈 호출 진입점
│       └── dto/
│           └── log/
├── src/
│   ├── domain/              # 도메인 모델과 규칙
│   │   ├── association/     # 관측과 트랙의 연관 판정
│   │   ├── factory/
│   │   └── tracking/        # 트랙 상태 추정과 위치 불확실성 계산
│   ├── dto/
│   │   └── factory/
│   ├── repository/          # 도메인 객체의 저장과 조회
│   └── service/             # 도메인 동작을 조합하는 처리 흐름
└── test/
    ├── unit/                # 단위 테스트
    │   ├── domain/
    │   │   └── association/
    │   └── repository/
    ├── integration/         # 통합 테스트
    │   ├── config/
    │   ├── controller/
    │   ├── domain/
    │   │   ├── association/
    │   │   └── tracking/
    │   └── service/
    └── support/             # 공용 보조 코드
```

## 모듈 입출력 흐름

```mermaid
flowchart LR
    CommReceiveWorker[수신<br/>스레드]
    PoseWorker[레이더 포즈<br/>스레드]
    TickWorker[틱<br/>스레드]
    BeamWorker[빔 스케쥴링<br/>스레드]
    ConsoleSendWorker[전시기 송신<br/>스레드]

    subgraph TargetController
        setEnuGeoTransform["setEnuGeoTransform()"]
        handleDetection["handleDetection()"]
        removeExpiredTracks["removeExpiredTracks()"]
        getTrackEvents["getTrackEvents()"]
    end

    CommReceiveWorker -->|탐지 결과| handleDetection
    PoseWorker -->|위치 좌표계 변환| setEnuGeoTransform
    handleDetection -->|빔 요청| BeamWorker
    TickWorker -->|10ms 틱| removeExpiredTracks
    TickWorker -->|10ms 틱| getTrackEvents
    getTrackEvents -->|추적 결과| ConsoleSendWorker

    classDef communication fill:#DCFCE7,stroke:#16A34A,color:#166534,stroke-width:2px
    classDef processing fill:#DBEAFE,stroke:#2563EB,color:#1E40AF,stroke-width:2px
    classDef utility fill:#F3E8FF,stroke:#9333EA,color:#6B21A8,stroke-width:2px
    classDef control fill:#F1F5F9,stroke:#64748B,color:#334155,stroke-width:2px

    class CommReceiveWorker,ConsoleSendWorker communication
    class BeamWorker processing
    class PoseWorker utility
    class TickWorker control
    style TargetController fill:#DBEAFE,stroke:#2563EB,color:#1E40AF,stroke-width:2px
```

## 클래스 다이어그램

```mermaid
classDiagram
    direction TB

    class DetectionResult {
        <<variant>>
    }

    TentativeTrack *-- Observation
    Observation "0..*" --* DetectionResult
    ConfirmedTrack *-- TrackState
    TentativeTrack ..> ConfirmedTrack : 생성

    TentativeTrack o-- ITrackingAlgorithm : 공유 소유
    ConfirmedTrack o-- ITrackingAlgorithm : 공유 소유

    IPositionUncertainty <|-- ConstantAccelerationAlgorithm
    ITrackingAlgorithm <|-- ConstantAccelerationAlgorithm
    ITrackingAlgorithm <|-- ConstantVelocityAlgorithm

    Observation <.. TentativeSelector : 선택
    Observation <.. TrackSelector : 선택
    TentativeSelector *-- IAssociationAlgorithm
    TrackSelector *-- IAssociationAlgorithm
    IAssociationAlgorithm <|-- NdsAlgorithm
```

```mermaid
classDiagram
    direction TB

    DetectionResultBase <|-- SearchDetectionResult
    DetectionResultBase <|-- ConfirmationDetectionResult
    DetectionResultBase <|-- TrackingDetectionResult
```

## 시퀀스 다이어그램

### 탐색 결과 처리

```mermaid
sequenceDiagram
    actor app
    participant TargetController
    participant DetectionHandler
    participant TrackRepository
    participant TentativeSelector
    participant TrackFactory

    app->>TargetController: handleDetection(event)
    TargetController->>DetectionHandler: handle(SearchDetectionResult)
    Note over DetectionHandler: requestId별 마지막 탐색 결과 시각 확인
    break 중복되거나 오래된 탐색 결과
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    DetectionHandler->>TrackRepository: tracks()
    TrackRepository-->>DetectionHandler: 기존 트랙 목록
    DetectionHandler->>TentativeSelector: selectNewCandidate(observations, tracks)
    Note over TentativeSelector: 기존 트랙과 연관되지 않는 첫 관측 선택
    TentativeSelector-->>DetectionHandler: 후보 관측 또는 후보 없음

    alt 새 후보가 있음
        DetectionHandler->>TrackFactory: createTentative(candidate)
        Note over TrackFactory: 새 ID를 발급해 잠정 트랙 생성<br/>팩터리 생성 시 받은 수명주기 설정 전달
        TrackFactory-->>DetectionHandler: ID가 부여된 TentativeTrack
        DetectionHandler->>TrackRepository: add(tentativeTrack)
        TrackRepository-->>DetectionHandler: id
        Note over DetectionHandler: 마지막 탐색 결과 시각 기록<br/>transmitTime + confirmationDelay에 확인 빔 요청
        DetectionHandler-->>TargetController: RequestBeamCommand(CONFIRMATION)
    else 새 후보가 없음
        Note over DetectionHandler: 후보가 없어도 마지막 탐색 결과 시각 기록
        DetectionHandler-->>TargetController: 빔 요청 없음
    end
    TargetController-->>app: 확인 빔 요청 또는 요청 없음
```

### 확인 결과 처리

```mermaid
sequenceDiagram
    actor app
    participant TargetController
    participant DetectionHandler
    participant TrackRepository
    participant TrackSelector
    participant TentativeTrack
    participant ConfirmedTrack

    app->>TargetController: handleDetection(event)
    TargetController->>DetectionHandler: handle(ConfirmationDetectionResult)
    DetectionHandler->>TrackRepository: findTentativeTrack(requestId)
    TrackRepository-->>DetectionHandler: 잠정 트랙 또는 트랙 없음
    opt 잠정 트랙이 있음
        DetectionHandler->>TentativeTrack: canProcessResult(transmitTime)
        TentativeTrack-->>DetectionHandler: 결과 처리 가능 여부
    end
    break 잠정 트랙이 없거나 결과를 처리할 수 없음
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    DetectionHandler->>TrackSelector: selectConfirmationObservation(track, transmitTime, observations)
    TrackSelector-->>DetectionHandler: 연관 관측 또는 관측 없음
    break 연관된 확인 관측이 없음
        DetectionHandler->>TrackRepository: removeById(requestId)
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    DetectionHandler->>TentativeTrack: confirm(observation)
    Note over TentativeTrack: 두 관측으로 추적 상태 초기화<br/>생성 시 받은 수명주기 설정 유지
    TentativeTrack-->>DetectionHandler: 확정 트랙 또는 초기화 실패
    break 추적 상태 초기화 실패
        DetectionHandler->>TrackRepository: removeById(requestId)
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    DetectionHandler->>TrackRepository: replace(confirmedTrack)
    Note over TrackRepository: ID와 등록 순서를 유지하며 확정 트랙으로 교체
    TrackRepository-->>DetectionHandler: 교체 성공 여부
    break 교체할 트랙이 없음
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    DetectionHandler->>TrackRepository: findConfirmedTrack(requestId)
    TrackRepository-->>DetectionHandler: confirmedTrack
    DetectionHandler->>ConfirmedTrack: predict(transmitTime + trackingInterval)
    ConfirmedTrack-->>DetectionHandler: 예측 결과 또는 예측 실패
    alt 예측 성공
        DetectionHandler-->>TargetController: RequestBeamCommand(TRACKING)
    else 예측 실패
        DetectionHandler->>TrackRepository: removeById(requestId)
        DetectionHandler-->>TargetController: 빔 요청 없음
    end
    TargetController-->>app: 첫 추적 빔 요청 또는 요청 없음
```

### 추적 결과 처리

```mermaid
sequenceDiagram
    actor app
    participant TargetController
    participant DetectionHandler
    participant TrackRepository
    participant TrackSelector
    participant ConfirmedTrack

    app->>TargetController: handleDetection(event)
    TargetController->>DetectionHandler: handle(TrackingDetectionResult)
    DetectionHandler->>TrackRepository: findConfirmedTrack(requestId)
    TrackRepository-->>DetectionHandler: 확정 트랙 또는 트랙 없음
    opt 확정 트랙이 있음
        DetectionHandler->>ConfirmedTrack: canProcessResult(transmitTime)
        ConfirmedTrack-->>DetectionHandler: 결과 처리 가능 여부
    end
    break 확정 트랙이 없거나 결과를 처리할 수 없음
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    DetectionHandler->>TrackSelector: selectTrackingObservation(track, transmitTime, observations)
    TrackSelector-->>DetectionHandler: 연관 관측 또는 관측 없음
    opt 연관된 관측이 없음
        DetectionHandler->>ConfirmedTrack: willExpireOnNextMiss()
        ConfirmedTrack-->>DetectionHandler: 이번 미탐으로 한도에 도달할지 여부
    end
    break 관측이 없고 이번 미탐으로 연속 미탐 한도에 도달
        DetectionHandler->>TrackRepository: removeById(requestId)
        DetectionHandler-->>TargetController: 빔 요청 없음
        TargetController-->>app: 빔 요청 없음
    end

    opt 연관된 관측이 있음
        DetectionHandler->>ConfirmedTrack: update(observation)
        Note over ConfirmedTrack: 성공 시 추적 상태 갱신과 연속 미탐 횟수 초기화
        ConfirmedTrack-->>DetectionHandler: 갱신 성공 여부
    end
    opt 관측이 없거나 상태 갱신 실패
        DetectionHandler->>ConfirmedTrack: recordTrackingMiss(transmitTime)
        Note over ConfirmedTrack: 마지막 유효 상태 유지<br/>결과 시각과 연속 미탐 횟수 갱신
        DetectionHandler->>ConfirmedTrack: hasReachedMissLimit()
        ConfirmedTrack-->>DetectionHandler: 연속 미탐 한도 도달 여부
        break 연속 미탐 한도에 도달
            DetectionHandler->>TrackRepository: removeById(requestId)
            DetectionHandler-->>TargetController: 빔 요청 없음
            TargetController-->>app: 빔 요청 없음
        end
    end

    DetectionHandler->>ConfirmedTrack: predict(transmitTime + trackingInterval)
    ConfirmedTrack-->>DetectionHandler: 예측 결과 또는 예측 실패
    alt 예측 성공
        Note over DetectionHandler: 미탐 허용 중에도 마지막 유효 상태로 다음 빔 방향 예측
        DetectionHandler-->>TargetController: RequestBeamCommand(TRACKING)
    else 예측 실패
        DetectionHandler->>TrackRepository: removeById(requestId)
        DetectionHandler-->>TargetController: 빔 요청 없음
    end
    TargetController-->>app: 다음 추적 빔 요청 또는 요청 없음
```

### 트랙 만료

```mermaid
sequenceDiagram
    actor app
    participant TargetController
    participant TrackExpirationHandler
    participant TrackRepository
    participant Track as TentativeTrack / ConfirmedTrack

    Note over app: TargetWorker가 TickEvent에서 먼저 만료 처리
    app->>TargetController: removeExpiredTracks(currentTime)
    TargetController->>TrackExpirationHandler: removeExpiredTracks(currentTime)
    TrackExpirationHandler->>TrackRepository: tracks()
    TrackRepository-->>TrackExpirationHandler: 잠정 트랙과 확정 트랙 목록
    loop 각 트랙
        TrackExpirationHandler->>Track: isExpired(currentTime)
        Note over Track: 자신의 마지막 결과 시각과 트랙 종류에 따른 대기 한도로 판단
        Track-->>TrackExpirationHandler: 만료 여부
        opt 트랙이 만료됨
            Note over TrackExpirationHandler: 만료된 트랙 ID 수집
        end
    end
    loop 수집한 ID마다
        TrackExpirationHandler->>TrackRepository: removeById(id)
    end
    TrackExpirationHandler-->>TargetController: 삭제된 트랙 ID 목록
    TargetController-->>app: 만료 처리 완료
```

### 트랙 보고

```mermaid
sequenceDiagram
    actor app
    participant TargetController
    participant TrackReporter
    participant TrackRepository
    participant EnuGeoTransform

    Note over app: TargetWorker가 같은 TickEvent에서 만료 처리 후 보고 목록 요청
    app->>TargetController: getTrackEvents()
    TargetController->>TrackReporter: getTrackEvents()
    alt ENU와 지리 좌표 간 변환이 설정되지 않음
        TrackReporter-->>TargetController: 빈 목록
    else 좌표 변환이 설정됨
        TrackReporter->>TrackRepository: confirmedTracks(maxReportedTrackCount)
        TrackRepository-->>TrackReporter: 보고 상한 내의 확정 트랙 목록
        loop 각 확정 트랙
            TrackReporter->>EnuGeoTransform: enuToGeo(track.state().position())
            EnuGeoTransform-->>TrackReporter: 위도와 경도, 타원체고
            Note over TrackReporter: TrackEventFactory.from(track, position)으로 보고 항목 생성
        end
        TrackReporter-->>TargetController: TrackEvent 목록
    end
    TargetController-->>app: TrackEvent 목록
    Note over app: 빈 목록도 전시기 입력 큐에 전달<br/>ConsoleSendWorker가 sendTrackEvents() 호출
```

## 설정 구성

`TargetConfig`는 다음 설정을 조립한다.

| 설정 | 책임 |
|---|---|
| `AssociationConfig` | 관측과 트랙의 연관 판정 기준 |
| `MeasurementConfig` | 연관 계산과 추적 알고리즘이 공유하는 측정 오차 |
| `TrackingConfig` | 추적 알고리즘의 초기 가속도 오차와 과정 잡음 |
| `TrackLifecycleConfig` | 확인 지연, 추적 주기, 최대 연속 미탐 횟수 |
| `TrackReportingConfig` | 전시기 보고 개수와 프로토콜 상한 검증 |

`TrackFactory`는 수명주기 설정을 트랙 생성 시 전달한다. 잠정 트랙이 확정 트랙으로 전환되어도 같은 설정을 유지하며, `isExpired(currentTime)`은 보관한 설정으로 만료 여부를 판단한다. `TrackReporter`는 보고 설정만 전달받는다.

## 환경 설정

| 환경 변수 | 설정 용도 | 허용 범위 |
|---|---|---|
| `TARGET_MAX_ASSOCIATION_SPEED_MPS` | 관측과 기존 트랙을 연관할 때 허용하는 최대 속도 | 0 이상 |
| `TARGET_MAX_INITIAL_DOPPLER_DIFFERENCE_MPS` | 잠정 트랙 확인 시 허용하는 초기 도플러 속도 차이 | 0 이상 |
| `TARGET_MEASUREMENT_STANDARD_DEVIATION_M` | 위치 관측의 표준 편차 | 0 초과 |
| `TARGET_POSITION_NDS_THRESHOLD` | 위치 연관 NDS 점수 임계값 | 0 이상 |
| `TARGET_CONFIRMATION_DELAY_MS` | 확인 빔 요청까지의 지연 시간 | 0 ms 초과 |
| `TARGET_TRACKING_INTERVAL_MS` | 다음 추적 빔 요청까지의 간격. 연속 미탐 한도와 곱해 응답 대기 한도로도 사용 | 0 ms 초과 |
| `TARGET_MAX_CONSECUTIVE_TRACKING_MISSES` | 트랙을 유지할 최대 연속 미탐 수. 추적 주기와 곱해 응답 대기 한도로도 사용 | 1 이상 |
| `TARGET_INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2` | 초기 가속도 추정의 표준 편차 | 0 초과 |
| `TARGET_MAX_REPORTED_TRACK_COUNT` | 전시기에 보고할 최대 트랙 수 | 1–100 |
| `TARGET_PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2` | 추적 상태 예측에 사용하는 가속도 잡음 표준 편차 | 0 이상 |
