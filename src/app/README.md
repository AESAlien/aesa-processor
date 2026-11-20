# 실행 제어(`app`) 모듈

## 디렉터리 구조

```text
app/
├── include/
│   └── app/                 # .gitkeep만 있음
├── src/
│   ├── config/
│   ├── dto/
│   ├── log/
│   │   ├── logger/          # 모듈 이벤트를 로그 형식으로 변환
│   │   └── support/         # 공용 보조 코드
│   ├── operation/           # 운용 상태 전환 판정
│   ├── queue/               # 스레드 간 메시지 전달 큐
│   ├── worker/              # 컨트롤러 호출과 워커 입출력 처리
│   └── main.cpp             # 실행 진입점
└── test/
    ├── unit/                # 단위 테스트
    │   ├── operation/
    │   └── queue/
    └── integration/         # 통합 테스트
        └── worker/
```

## 실행 흐름

```mermaid
flowchart TB
    initializeLogging["1. 로깅 초기화"]
    registerMessageLoggers["2. 메시지 로거 등록"]
    registerCsvLogger["3. CSV 로거 등록"]
    registerShutdownSignalHandlers["4. 종료 시그널 핸들러 등록"]
    loadConfig["5. 설정 로드"]
    createInputQueues["6. 입력 큐 생성"]
    createControllers["7. 컨트롤러 생성"]
    createWorkers["8. 워커 생성"]
    openServer["9. 통신 서버 오픈"]
    startWorkers["10. 워커 실행"]
    waitForShutdownRequest["11. 종료 시그널 대기"]
    exit["12. 프로세스 종료"]

    initializeLogging --> registerMessageLoggers --> registerCsvLogger
    registerCsvLogger --> registerShutdownSignalHandlers --> loadConfig
    loadConfig --> createInputQueues --> createControllers --> createWorkers
    createWorkers --> openServer --> startWorkers --> waitForShutdownRequest
    waitForShutdownRequest -->|SIGINT 또는 SIGTERM| exit
```

## 워커 클래스 구성

```mermaid
classDiagram
    direction TB

    class Worker {
        <<abstract>>
        +run()
        #readInput()*
        #process(input)*
        #writeOutput(output)
    }

    Worker <|-- CommReceiveWorker
    Worker <|-- PoseWorker
    Worker <|-- BeamWorker
    Worker <|-- TargetWorker
    Worker <|-- ConsoleSendWorker
    Worker <|-- SimulatorSendWorker
    Worker <|-- TickWorker
```
