# app 모듈

각 채널에 이벤트 큐 하나와 워커 하나를 둡니다. 전체 큐와 워커는 각각 세 개입니다.

| 채널 | 같은 큐에 저장하는 DTO | 워커 |
| --- | --- | --- |
| BeamChannel | SetOperationStateCommand, SetAttitudeCommand, BeamRequest | BeamWorker |
| CommChannel | BeamCommand, vector<TrackEvent> | CommWorker |
| TargetChannel | DetectionEvent | TargetWorker |

각 채널의 이벤트는 std::variant로 정의합니다. 워커는 std::visit으로 DTO에 맞는
Handler::process 오버로드를 호출합니다. 각 채널 안에서는 입력 순서가 유지되며,
채널 사이에는 전역 처리 순서가 없습니다. 세 워커는 병렬로 실행됩니다.
단일 TrackEvent를 CommChannel에 쓰면 한 원소의 목록으로 묶어 같은 큐에 넣습니다.
독립 자세 채널과 워커는 Beam에 통합했습니다.

## 생산자와 소비자

write로 메시지를 넣는 코드가 생산자이며 read로 꺼내는 워커가 소비자입니다.
하나의 워커가 자기 채널에서 읽고 다른 채널에 처리 결과를 쓸 수 있습니다.
별도 생산자 워커 클래스는 필요하지 않습니다. 실행 예제에서는 main이 입력을 생성하고,
테스트에서는 Target → Beam → Comm으로 전달하는 파이프라인도 검증합니다.

현재 컨트롤러는 선언만 있으므로 실행 예제의 TestApi는 메시지를 출력합니다.
실제 모듈 연결은 각 Handler 구현에서 다음 메소드를 호출하도록 구성합니다.

- BeamHandler: setOperationState, setAttitude, requestBeam
- CommHandler: sendBeamCommand, sendTrackEvents
- TargetHandler: handleDetection

실제 네트워크 수신 루프는 생산자 역할로 BeamChannel과 TargetChannel에 입력을 전달합니다.
본 예제에는 네트워크 수신 루프와 실제 컨트롤러 구현이 포함되어 있지 않습니다.

## 큐 종료

read는 메시지 또는 종료를 기다리고, 닫힌 큐가 비면 nullopt를 반환합니다.
`write()`의 반환형은 `void`이며, `close()` 이후 호출하면 내부
`MessageQueue::push()`에서 `app::ChannelClosedError`를 던집니다.
종료 여부를 반환값으로 확인할 수 없으며, 기존 메시지는 계속 꺼낼 수 있습니다.
수신 시각 rxTimeNs는 steady_clock 기준 나노초이며 DTO의 timestamp와 별개입니다.

외부 생산자를 먼저 멈춘 뒤 실제 생산 흐름에 따라 채널을 닫고 워커 종료를 기다립니다.
예제의 종료 순서는 Target 종료 → Beam 종료 → Comm 종료입니다.
하위 채널을 너무 일찍 닫으면 상위 워커의 `write()`에서 `app::ChannelClosedError`가 발생합니다.
종료와 쓰기가 겹칠 수 있는 생산자는 이 예외를 처리해야 합니다.
이벤트 생산 경로가 순환하는 경우에는 별도 종료 프로토콜이 필요합니다.
워커 예외 발생 시 예제는 모든 채널을 닫고 실패를 반환합니다.

## 빌드와 테스트

프로젝트 루트에서 실행합니다.

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure -R app_channel_test
```

app_channel_test는 채널별 혼합 DTO 순서, 값 보존, 동시 생산자 4개의
400개 이벤트 파이프라인, 독립적인 채널 대기와 종료 동작을 검증합니다.
MessageQueue는 헤더에 구현된 템플릿이므로 별도 message_queue.cpp가 필요하지 않습니다.
