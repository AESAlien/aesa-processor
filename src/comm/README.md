# TCP 통신(`comm`) 모듈

## 디렉터리 구조

```text
comm/
├── include/
│   └── comm/
│       ├── config/
│       ├── controller/      # 모듈 호출 진입점
│       ├── dto/
│       └── error/
├── src/
│   ├── message/             # 프레임 분리, 메시지 인코딩과 디코딩
│   │   └── dto/
│   ├── network/             # TCP 서버, 상대 연결, 세션 관리
│   │   ├── dto/
│   │   └── error/
│   ├── protocol/            # 전송 형식을 정의하는 메시지 구조체
│   └── socket/              # 소켓 핸들 관리와 플랫폼 API 래퍼
└── test/
    ├── unit/                # 단위 테스트
    │   ├── config/
    │   ├── message/
    │   └── socket/
    ├── integration/         # 통합 테스트
    │   └── socket/
    ├── support/             # 공용 보조 코드
    │   └── native_socket/
    ├── comm_main.cpp        # 수동 통신 테스트
    └── comm_sender.cpp      # 수동 송신 테스트
```

## 네트워크 클래스 구성

```mermaid
classDiagram
    direction TB
    class NativeSocket {
        <<functions>>
    }

    NetworkManager *-- "0..1" TcpServer
    NetworkManager *-- "2" PeerConnection
    PeerConnection o-- "0..1" Session : 공유 소유
    TcpServer *-- ListenSocket
    TcpServer ..> Session : 생성
    Session *-- ConnectedSocket
    Session *-- MessageFramer
    Socket <|-- ListenSocket
    Socket <|-- ConnectedSocket
    Socket ..> NativeSocket : 호출
```

## 시퀀스 다이어그램

### 외부 메시지 수신

```mermaid
sequenceDiagram
    actor app
    participant CommController
    participant NetworkManager
    participant PeerConnection
    participant MessageDecoder
    app->>CommController: receiveData()
    loop 대기열이 비어 있는 동안
        CommController->>NetworkManager: poll(timeout)
        opt 읽을 수 있는 연결이 있을 때
            NetworkManager->>PeerConnection: receive()
            Note over PeerConnection: Session.receive()에서 소켓 바이트를 읽고 프레임 조립
            PeerConnection-->>NetworkManager: 완성된 프레임 목록
        end
        NetworkManager-->>CommController: 완성된 프레임 묶음
        loop 완성된 프레임마다
            CommController->>MessageDecoder: decode(frame)
            MessageDecoder-->>CommController: DecodeResult
            CommController->>CommController: 디코딩한 메시지를 대기열에 추가
        end
    end
    CommController-->>app: 대기열의 첫 IncomingDto
```

### 외부 메시지 송신

```mermaid
sequenceDiagram
    actor app
    participant CommController
    participant NetworkManager
    participant PeerConnection
    participant Session
    participant ConnectedSocket
    participant SocketSelector

    app->>CommController: send...(...)
    Note over CommController: 메시지 종류에 맞게 바이트로 인코딩
    CommController->>NetworkManager: sendTo...(bytes)
    NetworkManager->>PeerConnection: send(data, timeout)
    PeerConnection->>Session: send(data, timeout)

    loop 보낼 바이트가 남아 있는 동안
        Session->>ConnectedSocket: sendBytes(data, bytesSent)
        alt 바이트를 보냈을 때
            ConnectedSocket-->>Session: 이번에 보낸 바이트 수
            Session->>Session: bytesSent에 송신량 누적
        else 쓰기 가능 상태를 기다려야 할 때
            Session->>ConnectedSocket: waitWritableUntil(deadline)
            ConnectedSocket->>SocketSelector: waitWritable(remaining)
            SocketSelector-->>ConnectedSocket: 쓰기 가능한 소켓 목록
        end
    end
```

## 환경 설정

| 환경 변수 | 설정 용도 | 허용 범위 |
|---|---|---|
| `COMM_SERVER_ADDRESS` | 서버 바인딩 주소 | IPv4 주소 |
| `COMM_SERVER_PORT` | 서버 수신 대기 포트 | 1–65535 |
| `COMM_OPERATOR_CONSOLE_ADDRESS` | 전시기 연결의 출발지 주소 | IPv4 주소 |
| `COMM_OPERATOR_CONSOLE_PORT` | 전시기 연결의 출발지 포트 | 1–65535 |
| `COMM_SCENARIO_SIMULATOR_ADDRESS` | 모의기 연결의 출발지 주소 | IPv4 주소 |
| `COMM_SCENARIO_SIMULATOR_PORT` | 모의기 연결의 출발지 포트 | 1–65535 |
