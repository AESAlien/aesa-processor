# AESA Wireshark Lua 디섹터

`aesa.lua`는 `src/comm/src/protocol/ST_*.hpp`와 메시지 인코더·디코더의 TCP 와이어 형식을 분석합니다.
공통 헤더, 빔 명령, 플롯, 트랙, 레이더 자세, 삭제 트랙 ID를 필터 가능한 필드로 표시합니다.
프로그램은 평소처럼 실행하고 Wireshark에서 해당 네트워크 인터페이스를 캡처하면 됩니다.

## Windows 설치

저장소 루트에서 PowerShell로 실행합니다.

```powershell
./tools/wireshark/install.ps1
```

스크립트는 `aesa.lua`를 `%APPDATA%/Wireshark/plugins/aesa.lua`에 복사합니다.
다시 실행하면 설치된 사본을 갱신합니다. Wireshark를 다시 실행하거나
**Analyze → Reload Lua Plugins**를 선택합니다. 로딩 여부는
**Help → About Wireshark → Plugins**에서 `aesa.lua`로 확인할 수 있습니다.
개인 Lua 플러그인 경로는 같은 창의 **Folders → Personal Lua Plugins**에서 확인합니다.
다른 경로를 쓰는 환경은 `-PluginDirectory '경로'`를 지정합니다.

설치를 취소하려면 위 개인 플러그인 폴더의 `aesa.lua`만 삭제하고 Wireshark를 다시 실행합니다.

## 프로그램 통신 캡처

1. 프로그램과 콘솔·시뮬레이터가 통신하는 Ethernet/Wi-Fi 인터페이스를 선택합니다.
   `127.0.0.1`을 사용하는 로컬 통신은 Npcap의 loopback 인터페이스를 선택합니다.
2. 캡처 필터에 `tcp port 3200`을 입력하고 캡처를 시작합니다.
3. 프로그램 및 상대 장비에서 통신을 발생시킵니다.
4. 표시 필터에 `aesa`를 입력합니다. 패킷 상세의 **AESA Processor**를 펼쳐 필드를 확인합니다.

`3200`은 `.env.example`의 `COMM_SERVER_PORT`입니다. 실제 `.env`에서 변경했다면
캡처 필터와 **Edit → Preferences → Protocols → AESA → TCP ports**도 변경합니다.
여러 포트는 `3200,43200`처럼 지정할 수 있습니다. 선택한 TCP 패킷에서
**Analyze → Decode As… → TCP port → AESA**로 수동 지정할 수도 있습니다.
서버 포트 등록 하나로 송수신 양쪽이 분석되므로, 상대 장비 포트 `3100`, `3300`은 별도 등록이 필요 없습니다.

TCP 설정의 **Allow subdissector to reassemble TCP streams**를 활성화해야
여러 TCP 세그먼트에 걸친 메시지를 재조립할 수 있습니다. 하나의 세그먼트에 여러 메시지가 담긴 경우도 처리합니다.
재조립된 메시지는 마지막 조각을 포함한 패킷에 표시될 수 있습니다.

프로그램의 `NetworkManager`는 상대 IP와 포트를 `.env` 설정과 모두 비교합니다.
직접 테스트 클라이언트를 만들 경우 콘솔·시뮬레이터의 **출발지 IP/포트**도 설정과 일치해야 연결이 유지됩니다.

## 표시 필터 예시

| 목적 | 표시 필터 |
| --- | --- |
| 전체 AESA 메시지 | `aesa` |
| 운용 시작 명령 | `aesa.message_id == 0xca01 && aesa.operation == 1` |
| 빔 송신 | `aesa.message_id == 0xab01` |
| 빔 ID 추적 | `aesa.beam.id == 21` |
| 플롯 정보 | `aesa.message_id == 0xba01` |
| 특정 트랙 | `aesa.track.id == 42` |
| 탐지 없는 플롯 메시지 | `aesa.plot.count == 0` |
| 삭제 트랙 ID | `aesa.deletion.id == 42` |
| 잘못된 메시지 길이·항목 수 | `aesa.malformed` |
| 알 수 없는 메시지 ID | `aesa.unknown_message` |

배열의 필드는 같은 메시지 내에서 여러 번 나타날 수 있습니다.
`aesa.track.id == 42 && aesa.track.velocity_mps > 100`은 두 조건이 같은 배열 원소에 속함을 보장하지 않으므로,
개별 트랙의 값은 상세 트리에서 함께 확인합니다.

## 지원 메시지와 길이

| ID | 메시지 | 전체 길이 (바이트) |
| --- | --- | --- |
| `0xCA01` | 운용 제어 | 24 |
| `0xAB01` | 빔 송신 | 80 |
| `0xBA01` | 플롯 정보 | `84 + 24 × 플롯 수` |
| `0xAC01` | 트랙 정보 | `24 + 88 × 트랙 수` |
| `0xAC02` | 콘솔로 보내는 레이더 자세 | 68 |
| `0xBA02` | 시뮬레이터에서 받은 레이더 자세 | 68 |
| `0xAC03` | 트랙 삭제 | 구조체 선언은 224; 분석용으로 `24 + 2 × 삭제 수`도 허용 |

- 공통 헤더는 20바이트이며 offset 4의 `block_size`는 **헤더를 포함한 전체 길이**입니다.
- 현재 C++ 코드는 바이트 순서 변환 없이 packed 구조체를 `memcpy`합니다.
  디섹터는 Windows/x86 계열 실행 환경의 **little-endian** 형식을 해석합니다.
- 현재 송신 인코더는 헤더 version을 `0`으로 전송하고 수신 디코더는 version을 검사하지 않습니다.
  디섹터도 version을 그대로 표시하며 `0` 또는 `1`을 이유로 거부하지 않습니다.
- 플롯·트랙·삭제 개수는 최대 100개입니다. 트랙·플롯은 최대 배열 크기를 항상 전송하는 방식이 아니라
  실제 개수에 따른 메시지 길이를 사용합니다.
- `TRACK_DELETION`은 구조체와 ID가 정의되어 있지만 현재 앱의 송신 인코더·수신 디코더에서 처리하지 않습니다.
  디섹터의 지원은 해당 패킷의 구조 분석을 위한 것입니다.
- 빔·트랙 타임스탬프는 ms, 거리·고도는 km, 각도는 deg, 속도는 m/s로 표시합니다.
  공통 헤더의 시간·장비 ID는 원래 값으로 표시하며 현재 인코더는 이 값을 0으로 채웁니다.
- 길이·개수 오류는 Expert Info에 표시합니다. 이는 앱의 유한값·도메인 검증 전체를 재현하는 검증기가 아닙니다.
  잘린 캡처, 유실된 TCP 조각, 메시지 중간부터 시작한 캡처는 재조립할 수 없을 수 있습니다.
  `block_size`가 20 미만 또는 기본 프레이머 상한 65536 초과이면 현재 버퍼를 오류로 표시합니다.
  동기화 마커가 없으므로 그 버퍼 안의 다음 메시지 경계를 추측하지 않습니다.

## TShark 검증과 샘플

Python 표준 라이브러리와 Lua를 지원하는 TShark만 필요합니다. TShark 4.6.8 / Lua 5.4에서 검증했습니다.

```powershell
python tools/wireshark/test_aesa.py
# TShark가 다른 곳에 설치된 경우
python tools/wireshark/test_aesa.py --tshark 'D:/Wireshark/tshark.exe'
# 실제 장비 없이 열어볼 수 있는 합성 패킷
python tools/wireshark/test_aesa.py --write-sample build/aesa_sample.pcap
```

테스트는 임시 설정 디렉터리를 사용해 개인 Wireshark 설정에 영향을 주지 않으며,
7종 메시지·부호 있는 실수·양방향 트래픽·분할 헤더/본문·여러 메시지 병합·0/100개 배열·
길이/개수 오류·알 수 없는 ID·포트 설정·Decode As를 실제 TShark에서 검증합니다.

설치 없이 Lua를 직접 지정해서 분석할 수도 있습니다. **이미 설치했다면 `-X`는 빼야** 같은 프로토콜이 중복 등록되지 않습니다.

```powershell
& 'C:/Program Files/Wireshark/tshark.exe' -X 'lua_script:tools/wireshark/aesa.lua' -r build/aesa_sample.pcap -Y aesa
```

공식 참고: [Lua 플러그인 폴더](https://www.wireshark.org/docs/wsug_html_chunked/ChPluginFolders.html),
[TCP PDU 재조립 Lua API](https://www.wireshark.org/docs/wsdg_html_chunked/lua_module_Proto.html#lua_fn_dissect_tcp_pdus_tvb__tree__min_header_size__get_len_func__dissect_func___desegment__).
