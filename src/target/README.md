# Target 모듈

TargetController는 외부 탐지 결과의 변환·핸들러 호출과 추적 정보 보고를 담당하는 구체 클래스다. DetectionHandler가 검증된 결과의 빔 타입별 수명주기 처리와 다음 빔 요청 생성을 담당한다.
TrackRepository 하나가 list<variant<TentativeTrack, Track>>으로 잠정 표적과 추적 표적을 등록 순서대로 보관한다.
신규 등록은 SEARCH에서만 수행하며, CONFIRMATION/TRACKING에서는 요청한 표적 하나만 관리한다.

## 내부 디렉터리 구조

내부 구현은 객체의 역할을 기준으로 나눈다.

| 디렉터리 | 책임 |
|---|---|
| src/config | TargetConfig 조합과 하위 설정별 검증 |
| src/controller | TargetController의 입력 경계·핸들러 연결·추적 정보 보고 |
| src/domain | 관측·표적·추적 상태와 ITrackingAlgorithm 계약 |
| src/domain/factory | 검증된 탐지 결과·관측·표적 생성 |
| src/repository | 표적 보관·조회·교체와 처리 시각 관리 |
| src/service | DetectionHandler의 빔 결과 처리와 Matcher·Selector의 연관·선택 정책 |
| src/algorithm | NdsAlgorithm의 NDS 계산과 CA·CV의 구체적인 추적 계산 |

TentativeTrack과 Track은 domain/tracking_algorithm.hpp의 인터페이스에 연산을 위임한다. algorithm의 구현체는 이 계약을 구현하며, Controller에서 생성하여 주입한다. 팩토리는 표적 내부 알고리즘을 직접 호출하지 않으며, 도메인은 구체적인 추적 알고리즘 구현에 의존하지 않는다.

## 외부 DTO와 내부 도메인 경계

공개 메서드와 DTO 선언은 유지한다.

    std::optional<beam::RequestBeamCommand> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;

app과 테스트는 공개 헤더 include/target/controller/target_controller.hpp의 TargetController를 TargetConfig로 직접 생성한다. TargetWorker는 기존처럼 TargetController 참조를 받는다. 탐지 처리 핸들러는 내부 헤더 src/service/detection_handler.hpp에 선언한다.

DetectionEvent 하나는 완료된 빔 하나의 결과다.

- beamType: SEARCH / CONFIRMATION / TRACKING.
- requestId: SEARCH에서는 그리드 ID, CONFIRMATION/TRACKING에서는 요청한 Target ID.
- transmitTime: 공유 시작점 기준 실제 빔 송신 시각(milliseconds).
- plots: 해당 빔의 모든 탐지. 탐지가 없는 완료 결과는 빈 목록으로 전달한다.

Controller의 입력 경계에서 DetectionResultFactory::from()이 DetectionEvent를 검증된 결과로 변환한다.
DetectionResultBase는 요청 ID·송신 시각·관측 목록을 const 필드로 소유한다. DetectionResult는 세 결과 하위 타입을 담는 variant이며, 빔 종류는 하위 타입으로 구분한다.
Factory는 SearchDetectionResult, ConfirmationDetectionResult, TrackingDetectionResult 중 하나를 반환하고,
Controller는 검증된 결과를 DetectionHandler::handle()에 전달하고, 핸들러는 std::visit으로 결과 타입에 맞는 처리를 연결한다. 하위 타입은 Factory로만 생성할 수 있다.
결과 객체의 생성자는 DetectionEvent를 받지 않는다. DetectionResultFactory가 DTO의 요청 ID·송신 시각을 추출하고 ObservationFactory로 전체 관측 목록을 만든 뒤 하위 타입 생성자에 전달한다. 결과 객체는 빔 타입 필드를 갖지 않으며, DTO와 ObservationFactory에 의존하지 않는다.

결과 생성자는 송신 시각과 요청 ID를 검사한다. SEARCH의 ID는 0도 허용하고 확인·추적 ID는 0일 수 없다.
알 수 없는 빔 타입은 Factory에서 거부한다.
ObservationFactory는 모든 Plot을 Observation으로 변환한다.
Observation 생성자는 양수 거리·비음수 시각·유한한 신호 세기를 검사한다.
거리·각도·도플러의 NaN·무한대는 math 물리량 생성 단계에서 거부된다.
모든 관측의 생성이 끝난 뒤 저장소를 처리하므로 뒤쪽 Plot이 잘못되어도 일부만 등록되거나 갱신되지 않는다.
Selector와 Matcher는 검증된 관측을 받으며, DTO나 알고리즘 의존성을 관측 객체에 넣지 않는다.

반환값은 빔 요청 0개(nullopt) 또는 1개다. RequestBeamCommand.requestId에는 등록된 표적 ID를 넣는다.
메시지 파싱·패키징·직렬화는 Comm의 역할이다.
Track::toTrackEvent()가 추적 상태를 보고 DTO로 변환하며 Controller는 등록 순서와 보고 한도를 적용한다.

## 관측과 표적의 수명주기

기존 UNREGISTERED 상태는 ID와 알고리즘이 없는 불변 Observation으로 표현한다.
SEARCH의 신규 후보는 DetectionHandler가 자신이 소유한 TrackIdGenerator.nextId()로 다음 ID를 조회하고
TrackFactory로 TentativeTrack을 생성한 뒤 Repository.add(tentativeTrack)에 전달한다.
Repository.add(track)으로 이미 생성된 Track도 저장할 수 있다.
저장소는 ID의 중복 여부를 검사한다. ID 발급 순서나 콘솔 ID 범위는 저장소에서 검사하지 않는다.
DetectionHandler는 저장에 성공한 뒤에만 TrackIdGenerator.commit()으로 다음 ID를 진행시킨다.
TentativeTrack은 최초 관측과 알고리즘을 소유한다. initialize(observation)은 시각을 검사한 뒤 알고리즘에 초기화를 위임하고, 반환 시각 계약을 검증한 추적 상태를 반환한다. confirm(observation)은 initialize()를 한 번 호출하고 성공한 상태로 TrackFactory에 Track 생성을 요청한다. 두 연산 모두 최초 관측을 변경하지 않는다.
Track은 항상 추적 상태를 소유하고 update(), predict(), recordTrackingMiss()를 제공한다.
TargetState 분기나 추적 상태 존재 여부로 객체의 역할을 판단하지 않는다.

    DetectionEvent → DetectionResultFactory → ObservationFactory → Observation 목록
      → 요청 ID·송신 시각·관측 목록으로 타입별 DetectionResult 생성
      → SEARCH의 신규 후보: TentativeTrack 등록
      → CONFIRMATION의 연관 성공: 같은 ID의 Track으로 전환
      → TRACKING의 연관 성공: Track 갱신
      → RequestBeamCommand / TrackEvent

DetectionHandler가 TrackFactory.from(tentative, observation)으로 계산·검증·할당을 마친 Track을 생성한다.
계산 중 예외가 발생하면 저장된 잠정 표적은 그대로 유지한다. 결과가 없으면 DetectionHandler가 기존 정책대로 삭제한다.
생성에 성공하면 Repository.replace(track)이 같은 ID의 객체를 기존 list 노드 안에서 교체한다.
replace()는 관측이나 추적 알고리즘을 받지 않으며, 해당 ID가 없으면 false를 반환한다.
전환 과정에서 ID와 등록 순서를 유지한다.
list를 사용하므로 다른 표적 삭제 시 남은 객체나 추적 상태를 재구성하지 않는다.
전환된 TentativeTrack과 삭제된 객체의 참조는 재사용하지 않으며 조회 결과를 Selector 멤버에 저장하지 않는다.
TrackIdGenerator는 저장 성공 후에만 다음 ID를 증가시키며, 삭제된 ID는 다시 발급하지 않는다.
현재 TrackEvent.id가 uint16_t이므로 발급 범위는 1~65535이며 소진 시 nextId()가 등록 전에 예외를 발생시킨다.
범위 확대에는 DTO와 콘솔 계약 변경이 필요하다.
그리드 ID나 수신 빔 requestId를 새 표적 ID로 사용하지 않는다.

## 객체별 역할

| 객체 | 역할과 주요 메서드 |
|---|---|
| TargetController | 탐지 DTO 변환·핸들러 호출, 보고 순서·한도 적용 |
| DetectionHandler | 검증된 결과 타입별 처리, 그리드별 탐색 처리 시각 관리, 표적 등록·확인·갱신·삭제, 빔 요청 생성 |
| TrackIdGenerator | 다음 표적 ID 조회, 저장 성공 후 발급 확정, 콘솔 ID 범위·소진 관리 |
| DetectionResultFactory | DTO를 검증된 결과 하위 타입으로 변환 |
| ObservationFactory | Plot 목록을 순수 측정 Observation 목록으로 변환 |
| TrackFactory | TentativeTrack 생성, from(tentative, observation)은 확인을 표적에 위임, 검증된 상태로 Track 생성 |
| TentativeSelector | 저장된 잠정·추적 표적 모두와 비교하여 첫 신규 관측 선택 |
| TrackSelector | selectConfirmation/selectObservation: 타입별 조회와 최소 NDS 관측 선택 |
| DetectionMatcher | 잠정/추적 타입별 탐색 중복 비교, 확인·추적 연관 점수 계산 |
| NdsAlgorithm | 공분산 검증·대칭 보정·Cholesky 분해와 잔차의 NDS 계산 |
| Observation | ID나 알고리즘이 없는 유효한 불변 관측 |
| TentativeTrack | 최초 관측 보존, initialize/confirm으로 초기화 위임·시각 계약 검증·확인 연결 |
| Track | ID·추적 상태·미탐 이력 관리, update/predict/toTrackEvent |
| TrackRepository | add/findTentative/findTrack/replace/remove/targets, 객체 저장·조회·교체·삭제와 ID 중복 검사 |
| ITrackingAlgorithm | Observation과 Track 입력으로 initialize/update/predict 계산 |
| ConstantAccelerationAlgorithm | 두 점 차분 초기화, CA 선형 칼만필터 관측 보정, 위치·공분산 예측 |
| ConstantVelocityAlgorithm | 공분산을 제공하지 않는 기존 등속도 전략. Controller에서는 사용하지 않음 |
| TargetConfig | 연관·측정·추적·표적 관리 설정의 조합 |

TargetController 생성자는 app에서 주입받은 TargetConfig에서 표적 관리 설정의 복사본을 보관하고 Repository, Matcher, 추적 알고리즘, 두 Selector와 DetectionHandler를 한 번 생성하여 의존성을 연결한다. 핸들러는 표적 관리 설정을 복사하고 Repository·TrackFactory·Selector 참조를 주입받는다. 참조 대상은 핸들러보다 오래 살아야 하며 Controller가 함께 소유한다.
두 Selector는 const TrackRepository&와 const DetectionMatcher&를 생성자로 주입받는다.
처리 도중 별도 Matcher나 Selector를 만들지 않으며, 저장소 변경은 다음 조회에 그대로 반영된다.
Selector는 vector 요소의 포인터나 참조를 멤버에 보관하지 않는다.
DetectionHandler는 이미 조회한 TentativeTrack 또는 Track을 Selector의 참조 오버로드에 전달하여 같은 ID를 다시 조회하지 않는다. ID 기반 조회 오버로드도 유지한다.
TargetController 복사본은 표적 관리 설정·Repository·Matcher·Selector·알고리즘·DetectionHandler 객체 구성을 함께 공유한다. 핸들러가 소유한 ID 생성기와 탐색 처리 이력도 함께 공유된다. 원본 Controller가 소멸한 뒤에도 복사본이 핸들러와 참조 대상들을 유지한다.
외부 app은 공개 헤더 include/target/config/target_config.hpp의 TargetConfig를 생성하여 TargetController(const TargetConfig&)에 명시적으로 주입한다.
TargetConfig는 BeamConfig처럼 검증된 하위 설정 네 개를 public 멤버로 조합한다. 개별 설정값은 각 하위 설정의 private 필드로 보관하고 const 접근자로 조회한다. 하위 설정은 생성자에서 값을 검증하며, 상위 설정은 검증을 반복하지 않는다.

| 하위 설정 | 소유 값 | 사용처 |
|---|---|---|
| association: AssociationConfig | maxAssociationSpeed, maxInitialDopplerDifference, positionNdsThreshold | DetectionMatcher |
| measurement: MeasurementConfig | standardDeviation | DetectionMatcher, ConstantAccelerationAlgorithm |
| tracking: TrackingConfig | initialAccelerationStandardDeviation, processAccelerationStandardDeviation | ConstantAccelerationAlgorithm |
| trackManagement: TrackManagementConfig | confirmationDelay, trackingInterval, maxConsecutiveTrackingMisses, maxReportedTrackCount | DetectionHandler의 요청·미탐 정책, TargetController의 보고 한도 |

측정 잡음은 MeasurementConfig 하나에서 두 계산 객체에 전달하므로 연관과 필터가 같은 값을 사용한다. Matcher와 알고리즘은 TargetConfig 전체에 의존하지 않는다. 각 사용처는 필요한 설정을 복사하여 호출자 객체의 수명이나 이후 하위 설정 교체에 의존하지 않는다.
모든 설정은 Builder도 제공한다. 필수 값이 빠지면 build()에서 std::logic_error를 발생시키며, 유효하지 않은 값은 하위 설정 생성자에서 std::invalid_argument로 거부한다. 명시적으로 전달한 0은 허용되는 설정에서 그대로 유지한다.

```cpp
using namespace math::literals;
using namespace std::chrono_literals;

const auto targetConfig = target::TargetConfig::Builder()
    .associationConfig(target::AssociationConfig(1000_mps, 30_mps, 11.345))
    .measurementConfig(target::MeasurementConfig(7_m))
    .trackingConfig(target::TrackingConfig(10_mps2, 10_mps2))
    .trackManagementConfig(target::TrackManagementConfig(100ms, 1000ms, 3, 100))
    .build();
target::TargetController targetController(targetConfig);
```

Config 구현과 하위 설정별 검증은 src/config/의 대응하는 cpp 파일에 둔다. Controller에서 기본 설정을 암묵적으로 생성하거나 별도의 정적 설정 함수를 사용하지 않는다.

app은 기존 `.env`의 필수 키와 단위를 유지하여 하위 설정을 생성한 뒤 TargetConfig로 조합한다. 생성자 기본값은 없으며, `.env.example`의 시작 설정은 다음과 같다. 하위 설정 생성 시 한 번 검증한다.

| 설정 | 시작 설정 |
|---|---|
| maxAssociationSpeed | 1000m/s |
| maxInitialDopplerDifference | 30m/s |
| measurementStandardDeviation | 7m |
| positionNdsThreshold | 11.345 (3차원 99%, 임시 기준) |
| confirmationDelay | 100ms |
| trackingInterval | 1000ms |
| maxConsecutiveTrackingMisses | 3 |
| initialAccelerationStandardDeviation | 각 축 10m/s² (1σ, 조정 가능한 초기값) |
| maxReportedTrackCount | 100 (콘솔 보고 개수, 1~100) |
| processAccelerationStandardDeviation | 각 축 10m/s² (1σ, 예측 구간별 가속도 교란) |

측정 표준편차와 초기 가속도 표준편차는 양수이며, 각 분산과 2배 측정 분산은 유한한 양수여야 한다. 초기 가속도 표준편차는 math::LinearAcceleration으로 전달한다. 임계값과 속력·도플러 한도는 음수가 될 수 없다.
공정 가속도 교란 표준편차는 math::LinearAcceleration으로 전달하며 0 이상이어야 한다. 제곱한 분산은 유한해야 하고 양수 표준편차의 제곱이 0으로 언더플로하는 설정도 거부한다. 0은 기존 Q=0 동작을 보존한다.
요청 간격과 연속 미탐 한도는 양수여야 하며, 잘못된 설정은 std::invalid_argument를 발생시킨다.
보고 개수는 TrackManagementConfig::MAXIMUM_REPORTED_TRACK_COUNT(100)를 상한으로 1~100에서 설정한다. app이 하위 설정 생성 시 더 작은 값을 주입할 수 있으며, Controller는 trackManagement의 maxReportedTrackCount를 참조한다.
공정 잡음은 `TARGET_PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2=10`으로 설정한다. 5와 비교하려면 이 값만 5로 바꾸고 앱을 재시작한다. 기존 `.env`에도 이 필수 키를 추가해야 한다.

## 연관과 추적 계산

모든 탐지 연관은 src/service/의 DetectionMatcher를 거친다.
확인과 추적 연관은 각각 TentativeTrack과 Track만 받는다.

- matchesPosition(observation, other): Tentative는 최초 위치에서 1000m/s × 두 탐지의 시간 차이만큼 이동할 수 있다고 보고, 그 범위를 벗어난 잔여 위치 차이의 NDS가 임시 99% 기준인 11.345 이하이면 탐색 중복으로 판단한다. Tracking은 수신 시각의 예측 위치와 비교하되, 최근 관측보다 과거인 탐색 결과는 최신 저장 위치와 시간 차이로 같은 이동 범위 보정을 적용한다.
- matchesConfirmation(observation, tentative): 두 점의 안테나 좌표계 Cartesian 거리 / 실제 시간 차이가 1000m/s 이하이고 측정 도플러 차이가 30m/s 이하인 탐지의 NDS 점수를 반환한다. TrackSelector가 가장 작은 점수 하나를 선택한다.
- matchesTracking(observation, tracking): 수신 시각의 예측 위치와 관측 위치의 Cartesian 차이로 NDS를 계산한다. positionNdsThreshold 이하인 점수만 반환하고 TrackSelector가 최소 점수 하나를 선택한다. 탐색 중복의 이동 범위 보정은 적용하지 않는다.

확인 연관의 속력은 레이더와 표적 사이의 거리 변화만 사용하지 않고, 방위각·고각을 포함한 두 위치의 3차원 차분으로 계산한다.
Tentative 탐색 중복의 허용거리는 1000m/s × |현재 탐지 시각 − 최초 탐지 시각| + 측정오차 허용거리이다.
다른 그리드에서 늦게 도착한 과거 탐색 결과도 Tracking과 중복 비교한다. 이때 기준은 최초 시각이나 최근 미탐 시각이 아닌 최근 관측 시각이며, 추적 상태를 과거로 갱신하거나 역방향 예측을 수행하지 않는다.
동일 시각·미래 시각의 Tracking 예측이 실패하면 기존대로 점수를 반환하지 않으며, 저장 위치 비교로 우회하지 않는다.
기본 설정의 98I 공분산과 11.345 기준에서는 측정오차 허용거리가 약 33.34m이며, 이동 가능한 거리 안의 중복 점수는 0이다.
최초 관측은 갱신하지 않는다. Tentative는 등방성 2σ²I를 사용한다. 과거 SEARCH와 Tracking의 비교는 저장 위치 공분산에 측정 공분산을 더하고, 이동 가능한 거리만큼 잔차를 줄인 뒤 점수를 구한다. 이는 과거 결과의 중복 억제를 위한 보수적 처리이며 역방향 필터링은 아니다.
Matcher는 추적 상태를 초기화하거나 변경하지 않는다.
TrackSelector가 연관에 성공한 관측 하나를 선택하면 TentativeTrack.confirm()이 initialize()를 통해 자신의 추적 알고리즘에 초기화를 한 번 위임한다.
연관 통과와 추적 초기화 성공은 별도 조건이며, 선택 후 초기화가 실패하면 확인 실패로 삭제한다.
확인 분기에 완전 위치 일치 조건을 추가하지 않으므로 이동한 정상 표적도 확인 조건으로 판단한다.

추적 인터페이스는 src/domain/tracking_algorithm.hpp에, 구현체는 src/algorithm/에 둔다.

    initialize(const Observation& first, const Observation& second)
    update(const Track& current, const Observation& observation)
    predict(const Track& current, milliseconds time)

반환 형식은 domain/tracking_state.hpp의 TargetTrackingState 및 TargetPrediction이다.
Track마다 자신의 상태를 보관하고 shared_ptr<const ITrackingAlgorithm>으로 무상태 계산 객체를 공유한다.
알고리즘은 입력 관측과 Track을 수정하지 않는다.
TrackFactory는 생성자에서 주입받은 추적 알고리즘을 생성하는 TentativeTrack에 전달한다. Controller는 알고리즘이 주입된 팩토리를 보관하고, DetectionHandler가 팩토리로 생성한 객체를 Repository에 전달한다. 저장소의 add()는 팩토리에 의존하지 않는다. 정적 from(tentative, observation)은 tentative.confirm(observation)에 위임한다. TentativeTrack이 초기화와 반환 시각 검증을 마친 뒤 비공개 from(id, initialTime, state, algorithm)에 검증된 값과 알고리즘 소유권을 전달하여 ID·최초 시각을 유지한 Track을 생성한다. 팩토리는 TentativeTrack의 private 필드에 접근하거나 알고리즘을 실행하지 않는다. Track 생성자는 비공개이며 TrackFactory만 직접 호출할 수 있다.
저장한 추적 상태는 shared_ptr<const TargetTrackingState>로 소유한다. 알고리즘이 반환한 상태는 공분산을 복사하지 않고 이동한다. 새 상태의 생성·검증·할당을 모두 마친 뒤 포인터를 교체하여, 실패 시 이전 상태를 보존한다. Track 이동 시에는 상태 포인터만 이동하고 Repository 삭제 과정에서도 행렬을 복사하지 않는다.

Observation, TentativeTrack, TargetTrackingState, TargetPrediction은 생성자에서 각자의 계약을 검사한다. Track은 확인에 성공한 TentativeTrack만 생성한다.
추적 상태와 예측 결과는 모든 필드가 const이며, 생성 후 외부에서 필드를 바꿀 수 없다.
Track의 상태 변경에서는 새 상태 객체의 생성 검증을 신뢰하고, 수신 시각과 반환 시각의 일치 등 변경 조건만 확인한다.
이미 저장된 상태를 조회·연관·예측에 사용할 때 유효성을 재검사하지 않는다.
유효하지 않은 생성 값이나 알고리즘의 시각 계약 위반은 예외를 발생시키며, 상태 반영 전 검사하므로 기존 상태·시각·미탐 횟수를 보존한다.
연관 실패·과거 결과·잘못된 상태에 대한 처리 조건과 실제 계산 실패의 false/nullopt는 유지한다.
필터 추정값이 수신 측정값과 같아야 한다는 조건은 두지 않는다.
잠정 표적은 최초 관측을 보존하고, Track은 최초 시각과 최근 추적 상태를 보존한다. 현재 값 getter는 최근 추적 상태를 반환한다.
예측은 저장 상태와 관측 시각을 앞당기지 않는다.

position은 math::CartesianPosition, velocity는 math::CartesianVelocity이다.
acceleration은 math::CartesianAcceleration이며, 각 축은 math::LinearAcceleration으로 표현한다.
errorCovariance는 선택적인 math::SquareMatrix이며, 있을 때는 9×9이다. 상태 순서는 [px, py, pz, vx, vy, vz, ax, ay, az]이고, 각 원소의 단위는 대응하는 두 상태 성분의 단위 곱이다.
Controller가 사용하는 CA 알고리즘은 초기 가속도 [0, 0, 0]과 두 점 차분으로 계산한 공분산을 반환한다. 기존 등속도 전략의 공분산 미설정은 불확실성이 0이라는 뜻이 아니다.
공분산 생성 검사는 크기·유한성·정확한 대칭·대각 원소의 비음수 여부까지 수행한다. 이는 양의 준정부호를 보장하는 검사가 아니며, 공분산을 생성하는 알고리즘이 그 성질을 보장해야 한다. 현재 math에는 양의 준정부호 검사 API가 없고, 양의 정부호만 허용하는 Cholesky로 이 조건을 대체하지 않는다.
안테나 고정 좌표계는 math와 동일하게 방위각 0도 방향 y, 방위각 +90도 방향 x, 위쪽 z를 사용한다.
Observation과 추적 상태·예측 결과의 구면 위치는 math::SphericalPosition으로 저장한다.
Track.sphericalPosition()은 저장된 구면 위치의 const 참조를 반환하며, 조회할 때 좌표 객체를 다시 생성하지 않는다.
구면·직교 위치 변환은 math::SphericalPosition::toCartesian()과 math::CartesianPosition::toSpherical()에 위임한다.
모든 도메인 구면 위치의 방위각은 math의 [0, 360)도 기준으로 정규화된다.
위치 뺄셈은 math::CartesianDisplacement를 반환한다. Matcher는 toVector()로 NDS 입력을 얻고, 초기화·갱신 알고리즘은 toVelocity()로 속도 벡터를 계산한다.
확인·추적은 적합한 모든 탐지의 NDS 점수를 비교하여 최소 점수 하나를 선택하며, 동점이면 입력 순서상 첫 탐지를 유지한다. 추적만 위치 NDS 게이트를 적용하고 확인의 속력·도플러 조건과 무게이트 순위 계산은 유지한다.
추적 관측 선택은 한 결과의 첫 시각 일치 관측에서 TrackingAssociation을 생성하고 예측 위치와 분해된 공분산을 보관한 NdsAlgorithm을 나머지 관측과 공유한다. 예측·분해 실패도 해당 선택 호출 안에서만 재사용하며, 다음 호출에서는 다시 계산한다. 빈 결과·시각 불일치 관측·과거 결과에는 예측을 실행하지 않는다. TrackingAssociation은 표적 참조를 보관하지 않으며 선택이 끝나면 폐기한다.
CA 계산의 위치(m)·속도(m/s)·가속도(m/s²) 숫자 벡터 구성은 추적 알고리즘 내부에서 처리한다. 새 알고리즘은 직교 위치를 기준으로 계산하고 구면 위치는 math API로 변환하여 함께 반환한다.
별도의 좌표 변환 보조 헤더는 사용하지 않는다. cmath는 유한값 검사에만 사용한다.
NDS 계산은 src/algorithm/nds_algorithm.hpp의 NdsAlgorithm에 위임한다. from(covariance)은 검증·대칭 보정·math::cholesky() 분해 결과를 소유하는 객체를 반환하며, calculate(residual)은 SquareMatrix::solveLowerTriangular()로 r^T S^-1 r을 계산한다. 일회 계산을 위한 calculate(residual, covariance)도 제공한다. 3차원 크기 불일치·비유한 입력·분해 또는 풀이 실패·점수 오버플로는 점수 없음으로 처리한다. NdsAlgorithm은 DTO·표적·설정에 의존하지 않으며 게이트를 적용하지 않는다. Matcher는 잔차·공분산 구성과 속력·도플러·NDS 게이트 정책을 담당한다.
유효한 두 위치라도 변위나 속력이 수치 범위를 넘으면 Matcher는 계산 실패로 점수 없음을 반환한다.
기존 대칭 허용오차를 통과한 공분산의 비대각 원소는 같은 평균값으로 맞춘 뒤 분해한다.
별도의 detection_validation 파일은 제거하고 객체별 검증을 도메인 내부로 옮겼다.

### CA 칼만필터

ConstantAccelerationAlgorithm이 ITrackingAlgorithm을 구현한다. 설정과 계산 방법만 공유하고, 각 표적의 상태와 9×9 공분산은 TargetTrackingState에 보관한다.
측정은 독립적인 직교 위치 3축이며, 측정 공분산 R = σ²I이다. 시작 σ = 7m에서 R = 49I이다. 초기 가속도 평균은 0, 초기 가속도 공분산 A = σa²I이며 시작 σa = 10m/s²에서 A = 100I이다. 초기 가속도 불확실성과 공정 잡음 표준편차는 서로 다른 설정이다.

초기화 시 두 관측 사이의 실제 시간 차이를 d초로 두고, 두 번째 관측 시각의 상태를 p = z2, v = (z2-z1)/d, a = 0으로 만든다. 독립 측정 R1 = R2 = R을 사용하는 초기 공분산은 다음 3×3 블록들로 구성한다.

```text
Ppp = R2                  Ppv = R2/d                  Ppa = 0
Pvp = R2/d                Pvv = (R1+R2)/d² + d²A/4     Pva = dA/2
Pap = 0                   Pav = dA/2                  Paa = A
```

100ms 간격과 기본 설정에서 축별 (p,v,a) 공분산은 [[49,490,0],[490,9800.25,5],[0,5,100]]이다. 실제 저장 순서는 축별 묶음이 아니라 [px,py,pz,vx,vy,vz,ax,ay,az]이다.

예측은 p' = p + v·dt + a·dt²/2, v' = v + a·dt, a' = a와 P' = FPFᵀ + Q(dt)를 사용한다. 관측 보정은 위치 선택 행렬 H에 대해 S = HP'Hᵀ + R로 계산한다. 칼만 이득은 역행렬을 만들지 않고 math의 Cholesky와 하삼각·상삼각 풀이로 구한다. 보정 공분산은 Joseph 식 (I-KH)P'(I-KH)ᵀ + KRKᵀ을 사용하고 반올림 차이를 대칭으로 맞춘다.

#### 공정 잡음 행렬

각 예측 구간의 가속도 교란 w를 평균 0, 축별 독립인 공분산 σq²I로 모델링한다. 선택한 모델은 `x_next = F x + G w`이며, `G = [dt²/2 I; dt I; I]`이다. 이는 구간별 가속도 교란 모델이며, 연속 백색 jerk의 spectral density를 사용하는 모델과는 구분한다. 공정 표준편차 σq는 m/s² 단위이고, 행렬 Q 전체에 같은 숫자를 넣지 않는다.
가속도 교란의 상태 전달 형태는 [MathWorks의 Constant Acceleration Model](https://www.mathworks.com/help/fusion/ug/motion-model-state-and-process-noise.html)을 따른다.

상태 순서 `[px,py,pz,vx,vy,vz,ax,ay,az]`에서 각 I가 3×3 단위행렬일 때:

```text
                     [ dt⁴/4 I   dt³/2 I   dt²/2 I ]
Q(dt) = G σq²I Gᵀ = σq² [ dt³/2 I   dt² I     dt I    ]
                     [ dt²/2 I   dt I      I       ]
```

σq=10m/s², dt=1초에서는 다음 9×9 행렬이다. 각 원소의 단위는 대응하는 두 상태 성분의 단위 곱이다.

```text
     [25  0  0   50   0   0   50   0   0]
     [ 0 25  0    0  50   0    0  50   0]
     [ 0  0 25    0   0  50    0   0  50]
     [50  0  0  100   0   0  100   0   0]
Q =  [ 0 50  0    0 100   0    0 100   0]
     [ 0  0 50    0   0 100    0   0 100]
     [50  0  0  100   0   0  100   0   0]
     [ 0 50  0    0 100   0    0 100   0]
     [ 0  0 50    0   0 100    0   0 100]
```

dt는 고정 요청 주기가 아니라 저장된 관측 상태 시각부터 계산 대상 시각까지의 실제 경과 초를 사용한다. 같은 시각의 예측은 새로운 구간이 아니므로 Q(0)=0으로 처리한다. σq=0도 영행렬을 반환한다. σq=5이면 같은 dt에서 Q는 σq=10일 때의 1/4이다. 이 이산 모델은 계산 대상까지를 하나의 구간으로 다루며, 구간을 나눌 때마다 독립적인 교란을 더하는 모델과는 다르다.

Q는 ConstantAccelerationAlgorithm 내부에서 math의 행렬곱으로 구성한다. Controller나 Matcher에 계산을 두지 않고 update와 predict가 공유하는 예측 함수에서 한 번 더한다. 공정 잡음의 평균은 0이므로 예측 상태 평균에 난수나 고정 가속도를 더하지 않는다.

TargetPrediction은 예측 시각의 구면 위치와 3×3 위치 공분산을 전달한다. Matcher의 Tracking 비교는 이 공분산과 R을 더한 S로 NDS를 계산한다. 동일 시각 이후 SEARCH와 Tracking 비교에도 같은 예측 공분산 경로를 사용한다. Tentative·확인 비교는 두 측정의 차이이므로 2R을 유지한다. 공분산이 없는 기존 등속도 전략과 테스트 전략만 호환용 2R을 사용하며 Controller의 CA 경로는 항상 계산한 예측 공분산을 제공한다.

예측은 저장 상태를 변경하지 않는다. 미탐 뒤에는 최근 관측 시각부터 요청 시각까지의 전체 경과 시간으로 F와 Q를 계산하므로, 미탐 간격이나 이전 예측의 Q를 중복 적용하지 않는다. 반복 조회도 상태를 변경하지 않는다. 다음 정상 관측에서 위치·속도·가속도와 공분산을 함께 보정한다. σq는 조정 가능한 시작값이며 실제 기동과 측정 잡음이 있는 로그로 추적 유지와 오연관을 함께 검증해야 한다.

## 빔 타입별 처리

### SEARCH

1. 그리드별 과거·중복 결과를 무시한다.
2. 수신 Observation들을 저장된 Tentative·Tracking 모두와 Matcher로 비교한다.
3. 첫 신규 관측 하나만 등록한다. 새 ID와 최초 측정·시각을 보존한다.
4. 해당 위치로 실제 송신 시각 +100ms의 확인 요청 하나를 반환한다.
5. 나머지 신규 물체는 다음 탐색에서 발견되면 다시 등록을 검토한다.

### CONFIRMATION

1. 요청 ID의 Tentative를 찾는다. 잘못된 ID·상태·과거·중복 결과는 무시한다.
2. TrackSelector가 Matcher.matchesConfirmation()으로 수신 Observation 하나를 선택한다.
3. 선택된 Target으로 초기화에 성공하면 같은 ID의 Tracking으로 전환한다.
4. 실제 송신 시각 +1000ms의 예측 위치로 추적 요청 하나를 반환한다.
5. 빈 결과·연관 실패·초기화 실패이면 Tentative를 삭제한다.
6. 다른 수신 Observation은 등록하거나 갱신에 사용하지 않는다.

### TRACKING

1. 요청 ID의 Tracking만 처리한다. 잘못된 ID·상태·과거·중복 결과는 무시한다.
2. TrackSelector가 Matcher.matchesTracking()의 게이트를 통과한 수신 Observation 중 최소 NDS 하나를 선택한다.
3. 선택된 Target으로 기존 표적을 갱신하고, 실제 송신 시각 +1000ms의 추적 요청 하나를 반환한다.
4. 빈 결과·연관 실패·갱신 실패는 연속 실패 횟수를 1 증가시킨다.
5. 1·2회 실패는 최근 측정 상태를 보존하고 다음 시각으로 위치를 다시 예측한다.
6. 3회 연속 실패하면 삭제하며, 정상 갱신 시 실패 횟수는 0으로 초기화한다.
7. 미탐 결과 처리 시각은 보관하지만 최근 관측 시각은 변경하지 않는다.
8. 다른 수신 Observation은 등록하거나 다른 표적 갱신에 사용하지 않는다.

확인 성공 직후와 추적 갱신·미탐 후에 다음 요청 위치를 예측할 수 없으면 즉시 Target을 삭제한다.
이는 연속 미탐 3회 삭제와 별개다.
요청 시각의 정수 오버플로는 해당 상태 변경 전에 검사한다.
실제 대기나 타이머 실행은 하지 않는다.

빈 plots는 탐지 없는 완료 결과이며, 좌표가 0인 가짜 탐지로 변환하지 않는다.
현재 구현은 탐지가 없는 경우에도 완료 결과 메시지가 항상 도착한다고 가정한다.
결과 메시지 미수신이나 Beam 큐의 요청 폐기에 대한 시간 제한 처리는 구현하지 않는다.

## 추적 정보 보고와 미구현 범위

nextTrackEvents()는 등록 순서대로 Tracking 상태인 표적을 maxReportedTrackCount개(기본 100개)까지 복사하여 반환하며 조회로 표적을 소비하지 않는다. Tentative는 보고 개수에 포함하지 않는다.
보고 한도를 초과하는 Tracking도 Repository에 유지하며 관측 갱신·예측·빔 요청을 계속 처리한다. 앞쪽 표적이 삭제되면 뒤쪽 Tracking이 다음 보고에 포함된다. 이는 콘솔 보고 목록만 자르는 정책이며 동시 추적 개수 제한이나 순환 보고는 아니다.
최근 측정·추정의 거리·방위각·고각·도플러와 실제 관측 시각을 보고한다.

- groundRange: 레이더 기준 수평 투영 거리.
- altitude: 레이더 기준 상대 높이 z.
- velocity: 속도 벡터 크기.
- heading / flightPathAngle: 레이더 기준 수평 진행각 / 속도 상승각. 정지 표적은 0도.
- latitude / longitude: 기준 위치가 없어 현재 0도로 보고한다.

Tracking 결과 연관은 예측 위치 공분산 + 측정 잡음 공분산으로 NDS를 계산하고 최소 점수 하나를 선택한다. positionNdsThreshold의 기본값 11.345는 3차원 99% 카이제곱 기준의 조정 가능한 설정이다. 초기 가속도 불확실성과 잡음 가정의 실제 적합성은 운용 데이터로 확인해야 한다.
CA 칼만필터의 공분산 초기화·예측·관측 보정과 NDS 연결, 구간별 가속도 교란 공정 잡음은 구현되어 있다. 플랫폼 자세 보정과 운용 로그에 기반한 공정 잡음 최적값 조정은 포함하지 않는다.
연관 조건은 Matcher에서, 추적 알고리즘은 ITrackingAlgorithm 구현체에서 교체한다.
Beam Scheduler는 확인·추적 요청의 requestId를 BeamCommand.beamId로 전달한다. Comm은 수신 beamID를 DetectionEvent.requestId로 전달한다.
실제 장비의 추적 조향 범위 검증은 요청을 생성하는 쪽의 책임이다. 현재는 기하 유효성만 검사하며, 조향 범위 밖의 요청 중단·표적 정리 정책은 미구현이다.
추적 알고리즘 완료 후에는 구면·직교 위치 중 기준 표현을 정하거나 두 표현의 일관성 계약을 확정한다.

## 검증

detection_handler_test.cpp는 Controller 없이 핸들러에 검증된 결과를 전달하여 등록·확인·갱신·미탐 삭제·ID 보존, 잘못된 단계·과거 결과 무시와 요청 시각 오버플로 전 상태 보존·최종 미탐 삭제를 확인한다.
빈 탐색 결과의 그리드별 이력 갱신, 오래된 결과 무시와 등록 실패 후 ID·처리 이력을 보존한 재시도도 확인한다.
track_id_generator_test.cpp는 저장 확정 전 ID 보존, 순차 발급과 65535 소진 후 재사용·순환 방지를 검증한다.
nds_algorithm_test.cpp는 상관 공분산의 독립 계산값, 게이트 밖 점수, 분해 결과의 소유·재사용, 차원·비유한 값·특이/부정부호 공분산 거부, 대칭 허용오차와 작은 점수·수치 오버플로를 검증한다.
target_config_test.cpp는 설정 조합과 하위 Builder의 필수 값 누락, 분산 검증, 명시적 0 허용, 필요한 하위 설정만 받는 계산 객체의 생성 계약과 호출자 설정 교체 후 Controller의 상태 보존을 확인한다.
target_lifecycle_test.cpp는 결과 타입·불변 스냅샷·메타데이터 검증·등록 및 전환·확인 선택·시각 경계와 잘못된 입력의 상태 보존을 확인한다.
저장소는 미리 생성된 두 표적 타입의 등록, ID 중복 거절, 기존 ID만 교체하는 계약과 교체 시 등록 순서 보존을 검증한다.
세 빔 타입 모두에서 빈 관측 결과의 메타데이터와 여러 관측의 순서·값을 보존하며, 원본 DTO 변경이 생성된 결과에 영향을 주지 않는지도 검증한다.
Controller 복사·대입으로 공유한 핸들러와 저장소가 원본 소멸 후에도 유효한지 확인한다. 구체 Controller의 설정 주입과 복사 계약은 정적 검사로 검증한다.
TrackFactory의 직접 Track 생성, 팩토리·잠정 표적이 소멸한 뒤 알고리즘 수명 유지와 생성 실패 후 재시도도 수명주기·도메인 검증 테스트에서 확인한다.
TentativeTrack의 초기화 위임은 시도별 호출 횟수, 과거·중복 시각의 호출 생략, 직접 초기화·팩토리 확인·표적 확인 경로와 실패 후 최초 관측 보존·재시도도 검증한다.
test/의 기존 immutable_geometry_test.cpp는 추적 상태 교체·표적 삭제·보고와 math 좌표축 기준을 확인한다.
domain_validation_test.cpp는 생성 예외, 상태 보존, 불변 필드 및 설정 주입을 검증한다.
tracking_association_test.cpp는 추적 NDS의 3차원 잔차·예측 시각·게이트·최소 점수 선택과 미탐·ID 보존을 검증한다.
같은 테스트는 배치별 예측 1회, 갱신 후 재계산, 예측 불필요 입력의 호출 생략, 실패 재시도 범위와 비양의 정부호 혁신 공분산 거부도 확인한다.
tracking_state_test.cpp는 가속도·공분산의 기본값·구조 검증·불변성·상태 교체·미탐 보존·표적 삭제 후 값 보존을 검증한다.
constant_acceleration_algorithm_test.cpp는 초기 P, CA 전이, 관측 보정의 독립 계산값, 축간 상관, 가속도 학습, 시간·수치 경계와 설정 검증을 확인한다.
kalman_controller_test.cpp는 실제 Controller의 필터 좌표 보고, 예측 공분산 게이트와 최소 점수 선택, 3축 정가속·잡음 관측 및 미탐 예측을 검증한다.
track_reporting_limit_test.cpp는 보고 개수 경계, Tentative 제외, 조회의 비소비성 및 보고에서 제외된 표적의 추적·삭제 후 보고 편입을 확인한다.
MSVC C++17 Debug / Visual Studio 18 2026 전체 빌드와 CTest 573개를 통과했다.
연관 계산 재사용 리팩터링에는 배치 처리 회귀 테스트 4개를 추가했고, TrackFactory 통합에는 직접 생성과 알고리즘 수명·생성 실패 후 재시도 테스트 2개를 추가했다.
Controller 복사·대입에는 핸들러와 저장소의 수명 보존 회귀 테스트 1개를 유지한다.
TargetConfig 분리에는 Builder와 하위 설정 주입·수명 보존 회귀 테스트 7개를 추가했다.
초기화 위임에는 호출 횟수·최초 관측 보존·실패 후 재시도 회귀 테스트 2개를 추가했다.
NdsAlgorithm 분리에는 수치 계산·실패 경계·분해 결과 재사용 독립 테스트 8개를 추가했다.
DetectionHandler 분리에는 수명주기·과거 결과·시각 오버플로 직접 테스트 3개를 추가했다.
CA 계산의 초기화·공분산·관측 보정·공정 잡음과 Comm/app 파이프라인의 기존 테스트를 함께 실행한다.
실제 모의기·장비·운용 콘솔을 연결한 통합 실행은 수행하지 않았다.

## 추가 검토 후보

- TargetController 복사본은 저장소와 알고리즘을 공유한다. 향후 독립 Controller가 필요한 사용처가 생기면 복사 금지 또는 명시적인 공유 API로 계약을 드러내는 방안을 검토한다.
