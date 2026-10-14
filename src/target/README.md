# Target 모듈

TargetController가 외부 탐지 결과 수신과 추적 정보 조회를 처리한다.
TargetRepository 하나가 vector<Target>으로 Tentative와 Tracking 표적을 보관한다.
신규 등록은 SEARCH에서만 수행하며, CONFIRMATION/TRACKING에서는 요청한 표적 하나만 관리한다.

## 외부 DTO와 내부 도메인 경계

공개 메서드와 DTO 선언은 유지한다.

    std::optional<beam::BeamRequest> handleDetection(DetectionEvent detection);
    std::vector<TrackEvent> nextTrackEvents() const;

DetectionEvent 하나는 완료된 빔 하나의 결과다.

- beamType: SEARCH / CONFIRMATION / TRACKING.
- requestId: SEARCH에서는 그리드 ID, CONFIRMATION/TRACKING에서는 요청한 Target ID.
- transmitTime: 공유 시작점 기준 실제 빔 송신 시각(milliseconds).
- plots: 해당 빔의 모든 탐지. 탐지가 없는 완료 결과는 빈 목록으로 전달한다.

Controller의 입력 경계에서만 DetectionEvent와 Plot을 사용한다.
validateDetection()이 메타데이터와 전체 Plot의 위치·도플러를 먼저 검사한다.
toTargets()가 각 Plot의 측정값과 실제 시각을 Target으로 복사한다.
이후 처리에는 빔 메타데이터 지역변수와 vector<Target>만 전달한다.
Selector, Matcher, Repository, 도메인 및 추적 알고리즘은 DetectionEvent를 포함하거나 사용하지 않는다.
DTO를 검증하는 validateDetection()은 값 검증 함수를 사용하며, Matcher에는 Target만 전달한다.

반환값은 빔 요청 0개(nullopt) 또는 1개다. BeamRequest.requestId에는 등록된 Target ID를 넣는다.
메시지 파싱·패키징·직렬화는 Comm의 역할이다.
TrackEvent 변환도 Controller의 출력 경계에서만 수행한다.

## Target의 수신과 등록

수신 Plot을 변환한 Target은 ID가 0인 UNREGISTERED 상태다.
이는 비교에 사용할 수신 객체이며, 아직 Repository에 저장된 후보가 아니다.
신규 후보로 선택된 하나만 Repository.add(Target)에 전달한다.
Repository는 전달받은 복사본에 순차 ID를 부여하고 Tentative로 저장한 뒤 ID를 반환한다.
선택되지 않은 수신 Target이나 이미 등록된 표적에 연관된 수신 Target에는 새 ID를 발급하지 않는다.

    입력 검증
      → Plot마다 UNREGISTERED Target 생성
      → 저장 Target과 Matcher로 연관
      → 신규 하나만 add() / 기존 Target은 confirm() 또는 update()
      → BeamRequest 생성

등록 과정의 registerAs()는 private이며 Repository만 호출할 수 있다.
이미 등록된 Target의 재등록은 거부한다.
저장 성공 후에만 다음 ID를 증가시키고, 삭제된 ID는 재사용하지 않는다.
현재 TrackEvent.id가 uint16_t이므로 ID 범위는 1~65535이며 소진 시 등록 전에 예외를 발생시킨다.
범위 확대에는 DTO와 콘솔 계약 변경이 필요하다.
그리드 ID나 수신 빔 requestId를 새 Target ID로 사용하지 않는다.

## 객체별 역할

| 객체 | 역할과 주요 메서드 |
|---|---|
| TargetController | 입력 검증·Target 변환, 빔 타입별 처리 연결, BeamRequest·TrackEvent 생성 |
| TentativeSelector | selectNewCandidate(targets): 저장된 Tentative·Tracking 모두와 비교하여 첫 신규 Target 인덱스 선택 |
| TrackSelector | selectObservation(requestId, resultTime, targets): 요청 표적을 조회하고 Matcher로 수신 Target 하나 선택 |
| DetectionMatcher | matchesPosition: 탐색 중복 비교. matchesConfirmation: 확인 연관. matchesTracking: 추적 연관 |
| Target | 최초 측정·ID·추적 상태·실패 횟수 관리, confirm/update/predict에서 주입된 알고리즘 호출 |
| TargetRepository | add/find/remove/targets: 등록·조회·삭제·소유. 다음 ID 및 그리드별 처리 시각 관리 |
| ITrackingAlgorithm | Target 입력으로 initialize/update/predict 계산, 도메인 상태·예측 결과 반환 |
| ConstantVelocityAlgorithm | 두 점 차분 초기화·관측 갱신 및 등속도 예측 |

Controller 생성자는 Repository, Matcher, 추적 알고리즘과 두 Selector를 한 번 생성하고 의존성을 연결한다.
두 Selector는 const TargetRepository&와 const DetectionMatcher&를 생성자로 주입받는다.
처리 도중 별도 Matcher나 Selector를 만들지 않으며, 저장소 변경은 다음 조회에 그대로 반영된다.
Selector는 vector 요소의 포인터나 참조를 멤버에 보관하지 않는다.
Controller 복사본은 Repository·Matcher·Selector·알고리즘 객체 구성을 함께 공유한다.
외부 app은 TargetController()만 생성하면 된다.

## 연관과 추적 계산

모든 탐지 연관은 src/calculation/association/의 DetectionMatcher를 거친다.
확인 분기에서도 Target.canConfirm()을 호출하지 않는다.

- matchesPosition(observation, other): 거리·방위각·고각 완전 일치. NDS 도입 전 임시 탐색 중복 조건.
- matchesConfirmation(observation, tentative): 두 점의 안테나 좌표계 Cartesian 거리 / 실제 시간 차이가 1000m/s 이하이고 측정 도플러 차이가 30m/s 이하인지 검사.
- matchesTracking(observation, tracking): 수신 시각의 예측 위치와 완전 일치. NDS 도입 전 임시 추적 연관 조건.

확인 연관의 속력은 레이더와 표적 사이의 거리 변화만 사용하지 않고, 방위각·고각을 포함한 두 위치의 3차원 차분으로 계산한다.
Matcher는 추적 상태를 초기화하거나 변경하지 않는다.
TrackSelector가 연관에 성공한 Target 하나를 선택하면, 기존 Target.confirm()이 추적 알고리즘을 한 번 호출하여 초기화한다.
연관 통과와 추적 초기화 성공은 별도 조건이며, 선택 후 초기화가 실패하면 확인 실패로 삭제한다.
확인 분기에 완전 위치 일치 조건을 추가하지 않으므로 이동한 정상 표적도 확인 조건으로 판단한다.

추적 인터페이스와 구현체는 src/calculation/tracking/에 둔다.

    initialize(const Target& first, const Target& second)
    update(const Target& current, const Target& observation)
    predict(const Target& current, milliseconds time)

반환 형식은 domain/tracking_state.hpp의 TargetTrackingState 및 TargetPrediction이다.
Target마다 자신의 상태를 보관하고 shared_ptr<const ITrackingAlgorithm>으로 무상태 계산 객체를 공유한다.
알고리즘은 입력 Target을 수정하지 않는다.
Repository는 추적 알고리즘에 의존하지 않고 이미 생성된 Target을 보관한다.

Target은 알고리즘의 반환 시각과 좌표·각도·속도의 유효성을 확인한 뒤에만 결과를 저장하거나 반환한다.
실패한 초기화·갱신은 기존 상태를 덮어쓰지 않는다.
필터 추정값이 수신 측정값과 같아야 한다는 조건은 두지 않는다.
최초 측정값은 별도 필드에 보존하고 현재 값 getter는 Tracking일 때 최근 추적 상태를 반환한다.
예측은 저장 상태와 관측 시각을 앞당기지 않는다.

position은 math::CartesianPoint, velocity는 array<math::Velocity, 3>이다.
안테나 고정 좌표계는 방위각 0도 방향 x, 방위각 +90도 방향 y, 위쪽 z를 사용한다.
cartesian_math.hpp가 Target 극좌표의 Cartesian 변환과 단위 성분↔math::Vector 변환을 공유한다.
차분·외삽·크기는 math::Vector로 계산하고, 단위 리터럴과 equals를 사용한다.
math 모듈은 수정하지 않는다. cmath는 isfinite와 math API에 없는 sin/cos/atan2에만 사용한다.
validation/detection_validation.hpp/.cpp는 DTO 의존성 없이 기하·Target·추적 상태·예측을 검증한다.

## 빔 타입별 처리

### SEARCH

1. 그리드별 과거·중복 결과를 무시한다.
2. 수신 Target들을 저장된 Tentative·Tracking 모두와 Matcher로 비교한다.
3. 첫 신규 Target 하나만 등록한다. 새 ID와 최초 측정·시각을 보존한다.
4. 해당 위치로 실제 송신 시각 +100ms의 확인 요청 하나를 반환한다.
5. 나머지 신규 물체는 다음 탐색에서 발견되면 다시 등록을 검토한다.

### CONFIRMATION

1. 요청 ID의 Tentative를 찾는다. 잘못된 ID·상태·과거·중복 결과는 무시한다.
2. TrackSelector가 Matcher.matchesConfirmation()으로 수신 Target 하나를 선택한다.
3. 선택된 Target으로 초기화에 성공하면 같은 ID의 Tracking으로 전환한다.
4. 실제 송신 시각 +1000ms의 예측 위치로 추적 요청 하나를 반환한다.
5. 빈 결과·연관 실패·초기화 실패이면 Tentative를 삭제한다.
6. 다른 수신 Target은 등록하거나 갱신에 사용하지 않는다.

### TRACKING

1. 요청 ID의 Tracking만 처리한다. 잘못된 ID·상태·과거·중복 결과는 무시한다.
2. TrackSelector가 Matcher.matchesTracking()으로 수신 Target 하나를 선택한다.
3. 선택된 Target으로 기존 표적을 갱신하고, 실제 송신 시각 +1000ms의 추적 요청 하나를 반환한다.
4. 빈 결과·연관 실패·갱신 실패는 연속 실패 횟수를 1 증가시킨다.
5. 1·2회 실패는 최근 측정 상태를 보존하고 다음 시각으로 위치를 다시 예측한다.
6. 3회 연속 실패하면 삭제하며, 정상 갱신 시 실패 횟수는 0으로 초기화한다.
7. 미탐 결과 처리 시각은 보관하지만 최근 관측 시각은 변경하지 않는다.
8. 다른 수신 Target은 등록하거나 다른 표적 갱신에 사용하지 않는다.

확인 성공 직후와 추적 갱신·미탐 후에 다음 요청 위치를 예측할 수 없으면 즉시 Target을 삭제한다.
이는 연속 미탐 3회 삭제와 별개다.
요청 시각의 정수 오버플로는 해당 상태 변경 전에 검사한다.
실제 대기나 타이머 실행은 하지 않는다.

빈 plots는 탐지 없는 완료 결과이며, 좌표가 0인 가짜 탐지로 변환하지 않는다.
결과 메시지 자체의 미수신이나 Beam 큐의 요청 폐기는 별도의 통지·시간 제한 계약이 필요하다.

## 추적 정보 보고와 미구현 범위

nextTrackEvents()는 현재 Tracking만 복사하여 반환하며 조회로 표적을 소비하지 않는다.
최근 측정·추정의 거리·방위각·고각·도플러와 실제 관측 시각을 보고한다.

- groundRange: 레이더 기준 수평 투영 거리.
- altitude: 레이더 기준 상대 높이 z.
- velocity: 속도 벡터 크기.
- heading / flightPathAngle: 레이더 기준 수평 진행각 / 속도 상승각.
- latitude / longitude: 기준 위치가 없어 NaN. 속력 0의 운동 각도도 정의되지 않아 NaN.

Comm 및 콘솔의 NaN 수용과 실제 전송은 아직 통합 검증하지 않았다.
NDS·공분산·칼만필터/CA·플랫폼 자세 보정은 미구현이다.
연관 조건은 Matcher에서, 추적 알고리즘은 ITrackingAlgorithm 구현체에서 교체한다.
현재 Beam Scheduler의 확인·추적 beamId 연결은 담당 모듈의 별도 수정이 필요하다.

## 검증

test/는 요청대로 비워 둔다. Target 테스트·데모 및 GTest 대상은 모듈 CMake에 추가하지 않는다.
전체 프로젝트 빌드·기존 프로젝트 테스트와 저장소 밖의 Target 회귀 검증으로 변경을 확인한다.
외부 회귀 검증은 MSVC C++17 /W4 /WX로 실제 Target 소스를 컴파일한다.
실제 모의기·Comm·운용 콘솔의 통합 실행은 이번 변경 범위에 포함하지 않는다.
