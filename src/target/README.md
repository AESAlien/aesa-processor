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
validateDetection()은 빔 타입·요청 ID·송신 시각 등 메시지 메타데이터만 검사한다.
거리·각도·도플러의 NaN·무한대는 math 물리량 생성 단계에서 거부된다.
toTargets()는 각 Plot의 좌표를 math::SphericalPosition으로 묶어 Target을 생성한다.
math 좌표 생성자가 음수 거리·고각 범위를 검사하고, Target 생성자가 양수 거리·시각·유한한 신호 세기와 알고리즘 의존성을 검사한다.
전체 Plot의 Target 생성이 끝난 뒤 저장소를 처리하므로, 뒤쪽 Plot이 잘못되어도 앞쪽 Plot만 저장되는 일은 없다.
이후 처리에는 빔 메타데이터 지역변수와 vector<Target>만 전달한다.
Selector, Matcher, Repository, 도메인 및 추적 알고리즘은 DetectionEvent를 포함하거나 사용하지 않는다.
이미 생성된 Target은 유효한 객체로 신뢰하며, Matcher와 Selector에서 값의 유효성을 반복 검사하지 않는다.

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
| TargetConfig | 연관 기준·측정 잡음·빔 요청 간격·연속 미탐 한도 설정 |

Controller 생성자는 app에서 주입받은 Config를 보관하고 Repository, Matcher, 추적 알고리즘과 두 Selector를 한 번 생성하여 의존성을 연결한다.
두 Selector는 const TargetRepository&와 const DetectionMatcher&를 생성자로 주입받는다.
처리 도중 별도 Matcher나 Selector를 만들지 않으며, 저장소 변경은 다음 조회에 그대로 반영된다.
Selector는 vector 요소의 포인터나 참조를 멤버에 보관하지 않는다.
Controller 복사본은 Config·Repository·Matcher·Selector·알고리즘 객체 구성을 함께 공유한다.
외부 app은 공개 헤더 include/target/config/target_config.hpp의 TargetConfig를 생성하여 TargetController(const TargetConfig&)에 명시적으로 주입한다.
설정 필드 7개는 모두 const이며, 기본값 또는 전체 설정값을 받는 생성자로 초기화한 뒤 변경할 수 없다. 잘못된 설정은 Config 생성 시 거부한다.
Controller는 전달받은 Config의 읽기 전용 복사본을 소유하여 호출자 객체의 수명에 의존하지 않는다. 이 설정을 Matcher에도 전달하고 요청 시각·미탐 한도 처리에 사용한다.

```cpp
const target::TargetConfig targetConfig;
target::TargetController targetController(targetConfig);
```

Config 구현은 src/config/target_config.cpp에 두며, Controller에서 기본 설정을 암묵적으로 생성하거나 별도의 정적 설정 함수를 사용하지 않는다.

TargetConfig의 기본값은 다음과 같으며 설정 생성 시 한 번 검증한다.

| 설정 | 기본값 |
|---|---|
| maxAssociationSpeed | 1000m/s |
| maxInitialDopplerDifference | 30m/s |
| measurementStandardDeviation | 7m |
| positionNdsThreshold | 11.345 (3차원 99%, 임시 기준) |
| confirmationDelay | 100ms |
| trackingInterval | 1000ms |
| maxConsecutiveTrackingMisses | 3 |

잡음은 양수이며 2σ²가 유한한 양수여야 한다. 임계값과 속력·도플러 한도는 음수가 될 수 없다.
요청 간격과 연속 미탐 한도는 양수여야 하며, 잘못된 설정은 std::invalid_argument를 발생시킨다.

## 연관과 추적 계산

모든 탐지 연관은 src/calculation/association/의 DetectionMatcher를 거친다.
확인 분기에서도 Target.canConfirm()을 호출하지 않는다.

- matchesPosition(observation, other): Tentative는 최초 위치에서 1000m/s × 두 탐지의 시간 차이만큼 이동할 수 있다고 보고, 그 범위를 벗어난 잔여 위치 차이의 NDS가 임시 99% 기준인 11.345 이하이면 탐색 중복으로 판단한다. Tracking은 수신 시각의 예측 위치와 비교하되, 최근 관측보다 과거인 탐색 결과는 최신 저장 위치와 시간 차이로 같은 이동 범위 보정을 적용한다.
- matchesConfirmation(observation, tentative): 두 점의 안테나 좌표계 Cartesian 거리 / 실제 시간 차이가 1000m/s 이하이고 측정 도플러 차이가 30m/s 이하인 탐지의 NDS 점수를 반환한다. TrackSelector가 가장 작은 점수 하나를 선택한다.
- matchesTracking(observation, tracking): 수신 시각의 예측 위치와 완전 일치. NDS 도입 전 임시 추적 연관 조건.

확인 연관의 속력은 레이더와 표적 사이의 거리 변화만 사용하지 않고, 방위각·고각을 포함한 두 위치의 3차원 차분으로 계산한다.
Tentative 탐색 중복의 허용거리는 1000m/s × |현재 탐지 시각 − 최초 탐지 시각| + 측정오차 허용거리이다.
다른 그리드에서 늦게 도착한 과거 탐색 결과도 Tracking과 중복 비교한다. 이때 기준은 최초 시각이나 최근 미탐 시각이 아닌 최근 관측 시각이며, 추적 상태를 과거로 갱신하거나 역방향 예측을 수행하지 않는다.
동일 시각·미래 시각의 Tracking 예측이 실패하면 기존대로 점수를 반환하지 않으며, 저장 위치 비교로 우회하지 않는다.
기본 설정의 98I 공분산과 11.345 기준에서는 측정오차 허용거리가 약 33.34m이며, 이동 가능한 거리 안의 중복 점수는 0이다.
최초 관측은 갱신하지 않는다. 이 이동 보정은 등방성 2σ²I를 전제로 하므로 향후 축별 공분산 모델을 도입할 때 다시 검토한다.
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

Target, TargetTrackingState, TargetPrediction은 생성자에서 각자의 private validate()를 호출한다.
추적 상태와 예측 결과는 모든 필드가 const이며, 생성 후 외부에서 필드를 바꿀 수 없다.
Target의 상태 변경에서는 새 상태 객체의 생성 검증을 신뢰하고, 수신 시각과 반환 시각의 일치 등 변경 조건만 확인한다.
이미 저장된 상태를 조회·연관·예측에 사용할 때 유효성을 재검사하지 않는다.
유효하지 않은 생성 값이나 알고리즘의 시각 계약 위반은 예외를 발생시키며, 상태 반영 전 검사하므로 기존 상태·시각·미탐 횟수를 보존한다.
연관 실패·과거 결과·잘못된 상태에 대한 처리 조건과 실제 계산 실패의 false/nullopt는 유지한다.
필터 추정값이 수신 측정값과 같아야 한다는 조건은 두지 않는다.
최초 측정값은 별도 필드에 보존하고 현재 값 getter는 Tracking일 때 최근 추적 상태를 반환한다.
예측은 저장 상태와 관측 시각을 앞당기지 않는다.

position은 math::CartesianPosition, velocity는 math::CartesianVelocity이다.
안테나 고정 좌표계는 math와 동일하게 방위각 0도 방향 y, 방위각 +90도 방향 x, 위쪽 z를 사용한다.
Target의 최초 위치와 추적 상태·예측 결과의 구면 위치는 math::SphericalPosition으로 저장한다.
Target.sphericalPosition()은 저장된 구면 위치의 const 참조를 반환하며, 조회할 때 좌표 객체를 다시 생성하지 않는다.
구면·직교 위치 변환은 math::SphericalPosition::toCartesian()과 math::CartesianPosition::toSpherical()에 위임한다.
모든 도메인 구면 위치의 방위각은 math의 [0, 360)도 기준으로 정규화된다.
위치 뺄셈은 math::CartesianDisplacement를 반환한다. Matcher는 toVector()로 NDS 입력을 얻고, 초기화·갱신 알고리즘은 toVelocity()로 속도 벡터를 계산한다.
추적의 완전 위치 비교는 SphericalPosition::equals()에 허용오차 없음을 명시해 수행한다.
등속도 예측의 위치(m)·속도(m/s) 숫자 벡터 구성은 추적 알고리즘 내부에서 처리한다.
별도의 좌표 변환 보조 헤더는 사용하지 않는다. cmath는 유한값 검사에만 사용한다.
NDS는 math::cholesky()와 SquareMatrix::solveLowerTriangular()로 계산하며, 분해·연립방정식 풀이의 계산 실패는 점수 없음으로 처리한다.
유효한 두 위치라도 변위나 속력이 수치 범위를 넘으면 Matcher는 계산 실패로 점수 없음을 반환한다.
기존 대칭 허용오차를 통과한 공분산의 비대각 원소는 같은 평균값으로 맞춘 뒤 분해한다.
별도의 detection_validation 파일은 제거하고 객체별 검증을 도메인 내부로 옮겼다.

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
현재 구현은 탐지가 없는 경우에도 완료 결과 메시지가 항상 도착한다고 가정한다.
결과 메시지 미수신이나 Beam 큐의 요청 폐기에 대한 시간 제한 처리는 구현하지 않는다.

## 추적 정보 보고와 미구현 범위

nextTrackEvents()는 현재 Tracking만 복사하여 반환하며 조회로 표적을 소비하지 않는다.
최근 측정·추정의 거리·방위각·고각·도플러와 실제 관측 시각을 보고한다.

- groundRange: 레이더 기준 수평 투영 거리.
- altitude: 레이더 기준 상대 높이 z.
- velocity: 속도 벡터 크기.
- heading / flightPathAngle: 레이더 기준 수평 진행각 / 속도 상승각. 정지 표적은 0도.
- latitude / longitude: 기준 위치가 없어 현재 0도로 보고한다.

NDS는 설정된 축별 표준편차 σ로 구성한 2σ²I 공분산을 임시로 사용한다. 기본 7m 설정에서는 98I이다.
Tracking 결과 연관은 아직 완전 위치 일치와 첫 통과 탐지 선택을 사용한다. NDS 점수와 최소 점수 선택은 미구현이다.
추적 공분산·칼만필터/CA·플랫폼 자세 보정은 미구현이다. 칼만필터 연결 시 표적별 가속도·공분산 저장 및 예측 공분산 전달 구조도 함께 확장해야 한다.
연관 조건은 Matcher에서, 추적 알고리즘은 ITrackingAlgorithm 구현체에서 교체한다.
Beam Scheduler는 확인·추적 요청의 requestId를 BeamCommand.beamId로 전달한다. Comm은 수신 beamID를 DetectionEvent.requestId로 전달한다.
실제 장비의 추적 조향 범위 검증은 요청을 생성하는 쪽의 책임이다. 현재는 기하 유효성만 검사하며, 조향 범위 밖의 요청 중단·표적 정리 정책은 미구현이다.
추적 알고리즘 완료 후에는 구면·직교 위치 중 기준 표현을 정하거나 두 표현의 일관성 계약을 확정한다.

## 검증

test/의 기존 immutable_geometry_test.cpp는 추적 상태 교체·표적 삭제·보고와 math 좌표축 기준을 확인한다.
domain_validation_test.cpp는 생성 예외, 상태 보존, 불변 필드 및 설정 주입을 검증한다.
전체 프로젝트 빌드·기존 프로젝트 테스트와 저장소 밖의 Target 회귀 검증으로 변경을 확인한다.
외부 회귀 검증은 MSVC C++17 /W4 /WX로 실제 Target 소스를 컴파일한다.
이번 변경은 전체 프로젝트를 새로 빌드하여 프로젝트 테스트 331개와 외부 Target 회귀 검사 220개를 통과했다.
실제 모의기·Comm·운용 콘솔의 통합 실행은 이번 변경 범위에 포함하지 않는다.
