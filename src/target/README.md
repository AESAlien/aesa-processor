# Target 모듈

`TargetController`가 외부의 탐지 결과 수신과 추적 정보 조회를 처리한다.
`TargetRepository` 하나가 `std::vector<Target>`으로 Tentative와 Tracking 타겟을 보관한다.
신규 등록은 SEARCH에서만 수행하며, CONFIRMATION/TRACKING에서는 요청한 타겟 하나만 관리한다.

## 공개 입력과 출력

공개 Controller 헤더와 DTO는 기존 선언을 유지한다.

```cpp
std::optional<beam::BeamRequest> handleDetection(DetectionEvent detection);
std::vector<TrackEvent> nextTrackEvents() const;
```

`DetectionEvent` 하나는 완료된 빔 하나의 결과다.

- `beamType`: SEARCH / CONFIRMATION / TRACKING.
- `requestId`: SEARCH에서는 그리드 ID, CONFIRMATION/TRACKING에서는 요청한 Target ID.
- `transmitTime`: 공유 시작점 기준 실제 빔 송신 시각(milliseconds).
- `plots`: 해당 빔의 탐지 목록. 탐지가 없는 완료 결과도 빈 목록으로 전달한다.

Controller는 메타데이터와 전체 Plot의 위치·도플러 유효성을 검증한 뒤 상태를 변경한다.
반환값은 빔 요청 0개(`std::nullopt`) 또는 1개다. `BeamRequest.requestId`에는 Target ID를 넣는다.
메시지 파싱·패키징·직렬화는 Comm에서 담당하며 Target에는 포함하지 않는다.

## 객체별 역할

| 객체 | 역할과 주요 메서드 |
|---|---|
| TargetController | `handleDetection`: 빔 타입별 흐름 연결 및 요청 생성. `nextTrackEvents`: 현재 Tracking 정보의 복사본 반환 |
| TentativeSelector | `selectNewCandidate`: SEARCH의 Plot들을 모든 저장 타겟과 비교하여 첫 신규 Plot 인덱스 선택 |
| TrackSelector | `selectObservation`: 요청 ID·상태·시각을 확인하고 해당 타겟에 적합한 Plot 인덱스 하나 선택 |
| DetectionMatcher | `isValidPosition`: 기하·도플러 유효성 검사. `matchesPosition`: 현재의 위치 완전 일치 비교 |
| Target | ID, 최초 Plot/시각, 표적별 추적 상태와 실패 횟수 관리. 확인 조건·상태 전환을 판단하고 초기화·갱신·예측 계산은 주입된 알고리즘에 위임 |
| TargetRepository | `add`/`find`/`remove`/`targets`: 타겟 소유·ID 발급·조회·삭제. 그리드별 최신 탐색 결과 시각 보관 |
| ITrackingAlgorithm | `initialize`/`update`/`predict`: 추적 계산의 인터페이스. 상태를 입력받고 계산 결과를 반환 |
| ConstantVelocityAlgorithm | 두 점의 위치 차분으로 속도를 구하고 등속도 모델로 미래 위치를 예측하는 현재 구현체 |

Selector는 탐지를 고르고, Target은 선택된 탐지로 자신의 상태를 변경한다.
두 Selector는 생성자에서 `const TargetRepository&`를 주입받아 같은 저장소를 읽는다.
Controller가 내부에서 Selector를 생성하고 저장소를 연결하므로 app은 Repository를 생성하거나 주입하지 않는다.
`TentativeSelector::selectNewCandidate(plots)`는 탐지 목록만 받고,
`TrackSelector::selectObservation(detection)`는 주입된 저장소에서 요청 ID의 타겟을 조회한다.
Selector는 처리 중 지역 객체로 사용하며 저장소를 소유하거나 타겟 포인터를 보관하지 않는다.
Controller에 기존 처리 흐름을 통합하여 TargetProcessor, BeamDetectionBatch, TargetBeamRequest를 제거했다.
별도 TargetObservation 없이 DTO의 Plot과 실제 시각을 직접 전달한다.
Plot의 const 필드는 도메인 내부 값으로 복사하여 `vector<Target>`의 이동·삭제가 가능하게 유지한다.

## 추적 알고리즘 주입

인터페이스와 구현체는 `src/calculation/tracking/`에 둔다.
연관 계산은 `src/calculation/association/`에 두어 추적 계산과 역할을 구분한다.
`TargetTrackingState`와 `TargetPrediction`은 `src/domain/tracking_state.hpp`에 정의한다.

```text
TargetController
  └─ ConstantVelocityAlgorithm 생성
      └─ TargetRepository 생성자에 주입
          └─ add()에서 각 Target 생성자에 주입
```

`Target`과 `TargetRepository`는 `ITrackingAlgorithm`에 의존한다.
Controller만 현재 사용할 구체 구현체를 선택하므로 외부 app과 공개 Controller 헤더는 바뀌지 않는다.
각 Target이 자신의 추적 상태를 소유하고, `shared_ptr<const ITrackingAlgorithm>`으로 상태 없는 계산 객체를 공유한다.
알고리즘은 타겟별 상태를 멤버에 보관하지 않으며, 입력 상태도 수정하지 않는다.
따라서 한 타겟의 갱신이나 예측이 다른 타겟의 상태를 바꾸지 않는다.

- `initialize(first, firstTime, second, secondTime)`: 두 관측으로 초기 추적 상태를 계산한다.
- `update(state, plot, time)`: 기존 상태와 새 관측으로 다음 추적 상태를 계산한다.
- `predict(state, time)`: 지정 시각의 예측 위치를 반환한다. 관측 시각이나 저장 상태를 앞당기지 않는다.

계산 실패는 `std::nullopt`로 반환한다. Target은 초기화·갱신에 성공한 결과만 저장하고,
확인 조건인 초기 속력 1000m/s 이하와 측정 도플러 차이 30m/s 이하는 공통 정책으로 유지한다.
미탐 1·2회에는 저장된 최근 관측 상태에서 다음 시각을 다시 예측하고, 3회 연속 실패 시 삭제한다.
향후 알고리즘을 교체하려면 인터페이스 구현체를 추가하고 Controller의 생성 부분을 바꾼다.
공분산 등 새로운 상태가 필요한 필터는 상태 형식도 함께 확장해야 한다.

## 처리 흐름

### SEARCH

1. 그리드별 과거·중복 결과를 무시한다.
2. TentativeSelector가 저장된 Tentative와 Tracking 모두와 비교하여 첫 신규 Plot 하나를 선택한다.
3. 신규 탐지가 없으면 요청을 반환하지 않는다.
4. 신규 탐지가 있으면 순차 ID를 발급하여 Tentative로 저장한다. 최초 탐지와 실제 시각은 이후에도 보존한다.
5. 해당 위치로 송신 시각 +100ms의 확인 요청 하나를 반환한다.
6. 같은 탐색 결과의 나머지 신규 물체는 이후 탐색에서 다시 발견되면 등록을 검토한다.

### CONFIRMATION

1. `requestId`에 해당하는 Tentative를 찾는다. 미등록·삭제 ID, 다른 상태, 과거·중복 결과는 무시한다.
2. TrackSelector가 Plot 목록에서 해당 타겟을 확인할 수 있는 탐지 하나를 선택한다.
3. 현재 확인 조건은 양의 시간 차이, 두 점 위치 차분 속력 1000m/s 이하, 도플러 차이 30m/s 이하이다.
4. 성공하면 같은 ID로 Tracking으로 전환하고 송신 시각 +1000ms의 예측 위치로 추적 요청 하나를 반환한다.
5. 적합한 탐지가 없으면 Tentative를 삭제하고 요청을 반환하지 않는다. 빈 완료 결과도 동일하다.
6. 선택되지 않은 탐지는 신규 등록하거나 다른 타겟 갱신에 사용하지 않는다.

### TRACKING

1. `requestId`에 해당하는 Tracking만 찾는다. 다른 상태와 과거·중복 결과는 무시한다.
2. TrackSelector가 해당 관측 시각의 예측 위치에 일치하는 Plot 하나를 선택한다.
3. 선택된 Plot으로 해당 타겟만 갱신하고, 송신 시각 +1000ms의 다음 추적 요청 하나를 반환한다.
4. 적합한 Plot이 없거나 갱신에 실패하면 연속 실패 횟수를 1 증가시킨다. 빈 결과도 실패 1회다.
5. 1·2회 실패에서는 최근 측정 상태를 보존하고 다음 송신 시각으로 위치를 다시 예측하여 추적 요청을 만든다. 조준각은 마지막 요청에 고정하지 않는다.
6. 3회 연속 실패한 결과를 받으면 타겟을 삭제하고 다음 추적 요청을 반환하지 않는다.
7. 정상 갱신하면 연속 실패 횟수를 0으로 초기화한다. 횟수는 타겟마다 독립적으로 관리한다.
8. 실패 결과의 처리 시각도 보관하여 중복·역순 결과가 횟수를 증가시키지 않도록 한다. 미탐 시 최근 관측 시각은 변경하지 않는다.
9. 다른 Plot은 신규 등록하지 않는다. 잘못된 ID·상태의 결과도 실패 횟수에 반영하지 않는다.

여기서 실패 횟수는 수신한 유효한 완료 결과에 대한 연관·갱신 실패 횟수다.
`PlotNumber=0`은 Comm에서 빈 `plots`로 전달하며, 좌표가 0인 Plot을 만드는 것이 아니다.
결과 메시지 자체가 도착하지 않는 경우나 Beam 큐의 요청 폐기는 별도의 통지·시간 제한 계약이 필요하다.

다음 송신 시각의 예측 위치를 계산할 수 없으면 해당 Target을 즉시 삭제하고 추적 요청은 `std::nullopt`로 반환한다.
확인 성공 직후와 추적 갱신·미탐 후에 모두 적용하며, 연속 미탐이 3회에 도달하기 전이라도 예측에 실패하면 삭제한다.
요청 시각의 정수 오버플로는 해당 상태 변경 전에 검사한다. 실제 대기나 타이머 실행은 하지 않는다.

## 추적 정보 보고

`nextTrackEvents()`는 현재 Tracking 타겟만 반환하며 조회해도 저장 타겟을 소비하지 않는다.
최근 선택된 Plot의 거리·방위각·고각·도플러와 실제 관측 시각을 보고한다.
보고 위치와 운동은 입력 DetectionEvent와 동일한 고정 레이더 좌표계를 사용한다.

- `groundRange`: 레이더 기준 수평 투영 거리 `hypot(x, y)`.
- `altitude`: 레이더 기준 상대 높이 `z`. 절대고도가 아니다.
- `velocity`: 두 점 차분 속도 벡터의 크기.
- `heading`: 레이더 x축 기준 수평 진행각 `atan2(vy, vx)`.
- `flightPathAngle`: 레이더 수평면 기준 속도 상승각 `atan2(vz, hypot(vx, vy))`.

입력에 없는 `latitude`/`longitude`는 NaN(미계산)으로 표시한다.
수평 속력이 0이면 heading, 전체 속력이 0이면 flightPathAngle도 방향이 정의되지 않아 NaN이다.
현재 필수 DTO 필드를 위한 표시 규약이며 Comm 및 콘솔의 NaN 수용은 아직 검증하지 않았다.
지구 좌표 변환이나 플랫폼 운동 보정은 수행하지 않는다. 향후 좌표계 계약이 바뀌면 보고 계산도 함께 검토해야 한다.

ID는 Tentative에서 Tracking으로 전환해도 유지하고, 삭제 후 재사용하지 않는다.
현재 `TrackEvent.id`가 uint16_t이므로 발급 범위는 1~65535이다. 범위를 소진하면 등록 전에 예외를 발생시킨다.
범위 확대에는 DTO와 콘솔의 ID 계약 변경이 필요하며, ID를 잘라내거나 순환시키지 않는다.

## 현재 계산과 외부 연결 범위

- Tentative 중복 비교는 최근 측정의 거리·방위각·고각 완전 일치이다.
- Tracking 연관은 관측 시각의 예측 위치와 완전 일치하는 임시 조건이다.
- NDS, 공분산, 칼만필터/CA 모델 및 플랫폼 자세 변화에 따른 좌표 변환은 미구현이다.
- 확인 조건은 Target에 있으며, 공통 위치 비교는 DetectionMatcher에 있다. NDS 도입 시 두 기준을 함께 검토해야 한다.
- 위치 차분과 예측은 ConstantVelocityAlgorithm에서 동일한 안테나 좌표계의 등속도 모델로 계산한다. 최초 탐지와 최근 관측은 예측으로 덮어쓰지 않는다.
- 정지 타겟은 원래 극좌표를 보존해 완전 일치 비교에서 좌표 변환 오차를 피한다.
- 현재 Beam Scheduler는 확인·추적 BeamCommand의 beamId를 0으로 생성한다. Target의 requestId가 외부를 거쳐 그대로 돌아오도록 Beam/Comm 연결이 추가로 필요하다.
- Comm의 실제 보고 전송과 운용 콘솔 표시는 이번 Target 수정 범위에서 실행하지 않았다.

## 파일 구성

| 파일 | 역할 |
|---|---|
| src/controller/target_controller.cpp | 외부 진입, 빔 타입별 처리 연결, 빔 요청 및 보고 생성 |
| src/selector/tentative_selector.hpp, .cpp | 탐색 결과의 신규 Plot 최대 하나 선택 |
| src/selector/track_selector.hpp, .cpp | 확인·추적에서 요청 타겟에 사용할 Plot 선택 |
| src/calculation/association/detection_matcher.hpp, .cpp | 위치 검증과 임시 공통 위치 비교 |
| src/calculation/tracking/tracking_algorithm.hpp | 상태를 보관하지 않는 추적 계산 인터페이스 |
| src/calculation/tracking/constant_velocity_algorithm.hpp, .cpp | 두 점 초기화·관측 갱신·등속도 예측 계산 |
| src/domain/tracking_state.hpp | 표적별 추적 상태와 예측 결과 값 |
| src/domain/target.hpp, .cpp | 최초 탐지·추적 상태·생명주기 관리, 주입된 알고리즘 호출 |
| src/repository/target_repository.hpp, .cpp | 전체 타겟 소유, ID·탐색 결과 시각 관리, 알고리즘 전달 |

## 검증

`test/`는 추후 테스트를 작성할 때 사용하도록 현재 비워 둔다.
Target의 테스트·데모 실행 대상과 해당 GTest 설정은 CMake에서 제외했다.
제거 전 기존 Target 테스트 28개와 탐색 → 확인 → 추적 → 3회 실패 삭제 데모의 통과를 확인했다.
현재 Target 라이브러리는 프로젝트 빌드로 검증한다.
추적 알고리즘 주입 구조로 변경한 뒤 전체 프로젝트 빌드와 기존 프로젝트 테스트 152개가 통과했다.
저장소 밖의 별도 검증 폴더에서도 기존 Target 회귀 28개와 알고리즘 주입·상태 보존·시간 계약 검증 11개, 총 39개가 통과했다.
예측 실패 시 Target 삭제 정책은 별도 회귀 검증 5개를 추가해 총 44개가 통과했다. 확인·추적 갱신·첫째 및 둘째 미탐 후 삭제, 다른 표적 유지, 삭제 ID의 늦은 결과 무시와 ID 재사용 방지를 확인했다.
