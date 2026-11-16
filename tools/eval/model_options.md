# 계산 가설과 옵션

아래 식은 구현한 후보 모델의 정의다. 실제 플롯으로 부분 비교했지만 모의기의 내부 운동식을 확정한 것은 아니다.
전체 옵션과 기본값은 [model.example.json](model.example.json)에 있으며, JSON에서 생략한 값은 기본값을 쓴다.

## 초기 조건과 측정 좌표

| 옵션 | 의미와 선택값 |
| --- | --- |
| `range_scale_m`, `range_kind` | 필드 1에 곱할 m 단위 배율. `slant`: 경사거리, `ground`: 평면 수평거리 또는 구면 지표 호 길이 |
| `altitude_scale_m`, `altitude_reference` | 필드 3의 배율. `msl`: 해수면 기준, `radar`: 레이더 고도에 더함 |
| `radar_altitude_scale_m` | SCENARIO_INFORMATION의 고도 배율 |
| `speed_scale_mps`, `speed_kind` | 필드 4의 배율. `total`: 3차원 속력 유지, `horizontal`: 수평 속력 유지 |
| `azimuth_zero_deg`, `azimuth_sign`, `azimuth_frame` | 초기 방위 = zero + sign × 필드 2. `enu`: 북쪽 기준, `antenna`: 안테나 정면 기준 |
| `heading_zero_deg`, `heading_sign`, `heading_frame` | 진행 방향 = zero + sign × 필드 5. `enu` 또는 `antenna`의 수평 투영 |
| `vertical_kind`, `vertical_scale`, `vertical_sign` | 필드 6: `angle`이면 도 단위 상승각, `rate`이면 m/s 상승률로 배율·부호 적용 |
| `vertical_angle_mode` | `flight_path`: 상승각에서 정한 수직 속도 유지. `fixed_plane`: 초기 heading 방향의 기울어진 평면에 수평 궤적을 투영 |
| `output_frame` | `enu`: 지리 좌표 각도, `antenna`: 안테나 좌표 각도. 직교 출력은 항상 ENU |
| `output_azimuth_zero_deg`, `output_azimuth_sign` | 출력 방위 = sign × (기하 방위 - zero) |
| `output_elevation_sign`, `doppler_sign` | 각 ±1. 기본 고각은 위가 양수, 도플러는 멀어질 때 양수 |
| `attitude_enabled`, `attitude_scale_deg` | 자세 적용 여부, 시나리오 자세값의 도 단위 배율 |
| `attitude_order`, `attitude_signs`, `attitude_offset_deg` | 마지막 세 시나리오 값 중 roll/pitch/yaw에 대응할 인덱스·부호·오프셋 |
| `earth_model`, `earth_radius_m` | `flat`: 평면 ENU. `sphere`: 반지름 지정 구면 |

방위·방향의 기본 양수는 북→동(시계 방향)이다. 동쪽 기준은 zero=90,
반시계 방향은 sign=-1로 지정한다. 상승각 γ의 경우 `total`에서는 vz=v sinγ,
`horizontal`에서는 vz=vh tanγ를 쓴다. `total`의 수평 속력은 sqrt(v²-vz²)다.

자세 회전은 처리부 `src/pose/src/domain/ant_enu_transform.cpp`의 규약을 사용한다.
안테나/몸체 x=오른쪽, y=정면, z=위이고, ENU로의 회전은 yaw(z)·pitch(x)·roll(y) 순이다.
안테나 초기 방위와 지리 고도를 동시에 지정하면 해당 방위 평면과 고도 원의 교점을 구한다.
교점이 없으면 오류로 반환한다. 안테나 진행 방향은 수평 투영하고, 필드 6은 지리 수직 성분으로 해석한다.
별도 안테나 장착 회전행렬 및 시간에 따라 변하는 레이더 포즈는 지원하지 않는다.

구면 모델은 레이더·표적의 구면 좌표에서 ECEF 차를 구해 레이더 ENU로 투영한다.
지표거리는 지구 반지름 × 중심각, 경사거리는 두 점의 직선거리다. 고도는 구면 표면 기준이다.
지리적 heading을 유지하는 항정선 운동 후보로 적분하며, 대권 경로나 WGS84 타원체 모델은 아니다.
극점을 통과하는 적분은 오류다. 곡률 여부뿐 아니라 어떤 구면 운동인지도 추후 검증해야 한다.

## 시간

`scenario_time = processor_time_s - start_offset_s`다. `field10_role="start_time"`이면
필드 10 × `target_time_scale_s`를 한 번 더 빼서 표적의 경과 시간을 만든다.
기본 `field10_role="ignore"`는 필드 10을 운동에 반영하지 않는다.
필드 7·8 역시 의미를 부여하지 않고 `Target.values`에 보존한다.

`field11_role="ignore"`는 무기한 운동, `duration`은 표적 시작 기준 지속시간,
`end_time`은 scenario_time 기준 종료 시각이다. 단위는 `target_time_scale_s`로 바꾼다.
표적의 활성 구간은 시작과 종료 경계를 포함한다. 경계 밖 상태의 위치는 고정되고 속도는 0이다.

기동 시작값에는 `maneuver_time_scale_s`를 곱한다.
`maneuver_clock="target"`이면 표적 시작 기준, `scenario`이면 scenario_time 기준이다.
첫 플롯 시각이 곧 운동 시작이라는 가정은 하지 않는다. 오프셋은 실제 로그로 결정해야 한다.

`scenario_repeat_period_s`가 null이면 반복하지 않는다. 양의 주기를 지정하면 시작 오프셋 이후
scenario_time을 주기로 나눈 나머지로 바꾼 뒤 필드 10 지연과 기동 시각을 적용한다.
각 주기에서 처음부터 운동을 계산하며 이전 주기의 종료 상태를 이어 붙이지 않는다.
`field11_role="end_time"`과 함께 사용하면 종료~다음 주기 표적 시작 사이에는 비활성 상태다.
이 반복 주기는 명시적 후보 설정이며 TARGET 필드 11에서 자동으로 추정하지 않는다.

고정 기울기 평면은 초기 heading 단위 벡터 d를 기준으로 수평 변위의 평행 성분 s를 구한 뒤,
수평 변위에 `(cosγ-1)sd`, 상방 변위에 `sinγ s`를 더한다. 속도에도 같은 선형 변환을 적용한다.
따라서 선회 후 지리 heading이 바뀌면 수직 속도도 바뀐다. 평면 지구에서만 지원하며,
기울기가 0이 아닌 표적의 고도 기동과 결합하는 경우는 미지원 오류로 처리한다.
이는 첫 실험에서 표적 1의 고도에 더 잘 맞은 후보이며 모의기의 내부 회전식을 확인한 것은 아니다.

## 기동 후보

`maneuver_actions`는 종류 1~4를 순서대로 `climb`, `descend`, `left`, `right`, `ignore`에 매핑한다.
좌우·상하를 반대로 바꾸거나 모든 기동을 무시하는 후보를 만들 수 있다.
delta 목표는 크기만 사용하고 방향은 이 매핑으로 정한다.

| 옵션 | 모델 |
| --- | --- |
| `turn_model="lateral_acceleration"` | 각속도 크기 ω = a/vh, a = 필드 3 × `acceleration_scale` × `gravity_mps2` |
| `turn_model="load_factor"` | n = 필드 3 × `acceleration_scale`, a = g sqrt(n²-1), ω = a/vh. n>1 필요 |
| `turn_model="rate"` | `turn_rate_deg_s` 일정. 필드 3을 사용하지 않음 |
| `turn_model="rotating_acceleration"` | 각속도 ω=a/vh로 회전하는 수평 가속도 벡터를 속도에 더하는 후보. 기준 방향이 실제 진입 heading과 다르면 속력이 변함. 평면 ENU만 지원 |
| `turn_model="speed_coupled_rotation"` | 순간 수평 속력으로 가속도 방향의 회전율을 갱신하는 후보. 속도 heading과 가속도 기준각을 따로 적분. 평면 ENU만 지원 |
| `turn_acceleration_reference_deg` | 두 회전 가속도 후보의 기준 heading. null이면 기동 진입 heading, 숫자이면 고정 ENU 각도 |
| `turn_duration_step_s` | null이면 연속 공칭 시간. 양수이면 선회 공칭 시간을 해당 간격으로 올림. 목표 heading으로 다시 보정하지 않음 |
| `turn_duration_speed` | `entry`는 선회 진입 수평 속력, `initial`은 TARGET 초기 수평 속력으로 공칭 시간을 계산. 순간 회전율의 분모와 구분 |
| `turn_reference_mode` | `speed_coupled_rotation`에서 `reset`은 매 선회에 기준각을 초기화, `persistent`는 앞선 선회의 최종 기준각을 다음 선회로 유지 |
| `turn_reference_quantum_deg` | null이면 기준각 그대로. 양수이면 초기화할 기준각을 해당 도 간격의 최근접 값으로 반올림(정확한 중간은 증가 방향). 실패한 90도 양자화 가설도 재현할 수 있게 둠 |
| `turn_goal="delta"` | 필드 4 × `turn_goal_scale_deg`만큼 회전 |
| `turn_goal="absolute"` | 필드 4 × 배율을 ENU heading으로 보고, 지정 좌우 방향으로 도달할 때까지 회전 |
| `vertical_model="triangular"` | 일정 수직 가속 후 감속. 종료 시 기동 시작의 수직 속도로 복귀 |
| `vertical_model="discrete_triangular"` | 삼각 프로파일의 가속도를 일정 간격으로 속도에 적용하고 그 사이 등속 운동. 목표 고도·종료 수직 속도로 강제 보정하지 않음 |
| `vertical_update_step_s` | 이산 수직 가속도의 물리 갱신 간격. 기본 후보 0.1초. 수치 적분 간격 `integration_step_s`와 다른 옵션 |
| `vertical_model="acceleration"` | 일정 수직 가속으로 목표 고도까지 이동. 도달한 수직 속도로 계속 진행 가능 |
| `vertical_model="rate"` | 즉시 ±`vertical_rate_mps`로 바꾸어 목표 고도까지 이동. 필드 3을 사용하지 않음 |
| `vertical_goal="delta"` | 필드 4 × `vertical_goal_scale_m`만큼 고도 변경 |
| `vertical_goal="absolute"` | 필드 4 × 배율을 절대 고도로 해석. `altitude_reference` 적용 |
| `post_maneuver="tangent"` | 선회 종료 방향·고도 기동 종료 수직 속도로 계속 운동 |
| `post_maneuver="initial"` | 시나리오 초기 heading·수직 속도로 복귀 |
| `post_maneuver="stop"` | 표적 정지 |
| `overlap_policy="error"` / `"replace"` | 겹치는 기동을 오류로 알리거나, 새 기동이 현재 상태에서 이전 기동을 대체 |

수직 가속 크기 a는 필드 3 × acceleration_scale × g다. 삼각 프로파일은
기동 시작 수직 속도 v0에서 ±a를 T/2 동안 적용하고 반대 가속도를 T/2 동안 적용한다.
고도 변화는 Δh=v0T±aT²/4이며 T의 양의 해를 사용한다. 가속 유지 후보는 Δh=v0T±aT²/2다.
수직 속도가 일정한 후보는 T=|Δh|/vertical_rate_mps다. 연속 모델의 종료 지점은 목표 고도로 맞춘다.
목표 도달 뒤 `tangent`라면 가속 후보와 일정 상승률 후보는 고도가 계속 변한다.
이는 구별해서 시험할 후보이며, 목표 고도를 유지한다는 뜻이 아니다.

`discrete_triangular`는 위 연속 삼각식으로 공칭 종료 시간 T를 구하고, 기동 경과 시간
0, dt, 2dt, ... 중 T보다 작은 시각마다 `vz += acceleration * dt`로 갱신한다.
T/2 이전 갱신은 가속, 이후는 감속이며 갱신 사이 위치는 현재 속도로 적분한다.
종료 시 고도와 vz를 덮어쓰지 않으므로, 가속·감속 횟수가 다르면 남은 vz로 계속 상승/하강한다.
총속력 옵션에서는 이 vz에 따라 수평 속력을 계산한다. 시작 시각을 포함하는 갱신 규칙도 후보의 일부다.
이산 시간 모의기 내부를 확인한 것은 아니다. 0.1초 후보가 단일 종류 1의 고도 초과와 종류 2의
지속 하강을 설명하지만 모든 거리·도플러 항목이 개선되지는 않으며, 원본의 선회 오차도 남는다.
공칭 종료 시간과 가속도 적용 순서에 대한 확정 검증은 남아 있다.

회전 가속도 후보는 기동 진입 수평 속력 vh와 기준 heading α, 부호 있는 ω를 고정하고
기동 경과 시간 s에서 `v(s) = v(0) + vh(d(α+ωs)−d(α))`, `d(θ)=(sinθ,cosθ)`로 계산한다.
위치는 이 식의 해석적 적분이며, 종료 후 계산된 속도 벡터로 계속 운동한다.
종료 시간은 목표 각도/|ω|다. 기준 방향이 진입 heading과 다르면 실제 속도 방향 변화는 입력 목표 각도와 다를 수 있다.
null 기준은 기존 평면 등속력 선회와 동등하다. 고정 90도 후보는 단독 종류 4의 가속도 곡선에서
유도했으며 일부 잔차를 줄이지만 전체 모의기 모델로 확정하지 않았다. 기본 모델과 기존 후보 파일은 유지한다.
종류 3 단독 실험에 같은 규칙을 적용하면 종료 후 거리 RMS 약 1053m가 남는다.
좌우 선회를 모두 설명하는 규칙으로 검증되지 않았으며, 실제 수치는 보고서에 기록했다.

순간 속력 결합 후보는 가속도 방향의 기준각 ψ와 속도 방향을 따로 갖는다.
우측을 양으로 하는 부호 있는 가속도 a에 대해 `dvE/dt=a cosψ`, `dvN/dt=−a sinψ`,
`dψ/dt=a/hypot(vE,vN)`이며 위치 미분은 해당 속도다. 수직 속도는 이 수평 선회 중 유지한다.
RK4로 적분하며, 숫자 적분 간격을 바꿔 위치·속도의 수렴을 검사했다.
공칭 시간은 `turn_duration_speed`에서 선택한 속력×목표각/|a|이며 순간 회전율과 별도로 정한다.
0.1초 올림과 고정 90도 초기화가 단독 105도 좌우 선회의 속력 변화와 누적 오차를 함께 설명했다.
90도 실험에서는 같은 후보가 등속력 원호와 일치하며 목표각의 약 0.12도 초과를 설명한다.
연속 기동에서 기준각 유지와 TARGET 초기 속력으로 시간을 계산하는 후보도 비교했다.
원본의 8개 표적은 진입 heading 초기화, 표적 3·9는 고정 90도 초기화에서 맞는다.
이 차이를 가르는 공통 초기화 규칙은 확인하지 못했다. 표적마다 유리한 옵션만 고른 결과는
전체 정답의 검증 성공이 아니다. 기본 모델·기존 후보·미확정 출력 상태는 유지한다.

`integration_step_s`는 최대 적분 간격이다. 기동 시작·삼각 프로파일 중간·종료에서 간격을 나누고
중점 적분한다. 평면 일정 속력 선회는 원호를 정확히 적분하고, 직진도 정확히 적분한다.
수직 기동의 총속력 유지와 구면 경로는 수치 근사다. 간격을 줄여 수렴을 확인해야 한다.
속력보다 큰 수직 속도, 불가능한 초기 거리·고도, 기동 방향과 반대인 절대 목표 등은 오류다.
오류가 난 후보를 모의기 미탐지나 검증 통과로 취급하면 안 된다.

상승각을 회전시키는 수직 평면 선회, 가속도 제한 해제, 항력·추력, 지구 타원체,
기동 시작시간의 다른 필드 배치 등은 구현하지 않았다. 실제 플롯이 후보들과 맞지 않으면
모델을 추가해야 하며 현재 메뉴 중 하나를 억지로 확정하면 안 된다.
