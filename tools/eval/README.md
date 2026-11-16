# 시나리오 궤적 계산 도구

Python 3.10 이상, 표준 라이브러리만 사용한다. 저장소 루트에서 실행한다.
**실제 플롯으로 부분 검증한 가설 계산기다. 전체 완료 기준은 아직 통과하지 못했다.**
기본값과 `model.example.json`은 설명용 가정이며, 첫 실험 보정 후보는 `manual_run_1_candidate.json`이다.
특히 표적 3·9의 기동 모델에는 큰 오차가 남아 있다.
진행 상태와 수동 실행 절차는 [report.md](report.md)에 있다.

완만한 6표적 시나리오를 같은 모델·시계로 두 구간 검증한 결과와 바로 사용할 옵션은
[6표적 검증 보고서](multi_target_moderate_report.md)에 있다. 검증한 조건에서 거리 잔차 RMS는
6.771~8.012m다. 수 m 산포를 고려하는 비교용 기준 궤적으로 조건부 사용할 수 있으며,
원래 속력×20ms 최대 오차 기준과 임의의 ini에 대한 전체 검증은 아직 미충족이다.
시나리오 위상 초를 직접 입력하는 예시는 다음과 같다.

```powershell
python -m tools.eval predict --ini tools/eval/scenarios/multi_target_moderate_6_in_view.ini --options tools/eval/multi_target_moderate_phase_options.json --target 6 --time 10 --time 35 --time 75
```

이 예시의 원점은 시나리오 0초다. 실제 CSV 시각에는 해당 처리부의 시간 원점 정렬이 필요하다.
250~320m/s·0.2~0.4g·초기 heading 0/90/270도의 이번 입력 범위 밖으로 검증을 확대 해석하지 않는다.

```powershell
python -m tools.eval inspect --ini "C:\Users\user\Downloads\Target_10_2.ini"
python -m tools.eval predict --ini "C:\Users\user\Downloads\Target_10_2.ini" --options tools/eval/model.example.json --target 1 --time 0 --time 4 --time 100
python -m unittest discover -s tools/eval/tests -v
python -m compileall -q tools/eval
```

`--time`은 처리부 CSV의 `transmitTime_ms / 1000`이다. `--target`, `--time`은 반복 지정할 수 있고,
`--target`을 생략하면 모든 표적을 계산한다. 결과는 JSON이며, `--output <파일>`로 저장한다.
부모 디렉터리는 미리 만들어야 한다. `--encoding cp949`처럼 입력 인코딩도 지정할 수 있다.
CR, LF, CRLF 줄 구분을 지원하며 원본 ini는 변경하지 않는다.

설정을 복사해 가설별로 보관한다. 모든 기본 옵션을 새로 출력하려면 `options` 명령을 쓴다.

```powershell
New-Item -ItemType Directory -Force tools/eval/output | Out-Null
python -m tools.eval options --output tools/eval/output/model.json
```

`output/model.json`에서 예를 들어 노트 가설은 `speed_scale_mps=0.5144444444444445`,
해리 가설은 `range_scale_m=1852`, ft 고도 가설은 `altitude_scale_m=0.3048`로 지정한다.
필드 6이 ft/min이면 `vertical_kind="rate"`, `vertical_scale=0.00508`이다.
나머지 옵션과 운동식은 [model_options.md](model_options.md)를 참고한다. 알 수 없는 옵션 이름은 오류다.

API로도 사용할 수 있다.

```python
from tools.eval.scenario import load_scenario
from tools.eval.truth import ModelOptions, TruthCalculator

scenario = load_scenario(r"C:\Users\user\Downloads\Target_10_2.ini")
options = ModelOptions.load("tools/eval/model.example.json")
result = TruthCalculator(scenario, options).predict(target_number=1, processor_time_s=4.0)
print(result.position_enu_m, result.velocity_enu_mps)
print(result.slant_range_m, result.azimuth_deg, result.elevation_deg, result.doppler_velocity_mps)
```

직교 출력은 항상 레이더 원점 ENU(동·북·상), 단위는 m와 m/s다. 각도는 도,
방위각은 `[-180, 180)`이다. 양의 도플러는 기본적으로 멀어지는 방향이며 `doppler_sign`으로 바꾼다.
결과에는 `active`, 적용 중인 `maneuver_kind`, `model_status="unverified_hypothesis"`와
전체 계산 옵션이 남는다. 시작 전·종료 후에는 위치를 경계 시각에 고정하고 속도·도플러를 0으로 돌려준다.
이는 미출현 표적의 위치를 정답으로 쓰지 않도록 하는 API 규약이며 모의기 동작을 확인한 결과가 아니다.

`scenario.py`는 구조·원시 필드를, `geometry.py`는 좌표 변환을, `truth.py`는 운동 가설을 담당한다.
Python 코드는 Python 명명 관례를 따르며 `CONTRIBUTING.md`의 C++ 전용 명명 규칙을 적용하지 않는다.

## 실제 실험 분석 재현

사용자가 수동으로 확보한 두 운용 구간의 원본 사본은 `data/manual_20261116_2228/`,
`data/manual_20261116_2249/`에 있다. `data/`, `output/`은 Git에서 제외하므로 새 체크아웃에는 없다.
원본의 실제 경로·SHA-256은 보고서에 기록했다. 그래프 생성에는 `matplotlib`이 추가로 필요하며,
파서·계산·매칭·통계·후보 비교는 표준 라이브러리만 사용한다.

```powershell
$first = "tools/eval/data/manual_20261116_2228"
$second = "tools/eval/data/manual_20261116_2249"
python -m tools.eval.analyze --csv "$first/detections.csv" --start-ms 4379948 --end-ms 4511052 --ini "$first/scenario.ini" --options tools/eval/manual_run_1_candidate.json --mapping tools/eval/manual_run_1_mapping.json --output tools/eval/output/run_1 --plots
python -m tools.eval.analyze --csv "$second/detections.csv" --start-ms 5688456 --end-ms 5936144 --ini "$second/scenario.ini" --options tools/eval/manual_run_1_candidate.json --mapping tools/eval/manual_run_2_mapping.json --output tools/eval/output/run_2 --plots
python -m tools.eval.compare_runs --first tools/eval/output/run_1 --second tools/eval/output/run_2 --output tools/eval/output/repeat_comparison
```

`analyze`는 `assignments.csv`, `residuals.csv`, `summary.json`을 생성한다.
새 로그에서는 `--ini/--options/--mapping` 없이 먼저 실행해 연속 궤적을 검토하고,
초기 위치·고도·반복 연속성을 확인한 뒤 `{"연속 궤적 번호": TARGET번호}` 형식의 매핑을 지정한다.
기존 매핑은 해당 CSV와 선택 구간, 기본 매칭 게이트에만 유효하다.
잔차가 작아지도록 표적 대응을 매 후보마다 바꾸지 않는다. 미대응·모호성·비활성 예측 행은 JSON에 별도 기록한다.

초기 직진 후보 순위와 기동 후보 비교를 재현하려면 다음을 실행한다.

```powershell
python -m tools.eval.fit_straight --csv "$first/detections.csv" --ini "$first/scenario.ini" --mapping tools/eval/manual_run_1_mapping.json --start-ms 4379948 --end-ms 4511052 --phase-origin-s 4304.2546 --period-s 100.1 --output tools/eval/output/run_1
python -m tools.eval.compare_models --csv "$first/detections.csv" --ini "$first/scenario.ini" --options tools/eval/manual_run_1_candidate.json --mapping tools/eval/manual_run_1_mapping.json --start-ms 4379948 --end-ms 4511052 --output tools/eval/output/run_1/model_comparison.json
```

`fit_straight`는 같은 표본에 보정한 결과다. 시간 선택의 초기 위상과 매핑은 검토 입력이며, 후보 순위만으로
모든 필드를 확정하지 않는다. `compare_runs`는 같은 후보 옵션을 강제하고 두 번째 표본을 재보정하지 않는다.
1.2초 이하 간격의 관측만 위상별로 선형 보간하며, 비교에는 측정·보간·시간 정렬 오차가 포함된다.

새 원본을 보존하려면 `python -m tools.eval.intake --help`를 참고한다.
기존 실험 디렉터리/사본은 덮어쓰지 않으며 원본 바이트·해시·선택 구간을 보존한다.
`make_simple_scenarios`로 원본에서 단일 표적 직진/종류별 기동 파일을 만들 수 있다.
이미 생성한 `scenarios/target_3_*.ini` 다섯 파일 중 종류 1~4는 모두 사용자가 수동 실행해 분석했다.
직진만 있는 파일의 단독 실행은 아직 확인하지 못했다. 전체 검증 기준은 미충족이다.

종류 1 단독 실험은 다음과 같이 재현한다. 처리부 재시작 후 초기 직진 19개 표본으로 시계만 보정하며,
`--clock-options`는 기존 물리 옵션을 유지한다. 이는 같은 자료에 대한 보정이다.

```powershell
$kind1 = "tools/eval/data/manual_20261117_kind_1"
python -m tools.eval.fit_straight --csv "$kind1/detections.csv" --ini "$kind1/scenario.ini" --mapping tools/eval/manual_kind_1_mapping.json --start-ms 55957 --end-ms 206389 --phase-origin-s 0 --period-s 100.1 --clock-options tools/eval/manual_run_1_candidate.json --output tools/eval/output/kind_1
python -m tools.eval.analyze --csv "$kind1/detections.csv" --ini "$kind1/scenario.ini" --options tools/eval/output/kind_1/straight_options.json --mapping tools/eval/manual_kind_1_mapping.json --start-ms 55957 --end-ms 206389 --output tools/eval/output/kind_1 --plots
```

`manual_kind_1_candidate.json`은 위 시간 보정 결과를 보관한 후보다. `summary.json`의 `post_maneuver`는
해당 기동 시작 후 다음 기동이 시작되기 전까지, 후보 모델에서 기동이 끝난 표본의 잔차다.
단일 표적의 `--plots` 출력에는 `height.png/.svg`도 포함한다. 모든 잔차는 관측−계산이다.

종류 4 단독 실험은 다음과 같이 재현한다. 초기 직진이 한 주기에만 있어 원점만 보정되고
기존 반복 주기를 유지한다(`straight_fit.json`의 `period_identifiable_from_selection=false`).
두 후보는 같은 시간 보정·대응을 사용한다. 새 회전 가속도 후보는 일부 오차를 줄이지만 미확정이다.

```powershell
$kind4 = "tools/eval/data/manual_20261117_kind_4"
python -m tools.eval.fit_straight --csv "$kind4/detections.csv" --ini "$kind4/scenario.ini" --mapping tools/eval/manual_kind_4_mapping.json --start-ms 1032617 --end-ms 1183720 --phase-origin-s -0.016035843655701168 --period-s 100.1107252251997 --clock-options tools/eval/manual_kind_1_candidate.json --output tools/eval/output/kind_4
python -m tools.eval.analyze --csv "$kind4/detections.csv" --ini "$kind4/scenario.ini" --options tools/eval/manual_kind_4_candidate.json --mapping tools/eval/manual_kind_4_mapping.json --start-ms 1032617 --end-ms 1183720 --output tools/eval/output/kind_4 --plots
python -m tools.eval.analyze --csv "$kind4/detections.csv" --ini "$kind4/scenario.ini" --options tools/eval/manual_kind_4_rotating_candidate.json --mapping tools/eval/manual_kind_4_mapping.json --start-ms 1032617 --end-ms 1183720 --output tools/eval/output/kind_4_rotating --plots
python -m tools.eval.compare_models --csv "$kind4/detections.csv" --ini "$kind4/scenario.ini" --options tools/eval/manual_kind_4_candidate.json --mapping tools/eval/manual_kind_4_mapping.json --start-ms 1032617 --end-ms 1183720 --stride 1 --output tools/eval/output/kind_4/model_comparison.json
python -m tools.eval.fit_velocity --residuals tools/eval/output/kind_4/residuals.csv --target 1 --from-phase 30 --to-phase 90 --output tools/eval/output/kind_4/observed_velocity.json
```

`fit_velocity`는 선택한 위상 구간의 **관측 ENU 위치**를 각각 시간의 일차식에 회귀하고 속도와
성분별 위치 잔차 RMS를 낸다. 등속 직진 여부는 별도로 검토해야 한다. 반복 위상을 겹쳐 계산하므로
독립 실험 수로 해석하지 않는다. 단일 표적 `--plots`에는 `trajectory.png/.svg`도 생성된다.
새 후보를 원본 첫 실험에서 비교한 결과는 앞의 `compare_models` 명령에서 출력 경로를
`tools/eval/output/run_1/model_comparison_with_rotating.json`으로 지정해 재현할 수 있다.

종류 3 단독 실험은 다음 명령으로 재현한다. 새 처리부 시계는 초기 직진 26개로 보정하고,
종류 4에서 선택한 회전 가속도 물리 옵션을 그대로 비교했다. 이 후보도 종류 3의 누적 오차를 해결하지 못한다.

```powershell
$kind3 = "tools/eval/data/manual_20261117_kind_3"
python -m tools.eval.fit_straight --csv "$kind3/detections.csv" --ini "$kind3/scenario.ini" --mapping tools/eval/manual_kind_3_mapping.json --start-ms 50766 --end-ms 293811 --phase-origin-s 0 --period-s 100.1 --clock-options tools/eval/manual_run_1_candidate.json --output tools/eval/output/kind_3
python -m tools.eval.analyze --csv "$kind3/detections.csv" --ini "$kind3/scenario.ini" --options tools/eval/manual_kind_3_candidate.json --mapping tools/eval/manual_kind_3_mapping.json --start-ms 50766 --end-ms 293811 --output tools/eval/output/kind_3 --plots
python -m tools.eval.analyze --csv "$kind3/detections.csv" --ini "$kind3/scenario.ini" --options tools/eval/manual_kind_3_rotating_candidate.json --mapping tools/eval/manual_kind_3_mapping.json --start-ms 50766 --end-ms 293811 --output tools/eval/output/kind_3_rotating --plots
python -m tools.eval.compare_models --csv "$kind3/detections.csv" --ini "$kind3/scenario.ini" --options tools/eval/manual_kind_3_candidate.json --mapping tools/eval/manual_kind_3_mapping.json --start-ms 50766 --end-ms 293811 --stride 1 --output tools/eval/output/kind_3/model_comparison.json
python -m tools.eval.fit_velocity --residuals tools/eval/output/kind_3/residuals.csv --target 1 --from-phase 40 --to-phase 90 --output tools/eval/output/kind_3/observed_velocity.json
```

공통 라이브러리는 이번 실험에 맞춰 바꾸지 않았다. 종류 3 후보 파일 두 개는 같은 새 시계를 쓰며,
회전 가속도 기준 90도를 포함한 운동 규칙을 종류 4와 다르게 조정하지 않았다.

종류 2 단독 하강은 다음과 같이 재현한다. 기존 연속 후보와 0.1초 수직 속도 갱신 후보를
같은 시계·대응으로 비교했다. 새 후보는 종료 시 고도와 vz를 강제로 보정하지 않는다.

```powershell
$kind2 = "tools/eval/data/manual_20261117_kind_2"
python -m tools.eval.fit_straight --csv "$kind2/detections.csv" --ini "$kind2/scenario.ini" --mapping tools/eval/manual_kind_2_mapping.json --start-ms 857784 --end-ms 1067047 --phase-origin-s 0.00300591188740965 --period-s 100.09666978138864 --clock-options tools/eval/manual_kind_1_candidate.json --output tools/eval/output/kind_2
python -m tools.eval.analyze --csv "$kind2/detections.csv" --ini "$kind2/scenario.ini" --options tools/eval/manual_kind_2_candidate.json --mapping tools/eval/manual_kind_2_mapping.json --start-ms 857784 --end-ms 1067047 --output tools/eval/output/kind_2 --plots
python -m tools.eval.analyze --csv "$kind2/detections.csv" --ini "$kind2/scenario.ini" --options tools/eval/manual_kind_2_discrete_candidate.json --mapping tools/eval/manual_kind_2_mapping.json --start-ms 857784 --end-ms 1067047 --output tools/eval/output/kind_2_discrete --plots
python -m tools.eval.fit_velocity --residuals tools/eval/output/kind_2/residuals.csv --target 1 --from-phase 30 --to-phase 90 --output tools/eval/output/kind_2/observed_velocity.json
python -m tools.eval.compare_models --csv "$kind2/detections.csv" --ini "$kind2/scenario.ini" --options tools/eval/manual_kind_2_candidate.json --mapping tools/eval/manual_kind_2_mapping.json --start-ms 857784 --end-ms 1067047 --stride 1 --output tools/eval/output/kind_2/model_comparison.json
```

같은 이산 규칙을 이전 상승에 적용한 결과도 보관했다. 다음 명령은 앞에서 설정한 `$kind1`을 사용한다.

```powershell
python -m tools.eval.analyze --csv "$kind1/detections.csv" --ini "$kind1/scenario.ini" --options tools/eval/manual_kind_1_discrete_candidate.json --mapping tools/eval/manual_kind_1_mapping.json --start-ms 55957 --end-ms 206389 --output tools/eval/output/kind_1_discrete --plots
```

`manual_kind_1_discrete_candidate.json`과 `manual_kind_2_discrete_candidate.json`은 각각 기존 후보에서
`vertical_model="discrete_triangular"`, `vertical_update_step_s=0.1`만 바꾼 파일이다.
종류 2의 잔여 하강 속도를 보고 이 간격을 선택했으므로 독립 검증으로 해석하지 않는다.
이전 종류 1에 적용할 때 물리 계수·시계·목표값을 다시 맞추지 않았다.
원본 첫 실험의 같은 후보 비교는 앞의 `compare_models` 명령에서 출력 경로를
`tools/eval/output/run_1/model_comparison_with_discrete_vertical.json`으로 지정해 재현할 수 있다.

## GUI 추가 실험과 새 선회 후보

사용자의 직접 제어 요청 후 에이전트가 초기 heading 90도의 좌우 선회와 고정 Noise=0 비교를 실행했다.
시작 순서는 **전시기 Start → 모의기 모의 시작**이다. 시나리오·환경 설정은 전송 완료를 확인한 뒤 시작한다.
종료는 전시기 Stop과 모의기 모의 종료를 누른다. ini를 중간에 다시 불러오면 별도 구간으로 취급한다.
실제 보존 경로·해시와 조건은 [report.md](report.md)의 GUI 추가 실험 절에 있다.
Noise 값 0과 고정 체크+0 모두 수 m 산포가 남았으므로 오차 없는 플롯이라고 간주하지 않는다.

새 후보는 순간 속력으로 가속도 방향의 회전율을 바꾸며 공칭 종료 시간을 0.1초로 올림한다.
기본 모델과 이전 후보는 유지했다. 보정은 같은 자료의 초기 직진 시계에만 적용했으며,
좌우 비교에서 물리 계수나 대응을 다시 맞추지 않았다. GUI 자료 재분석 예시는 다음과 같다.

```powershell
$right = "tools/eval/data/gui_heading90_right"
python -m tools.eval.analyze --csv "$right/detections.csv" --ini "$right/scenario.ini" --mapping tools/eval/gui_heading90_right_mapping.json --options tools/eval/gui_heading90_right_coupled_candidate.json --start-ms 2463262 --end-ms 2606828 --output tools/eval/output/gui_heading90_right_coupled --plots
```

다른 GUI 구간도 위 명령의 자료 폴더·대응·옵션·시각을 아래 값으로 바꾸면 된다.

| 자료 폴더 (`data/` 아래) | 대응 파일 (`tools/eval/` 아래) | 옵션 파일 (`tools/eval/` 아래) | 시작~종료 ms |
| --- | --- | --- | --- |
| `gui_heading90_left` | `gui_heading90_left_mapping.json` | `gui_heading90_left_coupled_candidate.json` | 2742074~2894526 |
| `gui_heading90_right_fixed_noise_0` | `gui_heading90_right_fixed_noise_0_mapping.json` | `gui_heading90_right_coupled_candidate.json` | 3502904~3679565 |

고정 Noise 비교는 첫 우선회의 시계까지 그대로 유지한다. 이전 원호 후보를 비교하려면 GUI 우/좌 옵션을
각각 `gui_heading90_right_candidate.json`, `gui_heading90_left_candidate.json`으로 바꾸고 다른 출력 폴더를 쓴다.
`fit_velocity`는 앞 절과 같이 위상 30~90초에서 관측 속도를 계산할 수 있다.
기존 105도 단일 종류 3·4도 앞의 명령에서 옵션을 `manual_kind_3_coupled_candidate.json`,
`manual_kind_4_coupled_candidate.json`으로 바꿔 `kind_3_coupled`, `kind_4_coupled`에 출력한다.

원본 10표적의 연속 기동에서는 다음 두 초기화 후보를 비교했다. 둘 모두 초기 속력으로 공칭 시간을
계산하고 기준각을 다음 선회에 유지한다. 진입 heading 초기화와 고정 90도 초기화의 차이는 미해결이다.

```powershell
python -m tools.eval.analyze --csv "$first/detections.csv" --ini "$first/scenario.ini" --mapping tools/eval/manual_run_1_mapping.json --options tools/eval/manual_run_1_coupled_entry_candidate.json --start-ms 4379948 --end-ms 4511052 --output tools/eval/output/run_1_coupled_entry --plots
python -m tools.eval.analyze --csv "$first/detections.csv" --ini "$first/scenario.ini" --mapping tools/eval/manual_run_1_mapping.json --options tools/eval/manual_run_1_coupled_persistent_candidate.json --start-ms 4379948 --end-ms 4511052 --output tools/eval/output/run_1_coupled_persistent --plots
```

두 후보 중 표적별로 유리한 값만 모은 결과를 공통 모델 검증 성공으로 해석하지 않는다.
공통 초기화 규칙, 저속 표적의 20ms 최대 잔차 기준, 완전 재시작 2회 재현성은 미해결이다.
