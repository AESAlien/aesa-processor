#!/usr/bin/env python3
"""BeamTable 변환 시각화 / 검증 도구.

처리부는 "고정좌표계로 정의된 탐색 그리드(189개)"를 안테나 자세정보로
"안테나 좌표계 각도"로 바꿔서 BeamTable 에 보관한다. 이 도구는 그 과정을 3단계로 보여준다.

  (1) 왼쪽 그림  : 각도 값 비교. 점선 박스 = 처리부 고정좌표계로 정의된 탐색 영역
                   (방위각 +-45, 고각 0~41.2도), 주황 점 = 그 189개 빔을 안테나 좌표계로
                   변환해 BeamTable 에 저장한 각도. 대표 빔은 회색 화살표로
                   "고정 값 -> 안테나 값" 이동을 보여준다 (별 = 안테나 정면 (0, 0))
  (2) 가운데 3D  : (1)과 같은 비교를 3D 방향으로. 파란 점선 = 고정좌표계 탐색 영역의 윤곽,
                   주황 점 = BeamTable 의 안테나 각도를 방향으로 그린 것, 회색 화살표 = 대표 빔의
                   "고정 값 -> 안테나 값" 이동 (별 = 안테나 (0, 0) 방향)
  (3) 오른쪽 표  : 대표 빔(ID 1, 21, 95, 169, 189)의 "고정 (방위각, 고각) -> 안테나 (방위각, 고각)".
                   안테나 각도가 BeamTable 에 저장되어 빔 명령으로 나간다.
                   표 아래에 C++ 가 계산한 회전 행렬 R 을 같이 표시한다.
                   (안테나 각도 = R^T x 고정 방향)

값의 출처: BeamTable 각도(주황 점, 표의 antenna 열)와 회전 행렬 R 은 C++ 코드의 출력이다.
고정좌표계 값(점선 박스, 표의 fixed 열)은 문서의 그리드 규칙으로 이 스크립트가 만든 기준값이고,
numpy 계산은 C++ 결과를 검증하기 위한 독립 구현이다.

왼쪽 제목의 "round trip" 은 C++ 결과(안테나 각도)를 numpy로 다시 고정좌표계로
되돌렸을 때 원래 그리드와 얼마나 같은지(오차)를 나타낸다.
--verify 를 주면 numpy로 독립 계산한 변환 결과와도 비교하고,
오차가 허용치를 넘으면 종료 코드 1로 끝난다.
--csv 를 주면 189개 전체의 "고정 각도 / 안테나 각도"를 CSV로 저장한다 (자세별 파일).

사용 예 (aesa-processor 루트에서 빌드한 뒤):
  cmake --build build --target beam_table_dump
  python3 src/beam/test/visualize_beam_table.py --binary build/src/beam/beam_table_dump
  python3 src/beam/test/visualize_beam_table.py --binary build/src/beam/beam_table_dump \
          --att 0,0,0 --att 10,20,30 --verify --out beam_table.png
"""

import argparse
import csv
import io
import subprocess
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")  # 화면 없이도 PNG 저장 가능
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (3D 축 등록)

# ----- 고정 그리드 (docs: 방위각 21개 x 고각 9개) -----------------------------
AZ_COUNT, EL_COUNT = 21, 9
AZ_START, AZ_STEP = -42.0, 4.2
EL_START, EL_STEP = 3.0, 4.4

BEAM_HALF_DEG = 3.0   # 탐색 빔 폭 6도의 절반 (그리드 중심 -> 영역 가장자리)

# 변환 결과를 표/그림에 강조해서 보여줄 대표 빔 (1부터 시작하는 beamID): 네 모서리와 중심
LANDMARK_IDS = [1, 21, 95, 169, 189]

# ----- 색: 범주 슬롯 1(파랑)/2(주황), 텍스트는 중립색 --------------------------
C_FIXED = "#2a78d6"
C_ANT = "#eb6834"
C_TEXT = "#0b0b0b"
C_MUTED = "#52514e"
C_GRID = "#e4e3df"
C_SURFACE = "#fcfcfb"


def fixed_grid():
    """고정좌표계 그리드: idx = 고각 인덱스 * 21 + 방위각 인덱스."""
    idx = np.arange(AZ_COUNT * EL_COUNT)
    az = AZ_START + AZ_STEP * (idx % AZ_COUNT)
    el = EL_START + EL_STEP * (idx // AZ_COUNT)
    return az, el


def run_dump(binary, roll, pitch, yaw):
    proc = subprocess.run(
        [binary, str(roll), str(pitch), str(yaw)],
        capture_output=True, text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"beam_table_dump failed for att=({roll},{pitch},{yaw}): {proc.stderr.strip()}"
        )
    rows = list(csv.DictReader(io.StringIO(proc.stdout)))
    az = np.array([float(r["az_deg"]) for r in rows])
    el = np.array([float(r["el_deg"]) for r in rows])
    ids = np.array([int(r["beamID"]) for r in rows])
    return ids, az, el


def run_matrix(binary, roll, pitch, yaw):
    """C++ makeAntennaToEnu 가 만든 회전 행렬(안테나 -> ENU)."""
    proc = subprocess.run([binary, "--matrix", str(roll), str(pitch), str(yaw)],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"beam_table_dump --matrix failed: {proc.stderr.strip()}")
    return np.array([float(v) for v in proc.stdout.split()]).reshape(3, 3)


# ----- numpy 독립 계산 --------------------------------------------------------
def body_to_enu(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    rz = np.array([[np.cos(y), np.sin(y), 0], [-np.sin(y), np.cos(y), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
    ry = np.array([[np.cos(r), 0, np.sin(r)], [0, 1, 0], [-np.sin(r), 0, np.cos(r)]])
    return rz @ rx @ ry


def to_vec(az_deg, el_deg):
    az, el = np.radians(az_deg), np.radians(el_deg)
    return np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)])


def to_aed(v):
    az = np.degrees(np.arctan2(v[0], v[1]))
    el = np.degrees(np.arcsin(np.clip(v[2], -1.0, 1.0)))
    return az, el


def fixed_to_ant(az, el, att):
    """고정좌표계 각도 -> 안테나 좌표계 각도 (역변환 = 전치)."""
    return to_aed(body_to_enu(*att).T @ to_vec(az, el))


def ant_to_fixed(az, el, att):
    """안테나 좌표계 각도 -> 고정좌표계 각도."""
    return to_aed(body_to_enu(*att) @ to_vec(az, el))


def expected_table(att):
    return fixed_to_ant(*fixed_grid(), att)


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


# ----- 그리기 -----------------------------------------------------------------
def style_axis(ax, title):
    ax.set_facecolor(C_SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=C_TEXT)
    ax.set_xlabel("azimuth (deg)", color=C_MUTED, fontsize=9)
    ax.set_ylabel("elevation (deg)", color=C_MUTED, fontsize=9)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(colors=C_MUTED, labelsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(C_GRID)
    ax.set_aspect("equal", adjustable="box")


def draw_fixed_panel(ax, att, az_f, el_f, az_a, el_a, rt_err):
    """(1) 각도 값 비교: 고정좌표계 탐색 영역(점선 박스) vs BeamTable 의 안테나 각도(점)."""
    style_axis(ax, "(1) Angle values: fixed frame vs antenna frame\n"
                   f"round trip (antenna -> fixed) vs grid: {rt_err:.1e} deg")
    ax.set_xlabel("azimuth value (deg)", color=C_MUTED, fontsize=9)
    ax.set_ylabel("elevation value (deg)", color=C_MUTED, fontsize=9)

    # 고정좌표계 탐색 영역 = 그리드 중심 +- 빔 반폭(3도)  ->  방위각 +-45, 고각 0~41.2
    x0, x1 = az_f.min() - BEAM_HALF_DEG, az_f.max() + BEAM_HALF_DEG
    y0, y1 = el_f.min() - BEAM_HALF_DEG, el_f.max() + BEAM_HALF_DEG
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor=C_FIXED,
                           linewidth=1.6, linestyle=(0, (5, 3)), zorder=2))

    # 대표 빔: 고정 값(속 빈 파란 원) -> 안테나 값(주황 점) 이동
    for bid in LANDMARK_IDS:
        i = bid - 1
        ax.annotate("", xy=(az_a[i], el_a[i]), xytext=(az_f[i], el_f[i]),
                    arrowprops=dict(arrowstyle="->", color=C_MUTED, linewidth=0.8,
                                    alpha=0.7, shrinkA=3, shrinkB=3), zorder=3)
        ax.scatter(az_f[i], el_f[i], s=46, facecolor=C_SURFACE, edgecolor=C_FIXED,
                   linewidth=1.4, zorder=4)

    ax.plot(az_a, el_a, color=C_ANT, linewidth=0.5, alpha=0.35, zorder=1)
    ax.scatter(az_a, el_a, s=14, color=C_ANT, edgecolor=C_SURFACE, linewidth=0.8, zorder=5)
    ax.scatter([0], [0], s=110, marker="*", color=C_TEXT, edgecolor=C_SURFACE,
               linewidth=0.8, zorder=6)
    for bid in LANDMARK_IDS:
        i = bid - 1
        ax.annotate(str(bid), (az_a[i], el_a[i]), xytext=(7, 5), textcoords="offset points",
                    fontsize=8, color=C_TEXT, zorder=7)

    handles = [
        Line2D([0], [0], color=C_FIXED, linewidth=1.6, linestyle=(0, (5, 3))),
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor=C_SURFACE,
               markeredgecolor=C_FIXED, markeredgewidth=1.4, markersize=6),
        Line2D([0], [0], marker="o", linestyle="", color=C_ANT, markersize=4),
        Line2D([0], [0], marker="*", linestyle="", color=C_TEXT, markersize=9),
    ]
    labels = [
        "fixed-frame values: search area (az +-45, el 0~41.2) processor wants to scan",
        "sample beams' fixed values  (gray arrow -> their antenna values)",
        "antenna-frame values: 189 beams stored in BeamTable",
        "antenna boresight = antenna (0, 0)",
    ]
    ax.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.0, -0.17), ncol=1,
              fontsize=7, frameon=False, labelcolor=C_MUTED)

    xs = np.concatenate([[x0, x1], az_a, [0]])
    ys = np.concatenate([[y0, y1], el_a, [0]])
    ax.set_xlim(xs.min() - 8, xs.max() + 8)
    ax.set_ylim(ys.min() - 8, ys.max() + 8)


def draw_axes_panel(ax, att, az_f, el_f, az_a, el_a):
    """(2) 3D: (1)과 같은 비교. 고정 영역 윤곽(점선) vs BeamTable 각도(점)."""
    ax.set_title("(2) Same comparison in 3D (unit-sphere directions)", loc="left",
                 fontsize=10, color=C_TEXT)

    # 기준 축 (동/북/위) 과 지평선(고각 0도 원)
    for vec, lab in ((np.array([1, 0, 0]), "E"), (np.array([0, 1, 0]), "N"), (np.array([0, 0, 1]), "U")):
        ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=C_MUTED, linewidth=1.0)
        ax.text(vec[0] * 1.12, vec[1] * 1.12, vec[2] * 1.12, lab, color=C_MUTED, fontsize=8)
    t = np.linspace(0, 360, 181)
    h = to_vec(t, np.zeros_like(t))
    ax.plot(h[0], h[1], h[2], color=C_GRID, linewidth=1.0)

    # 고정좌표계 탐색 영역 윤곽 (방위각 +-45, 고각 0~41.2)
    x0, x1 = az_f.min() - BEAM_HALF_DEG, az_f.max() + BEAM_HALF_DEG
    y0, y1 = el_f.min() - BEAM_HALF_DEG, el_f.max() + BEAM_HALF_DEG
    n = 40
    ea = np.concatenate([np.linspace(x0, x1, n), np.full(n, x1), np.linspace(x1, x0, n), np.full(n, x0)])
    ee = np.concatenate([np.full(n, y0), np.linspace(y0, y1, n), np.full(n, y1), np.linspace(y1, y0, n)])
    o = to_vec(ea, ee)
    ax.plot(o[0], o[1], o[2], color=C_FIXED, linewidth=1.6, linestyle=(0, (5, 3)))

    # BeamTable 각도(안테나 값) 189개를 방향으로 표시
    v = to_vec(az_a, el_a)
    ax.plot(v[0], v[1], v[2], color=C_ANT, linewidth=0.5, alpha=0.35)
    ax.scatter(v[0], v[1], v[2], s=8, color=C_ANT, depthshade=False)

    # 대표 빔: 고정 값 -> 안테나 값
    for bid in LANDMARK_IDS:
        i = bid - 1
        f = to_vec(az_f[i], el_f[i])
        a = v[:, i]
        ax.plot([f[0], a[0]], [f[1], a[1]], [f[2], a[2]], color=C_MUTED, linewidth=0.8, alpha=0.8)
        ax.scatter(f[0], f[1], f[2], s=30, facecolor=C_SURFACE, edgecolor=C_FIXED,
                   linewidth=1.3, depthshade=False)
        ax.text(a[0], a[1], a[2] + 0.07, str(bid), color=C_TEXT, fontsize=7)

    # 안테나 정면 = 안테나 (0, 0) 방향
    ax.scatter([0], [1], [0], s=90, marker="*", color=C_TEXT, depthshade=False)

    ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-0.45, 1)
    ax.set_box_aspect((1, 1, 0.72))
    ax.view_init(elev=24, azim=-58)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.set_facecolor(C_SURFACE)
    ax.text2D(0.0, -0.06, "blue dashed: fixed-frame search area   orange: BeamTable angles   "
                          "star: antenna (0, 0)", transform=ax.transAxes, fontsize=7, color=C_MUTED)


def draw_table_panel(ax, att, az_f, el_f, az_a, el_a, R):
    """(3) 대표 빔 변환 표 + 회전 행렬."""
    roll, pitch, yaw = att
    ax.axis("off")
    ax.set_title("(3) Converted angles stored in BeamTable\n"
                 f"attitude: roll={roll:g}, pitch={pitch:g}, yaw={yaw:g}",
                 loc="left", fontsize=10, color=C_TEXT)

    rows = []
    for bid in LANDMARK_IDS:
        i = bid - 1
        rows.append([str(bid),
                     f"({az_f[i]:6.1f}, {el_f[i]:5.1f})",
                     "->",
                     f"({az_a[i]:6.1f}, {el_a[i]:5.1f})"])
    table = ax.table(cellText=rows,
                     colLabels=["beam ID", "fixed (az, el)", "", "antenna (az, el)"],
                     colWidths=[0.16, 0.33, 0.08, 0.35], loc="upper center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.7)
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor(C_GRID)
        cell.set_facecolor(C_SURFACE)
        cell.get_text().set_color(C_TEXT if r else C_MUTED)
        if r and c == 3:
            cell.get_text().set_fontweight("bold")   # BeamTable 에 저장되는 값

    mat = "\n".join("[ " + "  ".join(f"{v:6.3f}" for v in row) + " ]" for row in R)
    ax.text(0.0, 0.34, "R (antenna -> fixed), computed by the C++ code:", transform=ax.transAxes,
            fontsize=8, color=C_MUTED, va="top")
    ax.text(0.0, 0.29, mat, transform=ax.transAxes, fontsize=8, color=C_TEXT, va="top",
            family="monospace")
    ax.text(0.0, 0.13, "antenna angle = R^T x fixed direction\n"
                       "(bold column = values stored in BeamTable)",
            transform=ax.transAxes, fontsize=8, color=C_MUTED, va="top")


def parse_att(text):
    try:
        r, p, y = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError("--att 형식: roll,pitch,yaw (예: 10,20,30)")
    return (r, p, y)


def write_csv(path, att, az_f, el_f, ids, az_a, el_a):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["beamID", "fixed_az_deg", "fixed_el_deg", "antenna_az_deg", "antenna_el_deg"])
        for k in range(len(ids)):
            w.writerow([ids[k], f"{az_f[k]:.4f}", f"{el_f[k]:.4f}", f"{az_a[k]:.6f}", f"{el_a[k]:.6f}"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--binary", required=True, help="beam_table_dump 실행 파일 경로")
    ap.add_argument("--att", action="append", type=parse_att,
                    help="자세 roll,pitch,yaw (여러 번 지정 가능, 기본: 3가지)")
    ap.add_argument("--out", default="beam_table.png", help="저장할 PNG 경로")
    ap.add_argument("--csv", metavar="PREFIX",
                    help="189개 전체 변환값을 PREFIX_roll_pitch_yaw.csv 로 저장")
    ap.add_argument("--verify", action="store_true",
                    help="numpy 독립 계산과 비교 (허용 오차 초과 시 exit 1)")
    ap.add_argument("--tol", type=float, default=1e-3, help="허용 오차(deg)")
    args = ap.parse_args()

    atts = args.att or [(0, 0, 0), (0, 0, 20), (10, 20, 30)]

    fig = plt.figure(figsize=(16, 5.2 * len(atts)), layout="constrained")
    fig.patch.set_facecolor(C_SURFACE)
    gs = fig.add_gridspec(len(atts), 3, width_ratios=[1.5, 1.0, 1.25])

    az_f, el_f = fixed_grid()
    failed = False
    for row, att in enumerate(atts):
        ids, az, el = run_dump(args.binary, *att)
        R_cpp = run_matrix(args.binary, *att)

        assert len(ids) == AZ_COUNT * EL_COUNT, f"expected 189 beams, got {len(ids)}"
        assert list(ids) == list(range(1, 190)), "beamID must be 1..189"

        # C++ 결과(안테나 각도)를 다시 고정좌표계로 되돌려 원래 그리드와 비교
        az_back, el_back = ant_to_fixed(az, el, att)
        rt_err = max(np.abs(angle_diff(az_back, az_f)).max(), np.abs(el_back - el_f).max())
        failed |= rt_err > args.tol

        b_az, b_el = ant_to_fixed(np.array([0.0]), np.array([0.0]), att)
        line = (f"att(roll,pitch,yaw)=({att[0]:g},{att[1]:g},{att[2]:g})  beams=189  "
                f"antenna boresight in fixed frame=(az {b_az[0]:.1f}, el {b_el[0]:.1f})  "
                f"round-trip err={rt_err:.2e} deg [{'OK' if rt_err <= args.tol else 'FAIL'}]")

        if args.verify:
            az_e, el_e = expected_table(att)
            err = max(np.abs(angle_diff(az, az_e)).max(), np.abs(el - el_e).max())
            r_err = np.abs(R_cpp - body_to_enu(*att)).max()
            failed |= (err > args.tol) or (r_err > 1e-6)
            line += (f"  max|C++ - numpy|={err:.2e} deg [{'OK' if err <= args.tol else 'FAIL'}]"
                     f"  R max diff={r_err:.1e} [{'OK' if r_err <= 1e-6 else 'FAIL'}]")
        print(line)

        if args.csv:
            path = f"{args.csv}_{att[0]:g}_{att[1]:g}_{att[2]:g}.csv"
            write_csv(path, att, az_f, el_f, ids, az, el)
            print(f"saved: {path}")

        draw_fixed_panel(fig.add_subplot(gs[row, 0]), att, az_f, el_f, az, el, rt_err)
        draw_axes_panel(fig.add_subplot(gs[row, 1], projection="3d"), att, az_f, el_f, az, el)
        draw_table_panel(fig.add_subplot(gs[row, 2]), att, az_f, el_f, az, el, R_cpp)

    fig.suptitle("BeamTable: attitude -> rotation -> fixed-frame search grid converted to antenna angles",
                 x=0.01, ha="left", fontsize=12, color=C_TEXT)
    fig.savefig(args.out, dpi=130, facecolor=fig.get_facecolor())
    print(f"saved: {args.out}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
