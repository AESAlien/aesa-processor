#!/usr/bin/env python3
"""빔 스케줄러 동작 시각화 / 검증 도구.

beam_schedule_dump 가 Scheduler 를 시나리오로 실행해 틱(10ms)마다 어떤 빔이 나갔는지 CSV 로 출력하면,
이 도구가 그 결과를 그림으로 보여준다. 값은 모두 C++ Scheduler 의 출력이다.

시나리오 mixed (짧은 구간에서 요청이 끼어드는 순서와 폐기를 본다)
  (1) 위쪽 타임라인 : 아래 줄 = 10ms 틱마다 실제로 나간 빔(색/모양 = 종류, 속이 찬 마커).
                      위 줄 = 요청이 원한 시각(속이 빈 마커). 선은 요청 -> 실제 송신 틱을 잇고,
                      X 는 허용 지연을 넘겨 폐기된 요청이다. OFF / ON 이후 자세 대기 / READY 구간도 함께 표시.
  (2) 왼쪽 아래     : 안테나 각도 평면. 회색 점 = BeamTable 189개 탐색 구역, 파란 선/점 = 이번 구간에 송신된 탐색 빔
                      (순서대로), 주황/아쿠아 = 확인/추적 빔 (속이 빈 마커 = 폐기)
  (3) 오른쪽 아래   : 요청별 결과 표 (원한 시각, 송신된 틱, 지연, 결과)

시나리오 tracks (추적 20개가 1초 주기로 요청될 때 탐색 한 바퀴가 얼마나 늘어나는지 본다)
  (1) 위쪽          : 탐색 빔 번호(1~189)를 시간에 따라 그린 톱니. 점선 = 요청이 없을 때의 이상적인 진행,
                      아래 삼각형 = 추적 빔이 나가서 탐색이 멈춘 틱
  (2) 왼쪽 아래     : 첫 한 바퀴의 탐색 순서(밝은 색 -> 어두운 색)와 추적 빔 위치
  (3) 오른쪽 아래   : 한 바퀴 시간, 송신/폐기 요청 수

--verify 를 주면 출력이 설계 규칙과 맞는지 검사하고, 어긋나면 종료 코드 1로 끝난다
  (명령 카운트 연속, 탐색 번호 순환, 요청은 원한 시각 이후 허용 지연 이내에만 송신, 준비 전에는 송신 없음 등).

사용 예 (aesa-processor 루트에서 빌드한 뒤):
  cmake --build build --target beam_schedule_dump
  python3 src/beam/test/visualize_schedule.py --binary build/src/beam/beam_schedule_dump
  python3 src/beam/test/visualize_schedule.py --binary build/src/beam/beam_schedule_dump \
          --scenario mixed --att 10,20,30 --verify --out schedule
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
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D

# ----- 설계 값 ----------------------------------------------------------------
BEAM_COUNT = 189
TICK_MS = 10
# RequestQueue 의 종류별 허용 지연(request_queue.cpp)과 같아야 한다.
MAX_DELAY_MS = {"CONFIRMATION": 20, "TRACKING": 30}

# ----- 색: 범주 슬롯 1~3 (검증된 팔레트, 모양을 함께 써서 색에만 의존하지 않는다) ---------
C_SEARCH = "#2a78d6"
C_CONFIRM = "#eb6834"
C_TRACK = "#1baf7a"
C_TEXT = "#0b0b0b"
C_MUTED = "#52514e"
C_GRID = "#e4e3df"
C_DOT = "#c9c8c2"
C_SURFACE = "#fcfcfb"

TYPE_STYLE = {
    "SEARCH": (C_SEARCH, "o", "search"),
    "CONFIRMATION": (C_CONFIRM, "s", "confirmation"),
    "TRACKING": (C_TRACK, "^", "tracking"),
}

SEQ_BLUE = LinearSegmentedColormap.from_list(
    "seq_blue", ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])


# ----- 실행 / 파싱 --------------------------------------------------------------
def run_dump(binary, scenario, att):
    proc = subprocess.run([binary, "--scenario", scenario, *(str(v) for v in att)],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"beam_schedule_dump failed (scenario={scenario}, att={att}): {proc.stderr.strip()}")

    sections, name, lines = {}, None, []
    for line in proc.stdout.splitlines():
        if line.startswith("# "):
            if name:
                sections[name] = lines
            name, lines = line[2:].strip(), []
        elif line.strip():
            lines.append(line)
    if name:
        sections[name] = lines

    def rows(key):
        return list(csv.DictReader(io.StringIO("\n".join(sections[key]))))

    def number(text, cast=float):
        return cast(text) if text != "" else None

    ticks = [dict(t=int(r["tick_ms"]), status=r["status"], type=r["type"],
                  beam_id=number(r["beamID"], int), count=number(r["commandCount"], int),
                  az=number(r["az_deg"]), el=number(r["el_deg"]),
                  az_w=number(r["az_width_deg"]), el_w=number(r["el_width_deg"]))
             for r in rows("ticks")]
    requests = [dict(name=r["name"], type=r["type"], ts=int(r["timestamp_ms"]),
                     az=float(r["az_deg"]), el=float(r["el_deg"]), sent=int(r["sent_tick_ms"]))
                for r in rows("requests")]
    grid = [dict(idx=int(r["idx"]), beam_id=int(r["beamID"]), az=float(r["az_deg"]), el=float(r["el_deg"]))
            for r in rows("grid")]
    return dict(ticks=ticks, requests=requests, grid=grid)


def drop_tick(request, ticks):
    """폐기된 요청이 대기열에서 제거된 틱(허용 지연에 처음 도달한 틱)."""
    limit = request["ts"] + MAX_DELAY_MS[request["type"]]
    return next((tk["t"] for tk in ticks if tk["t"] >= limit), ticks[-1]["t"])


def first_cycle_ms(ticks):
    """탐색 1번 빔이 처음 나간 틱부터 다시 1번이 나간 틱까지의 시간(ms). 한 바퀴가 없으면 None."""
    starts = [tk["t"] for tk in ticks if tk["type"] == "SEARCH" and tk["beam_id"] == 1]
    return starts[1] - starts[0] if len(starts) >= 2 else None


# ----- 검증 ----------------------------------------------------------------------
def verify(scenario, data):
    ticks, requests = data["ticks"], data["requests"]
    failures = []

    sent = [tk for tk in ticks if tk["type"] != "NONE"]
    for expected, tk in enumerate(sent, start=1):
        if tk["count"] != expected:
            failures.append(f"commandCount at {tk['t']}ms is {tk['count']}, expected {expected}")
            break

    searches = [tk for tk in ticks if tk["type"] == "SEARCH"]
    for previous, current in zip(searches, searches[1:]):
        if current["beam_id"] != previous["beam_id"] % BEAM_COUNT + 1:
            failures.append(f"search beam order broken at {current['t']}ms "
                            f"({previous['beam_id']} -> {current['beam_id']})")
            break
    if searches and searches[0]["beam_id"] != 1:
        failures.append("first search beam is not beam 1")

    for tk in ticks:
        if tk["status"] != "READY" and tk["type"] != "NONE":
            failures.append(f"beam sent before READY at {tk['t']}ms")
            break
        if tk["status"] == "READY" and tk["type"] == "NONE":
            failures.append(f"no beam sent while READY at {tk['t']}ms")
            break
        if tk["type"] == "SEARCH" and not (tk["az_w"] == 6.0 and tk["el_w"] == 6.0):
            failures.append(f"search beam width is not 6 deg at {tk['t']}ms")
            break
        if tk["type"] in ("CONFIRMATION", "TRACKING") and not (
                tk["beam_id"] == 0 and tk["az_w"] == 3.0 and tk["el_w"] == 3.0):
            failures.append(f"request beam fields are wrong at {tk['t']}ms")
            break

    for r in requests:
        if r["sent"] < 0:
            continue
        delay = r["sent"] - r["ts"]
        if delay < 0 or delay >= MAX_DELAY_MS[r["type"]]:
            failures.append(f"request {r['name']} sent with delay {delay}ms (limit {MAX_DELAY_MS[r['type']]}ms)")

    dropped = sorted(r["name"] for r in requests if r["sent"] < 0)
    expected_dropped = ["D", "H"] if scenario == "mixed" else []
    if dropped != expected_dropped:
        failures.append(f"dropped requests {dropped}, expected {expected_dropped}")

    return failures


# ----- 공통 그리기 도구 -------------------------------------------------------------
def style_axis(ax, title):
    ax.set_facecolor(C_SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=C_TEXT)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(colors=C_MUTED, labelsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(C_GRID)


def hollow(ax, x, y, beam_type, size=58, zorder=5):
    color, marker, _ = TYPE_STYLE[beam_type]
    ax.scatter(x, y, s=size, marker=marker, facecolor=C_SURFACE, edgecolor=color, linewidth=1.7, zorder=zorder)


def filled(ax, x, y, beam_type, size=52, zorder=5):
    color, marker, _ = TYPE_STYLE[beam_type]
    ax.scatter(x, y, s=size, marker=marker, color=color, edgecolor=C_SURFACE, linewidth=0.8, zorder=zorder)


def sky_limits(grid):
    az = [g["az"] for g in grid]
    el = [g["el"] for g in grid]
    return (min(az) - 8, max(az) + 8), (min(el) - 8, max(el) + 8)


# ----- mixed ---------------------------------------------------------------------
def draw_timeline(ax, data):
    ticks, requests = data["ticks"], data["requests"]
    style_axis(ax, "(1) Timeline: what was asked (top row) vs what was actually sent each 10 ms tick (bottom row)")
    ax.grid(False, axis="y")

    # 상태 구간
    spans = []
    for tk in ticks:
        if spans and spans[-1][2] == tk["status"]:
            spans[-1][1] = tk["t"]
        else:
            spans.append([tk["t"], tk["t"], tk["status"]])
    # 구간이 짧아 라벨이 겹치므로 줄을 나누고 구간 왼쪽 끝에 맞춘다.
    labels = {"OFF": ("OFF: ignores all input", 2.8),
              "ON_WAIT_ATTITUDE": ("ON, waiting for attitude: sends nothing", 2.5),
              "READY": ("READY: one beam per tick", 2.8)}
    for start, end, status in spans:
        left, right = start - TICK_MS / 2, end + TICK_MS / 2
        if status != "READY":
            ax.axvspan(left, right, color=C_GRID, alpha=0.8 if status == "OFF" else 0.45, zorder=0)
        text, label_y = labels[status]
        ax.text(left + 1, label_y, text, ha="left", va="center", fontsize=8, color=C_MUTED)

    # 틱마다 실제로 나간 빔 (아래 줄)
    none_t = [tk["t"] for tk in ticks if tk["type"] == "NONE"]
    ax.scatter(none_t, [0] * len(none_t), s=14, marker=".", color=C_MUTED, zorder=3)
    for beam_type in TYPE_STYLE:
        xs = [tk["t"] for tk in ticks if tk["type"] == beam_type]
        filled(ax, xs, [0] * len(xs), beam_type, size=48 if beam_type == "SEARCH" else 62)

    # 요청 (위 줄): 한 틱(10ms) 안에 몰린 요청은 마커가 겹치지 않도록 위로 쌓는다
    ordered = sorted(requests, key=lambda item: item["ts"])
    levels = {r["name"]: sum(1 for q in ordered[:i] if r["ts"] - q["ts"] < TICK_MS)
              for i, r in enumerate(ordered)}
    for r in requests:
        y = 1.1 + 0.3 * levels[r["name"]]
        hollow(ax, r["ts"], y, r["type"])
        ax.annotate(r["name"], (r["ts"], y), xytext=(7, -1), textcoords="offset points",
                    fontsize=8, color=C_TEXT, va="center", zorder=6)
        if r["sent"] >= 0:
            ax.plot([r["ts"], r["sent"]], [y - 0.08, 0.1], color=C_MUTED, linewidth=0.8, alpha=0.7, zorder=2)
        else:
            t_drop = drop_tick(r, ticks)
            ax.plot([r["ts"], t_drop], [y - 0.08, 0.5], color=C_MUTED, linewidth=0.8, linestyle=(0, (3, 2)),
                    zorder=2)
            ax.scatter(t_drop, 0.5, s=80, marker="X", color=C_TEXT, edgecolor=C_SURFACE, linewidth=0.8, zorder=6)
            ax.annotate("dropped", (t_drop, 0.5), xytext=(8, 0), textcoords="offset points",
                        fontsize=8, color=C_TEXT, va="center", zorder=6)

    ax.set_ylim(-0.6, 3.05)
    ax.set_xlim(ticks[0]["t"] - 8, ticks[-1]["t"] + 8)
    ax.set_yticks([0, 1.3])
    ax.set_yticklabels(["sent\n(each tick)", "asked\n(request time)"], fontsize=8, color=C_MUTED)
    ax.tick_params(axis="y", length=0)
    ax.set_xlabel("time (ms)", color=C_MUTED, fontsize=9)

    handles = [
        Line2D([0], [0], marker="o", linestyle="", color=C_SEARCH, markersize=7),
        Line2D([0], [0], marker="s", linestyle="", color=C_CONFIRM, markersize=7),
        Line2D([0], [0], marker="^", linestyle="", color=C_TRACK, markersize=8),
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor=C_SURFACE, markeredgecolor=C_MUTED,
               markeredgewidth=1.6, markersize=7),
        Line2D([0], [0], marker="X", linestyle="", color=C_TEXT, markersize=8),
        Line2D([0], [0], marker=".", linestyle="", color=C_MUTED, markersize=6),
    ]
    names = ["search beam sent", "confirmation beam sent", "tracking beam sent",
             "request (hollow, at time asked)", "request dropped (waited too long)", "nothing sent"]
    ax.legend(handles, names, loc="upper left", bbox_to_anchor=(0.0, -0.2), ncol=6, fontsize=8,
              frameon=False, labelcolor=C_MUTED, columnspacing=1.6, handletextpad=0.4)


def draw_sky_mixed(ax, data):
    ticks, requests, grid = data["ticks"], data["requests"], data["grid"]
    style_axis(ax, "(2) Beam directions in the antenna frame")
    ax.set_xlabel("azimuth (deg)", color=C_MUTED, fontsize=9)
    ax.set_ylabel("elevation (deg)", color=C_MUTED, fontsize=9)
    ax.set_aspect("equal", adjustable="box")

    ax.scatter([g["az"] for g in grid], [g["el"] for g in grid], s=10, color=C_DOT, zorder=1)

    searches = [tk for tk in ticks if tk["type"] == "SEARCH"]
    az = np.array([tk["az"] for tk in searches])
    el = np.array([tk["el"] for tk in searches])
    # 한 줄(고각)이 끝나 다음 줄 왼쪽으로 넘어가는 곳에서는 선을 끊는다.
    breaks = np.where(np.diff(az) < 0)[0] + 1
    ax.plot(np.insert(az, breaks, np.nan), np.insert(el, breaks, np.nan), color=C_SEARCH, linewidth=1.0,
            alpha=0.55, zorder=2)
    ax.scatter(az, el, s=26, color=C_SEARCH, edgecolor=C_SURFACE, linewidth=0.8, zorder=3)
    for tk, offset in ((searches[0], (6, 7)), (searches[-1], (8, -15))):
        ax.annotate(f"search #{tk['beam_id']}", (tk["az"], tk["el"]), xytext=offset, textcoords="offset points",
                    fontsize=8, color=C_TEXT, zorder=7)

    for r in requests:
        (hollow if r["sent"] < 0 else filled)(ax, r["az"], r["el"], r["type"], size=70, zorder=6)
        label = r["name"] + (" (dropped)" if r["sent"] < 0 else "")
        ax.annotate(label, (r["az"], r["el"]), xytext=(7, 5), textcoords="offset points",
                    fontsize=8, color=C_TEXT, zorder=7)

    xlim, ylim = sky_limits(grid)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.text(0.0, -0.17, "gray dots: 189 search cells in BeamTable.  blue: search beams sent in this run, in order.\n"
                        "filled square/triangle: request beam sent.  hollow: request dropped.",
            transform=ax.transAxes, fontsize=7.5, color=C_MUTED, va="top")


def draw_result_table(ax, data):
    requests = data["requests"]
    ax.axis("off")
    ax.set_title("(3) Result of each request", loc="left", fontsize=10, color=C_TEXT)

    rows = []
    for r in requests:
        sent = f"{r['sent']}" if r["sent"] >= 0 else "-"
        delay = f"+{r['sent'] - r['ts']}" if r["sent"] >= 0 else "-"
        result = "sent" if r["sent"] >= 0 else "dropped"
        rows.append([r["name"], TYPE_STYLE[r["type"]][2], str(r["ts"]), sent, delay, result])
    table = ax.table(cellText=rows,
                     colLabels=["req", "type", "asked (ms)", "sent (ms)", "delay (ms)", "result"],
                     colWidths=[0.1, 0.22, 0.18, 0.17, 0.17, 0.16], loc="upper center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.55)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor(C_GRID)
        cell.set_facecolor(C_SURFACE)
        cell.get_text().set_color(C_TEXT if row else C_MUTED)
        if row and rows[row - 1][5] == "dropped":
            cell.get_text().set_fontweight("bold")
    ax.text(0.0, 0.02, "A request is dropped once it has waited this long past its time:\n"
                       f"confirmation {MAX_DELAY_MS['CONFIRMATION']} ms, tracking {MAX_DELAY_MS['TRACKING']} ms.\n"
                       "Same time: confirmation first, then arrival order.",
            transform=ax.transAxes, fontsize=8, color=C_MUTED, va="bottom")


def draw_mixed(data, att):
    fig = plt.figure(figsize=(16, 10), layout="constrained")
    fig.patch.set_facecolor(C_SURFACE)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.15], width_ratios=[1.15, 1.0])
    draw_timeline(fig.add_subplot(gs[0, :]), data)
    draw_sky_mixed(fig.add_subplot(gs[1, 0]), data)
    draw_result_table(fig.add_subplot(gs[1, 1]), data)
    fig.suptitle("Beam scheduler, mixed requests: one beam per 10 ms tick, requests first, search fills the gaps   "
                 f"(attitude roll,pitch,yaw = {att[0]:g},{att[1]:g},{att[2]:g})",
                 x=0.01, ha="left", fontsize=12, color=C_TEXT)
    return fig


# ----- tracks --------------------------------------------------------------------
def draw_cycle(ax, data):
    ticks = data["ticks"]
    style_axis(ax, "(1) Search beam number over time (one sweep = beams 1 to 189)")
    ax.set_xlabel("time (ms)", color=C_MUTED, fontsize=9)
    ax.set_ylabel("search beam ID", color=C_MUTED, fontsize=9)

    searches = [tk for tk in ticks if tk["type"] == "SEARCH"]
    t_first = searches[0]["t"]

    # 요청이 없을 때의 이상적인 진행: 틱마다 한 칸씩
    t_ideal = np.arange(t_first, ticks[-1]["t"] + 1, TICK_MS)
    ideal = (t_ideal - t_first) // TICK_MS % BEAM_COUNT + 1
    ideal_plot = ideal.astype(float)
    ideal_plot[np.where(np.diff(ideal) < 0)[0] + 1] = np.nan
    ax.plot(t_ideal, ideal_plot, color=C_MUTED, linewidth=1.0, linestyle=(0, (4, 3)), zorder=2)

    # 실제 진행: 한 바퀴가 끝나는 지점에서는 선을 끊는다
    xs = np.array([tk["t"] for tk in searches], dtype=float)
    ys = np.array([tk["beam_id"] for tk in searches], dtype=float)
    ys_plot = ys.copy()
    ys_plot[np.where(np.diff(ys) < 0)[0] + 1] = np.nan
    ax.plot(xs, ys_plot, color=C_SEARCH, linewidth=1.6, zorder=3)

    # 추적 빔이 나가서 탐색이 멈춘 틱
    hold = [tk["t"] for tk in ticks if tk["type"] == "TRACKING"]
    ax.scatter(hold, [-14] * len(hold), s=30, marker="^", color=C_TRACK, edgecolor=C_SURFACE, linewidth=0.6,
               zorder=4)

    ax.set_ylim(-28, 205)
    ax.set_xlim(ticks[0]["t"] - 20, ticks[-1]["t"] + 20)
    ax.annotate("tracking beam sent (search paused)", (hold[len(hold) // 3], -14), xytext=(10, 14),
                textcoords="offset points", fontsize=8, color=C_TEXT)
    ax.annotate("search only: 1 beam per tick", (t_ideal[len(t_ideal) // 6], ideal[len(t_ideal) // 6]),
                xytext=(-96, 14), textcoords="offset points", fontsize=8, color=C_MUTED)
    ax.annotate("with 20 tracking beams per second", (xs[len(xs) // 6], ys[len(xs) // 6]),
                xytext=(26, -22), textcoords="offset points", fontsize=8, color=C_SEARCH)

    handles = [
        Line2D([0], [0], color=C_SEARCH, linewidth=1.8),
        Line2D([0], [0], color=C_MUTED, linewidth=1.0, linestyle=(0, (4, 3))),
        Line2D([0], [0], marker="^", linestyle="", color=C_TRACK, markersize=7),
    ]
    ax.legend(handles, ["search beam (actual)", "search beam (if there were no requests)", "tracking beam sent"],
              loc="upper left", bbox_to_anchor=(0.0, -0.2), ncol=3, fontsize=8, frameon=False, labelcolor=C_MUTED)


def draw_sky_cycle(fig, ax, data):
    ticks, requests, grid = data["ticks"], data["requests"], data["grid"]
    style_axis(ax, "(2) First sweep order and tracking beam positions")
    ax.set_xlabel("azimuth (deg)", color=C_MUTED, fontsize=9)
    ax.set_ylabel("elevation (deg)", color=C_MUTED, fontsize=9)
    ax.set_aspect("equal", adjustable="box")

    time_of = {}
    for tk in ticks:
        if tk["type"] == "SEARCH" and tk["beam_id"] not in time_of:
            time_of[tk["beam_id"]] = tk["t"]
    t0 = min(time_of.values())
    order = [time_of.get(g["beam_id"], np.nan) - t0 for g in grid]
    scatter = ax.scatter([g["az"] for g in grid], [g["el"] for g in grid], c=order, cmap=SEQ_BLUE, s=34,
                         edgecolor=C_SURFACE, linewidth=0.6, zorder=3)
    colorbar = fig.colorbar(scatter, ax=ax, shrink=0.8, pad=0.02)
    colorbar.set_label("time of first sweep (ms from first search beam)", fontsize=8, color=C_MUTED)
    colorbar.ax.tick_params(labelsize=8, colors=C_MUTED)
    colorbar.outline.set_visible(False)

    sent = [r for r in requests if r["sent"] >= 0]
    ax.scatter([r["az"] for r in sent], [r["el"] for r in sent], s=34, marker="^", color=C_TRACK,
               edgecolor=C_TEXT, linewidth=0.5, zorder=5)
    first = min(grid, key=lambda g: g["beam_id"])
    last = max(grid, key=lambda g: g["beam_id"])
    for g, label, offset, align in ((first, "start: beam 1", (6, 8), "left"),
                                    (last, "end: beam 189", (-6, 8), "right")):
        ax.annotate(label, (g["az"], g["el"]), xytext=offset, textcoords="offset points", fontsize=8,
                    color=C_TEXT, ha=align, zorder=7)

    xlim, ylim = sky_limits(grid)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.text(0.0, -0.17, "Dots: 189 search cells, darker = searched later.  Green triangles: tracking beams "
                        "(20 tracks, 3 rounds).", transform=ax.transAxes, fontsize=7.5, color=C_MUTED, va="top")


def draw_tiles(ax, data):
    ticks, requests = data["ticks"], data["requests"]
    ax.axis("off")
    ax.set_title("(3) Cost of tracking beams on the search sweep", loc="left", fontsize=10, color=C_TEXT)

    cycle = first_cycle_ms(ticks)
    ideal = BEAM_COUNT * TICK_MS
    sent = sum(1 for r in requests if r["sent"] >= 0)
    dropped = len(requests) - sent
    tiles = [
        (f"{cycle} ms" if cycle else "n/a", f"one search sweep with 20 tracks at 1 Hz\n(search only: {ideal} ms)"),
        (f"{(cycle / ideal - 1) * 100:.0f}%" if cycle else "n/a", "longer than a search-only sweep"),
        (f"{sent} / {len(requests)}", "tracking requests sent"),
        (f"{dropped}", "requests dropped (requests are 50 ms apart,\nso none share a tick)"),
    ]
    for i, (value, caption) in enumerate(tiles):
        y = 0.92 - 0.24 * i
        ax.text(0.02, y, value, transform=ax.transAxes, fontsize=26, color=C_TEXT, va="top", fontweight="bold")
        ax.text(0.02, y - 0.14, caption, transform=ax.transAxes, fontsize=9, color=C_MUTED, va="top")


def draw_tracks(data, att):
    fig = plt.figure(figsize=(16, 9.5), layout="constrained")
    fig.patch.set_facecolor(C_SURFACE)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.1], width_ratios=[1.35, 1.0])
    draw_cycle(fig.add_subplot(gs[0, :]), data)
    draw_sky_cycle(fig, fig.add_subplot(gs[1, 0]), data)
    draw_tiles(fig.add_subplot(gs[1, 1]), data)
    fig.suptitle("Beam scheduler, 20 tracks at 1 Hz: tracking beams delay the search sweep   "
                 f"(attitude roll,pitch,yaw = {att[0]:g},{att[1]:g},{att[2]:g})",
                 x=0.01, ha="left", fontsize=12, color=C_TEXT)
    return fig


# ----- 실행 ----------------------------------------------------------------------
def parse_att(text):
    try:
        roll, pitch, yaw = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError("--att 형식: roll,pitch,yaw (예: 10,20,30)")
    return (roll, pitch, yaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--binary", required=True, help="beam_schedule_dump 실행 파일 경로")
    ap.add_argument("--scenario", choices=["mixed", "tracks", "both"], default="both", help="그릴 시나리오")
    ap.add_argument("--att", type=parse_att, default=(0.0, 0.0, 0.0), help="자세 roll,pitch,yaw (기본 0,0,0)")
    ap.add_argument("--out", default="beam_schedule", help="저장할 PNG 경로의 접두어 (<접두어>_<시나리오>.png)")
    ap.add_argument("--verify", action="store_true", help="출력이 설계 규칙과 맞는지 검사 (어긋나면 exit 1)")
    args = ap.parse_args()

    scenarios = ["mixed", "tracks"] if args.scenario == "both" else [args.scenario]
    drawers = {"mixed": draw_mixed, "tracks": draw_tracks}

    failed = False
    for scenario in scenarios:
        data = run_dump(args.binary, scenario, args.att)
        sent = sum(1 for tk in data["ticks"] if tk["type"] != "NONE")
        dropped = sum(1 for r in data["requests"] if r["sent"] < 0)
        line = (f"[{scenario}] ticks={len(data['ticks'])} beams sent={sent} "
                f"requests={len(data['requests'])} dropped={dropped}")
        cycle = first_cycle_ms(data["ticks"])
        if cycle:
            line += f" first search sweep={cycle}ms (search only: {BEAM_COUNT * TICK_MS}ms)"
        print(line)

        if args.verify:
            failures = verify(scenario, data)
            for message in failures:
                print(f"  FAIL: {message}")
            print(f"  verify [{scenario}]: {'OK' if not failures else 'FAIL'}")
            failed |= bool(failures)

        fig = drawers[scenario](data, args.att)
        path = f"{args.out}_{scenario}.png"
        fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
        plt.close(fig)
        print(f"saved: {path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
