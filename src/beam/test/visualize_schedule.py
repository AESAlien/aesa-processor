#!/usr/bin/env python3
"""빔 스케줄러 동작 시각화 / 검증 도구.

beam_schedule_dump 가 BeamScheduler 를 시나리오로 실행해 틱(10ms)마다 어떤 빔이 나갔는지 CSV 로 출력하면,
이 도구가 그 결과를 "처음 보는 사람도 읽을 수 있는" 그림으로 보여준다. 값은 모두 C++ BeamScheduler 의 출력이다.

만드는 그림 (시나리오마다 한 장, 한 장에는 한 가지 이야기만 담는다)
  <접두어>_mixed.png   요청이 끼어들 때
      - 위: 읽는 방법(규칙 4줄)
      - 가운데: 시간 순서대로 나간 빔. 한 칸이 10ms. 같은 종류의 탐색 빔은 한 덩어리로 묶어 번호 범위만 쓴다.
               확인/추적 빔은 칸마다 요청 이름(A, B ...)을 쓴다. 위 줄의 빈 도형은 그 요청이 들어온 시각.
      - 아래: 요청별 "언제 들어와서 언제 나갔는지, 왜 기다렸는지"를 글로 설명한 표
      (요청은 폐기되지 않는다. 밀린 요청은 "기다린 구간" 줄에 들어온 시각부터 나간 시각까지 선으로 보인다.)
  <접두어>_tracks.png  추적 빔이 많으면 탐색이 느려지는 정도
      - 위: 핵심 숫자 4개
      - 아래: 탐색 빔 번호가 시간에 따라 올라가는 그래프 (점선 = 요청이 없을 때, 실선 = 실제)
  <접두어>_mixed_map.png   (--map 을 줄 때만, mixed 시나리오에만 있다)
      - 빔이 가리키는 방향(ENU 기준, 모의기로 나가는 값)과 나간 시각. 나간 순서대로 이어서 요청 빔이 끼어드는 곳을 보인다.
        방위각은 0도 근처에서 끊기지 않도록 (-180, 180] 으로 바꿔 그린다.

--verify 를 주면 출력이 설계 규칙과 맞는지 검사하고, 어긋나면 종료 코드 1로 끝난다
  (명령 카운트 연속, 탐색 번호 순환, 요청은 원한 시각보다 일찍 나가지 않고 폐기 없이 모두 송신, 준비 전에는 송신 없음 등).

저장 위치
  기본값: <aesa-processor 의 상위 폴더>/document/testImage/<오늘 날짜>/   (예: .../LIGproject/document/testImage/2026-10-12/)
          폴더가 없으면 만든다. 같은 날 다시 실행하면 같은 이름의 파일을 덮어쓴다.
  --out-dir 폴더      저장 폴더를 직접 지정한다 (날짜 폴더를 만들지 않는다).
  --out 접두어         예전 방식. 접두어에 _<시나리오>.png 를 붙여 저장한다 (--out-dir 보다 우선).

글꼴
  한글 글꼴이 필요하다. NanumGothic / Malgun Gothic / AppleGothic / Noto Sans CJK KR 중 하나를 자동으로 찾는다.
  없으면 경고를 내고 계속하지만 글자가 네모로 보인다. (Ubuntu: sudo apt install fonts-nanum)

사용 예 (aesa-processor 루트에서 빌드한 뒤):
  cmake --build build --target beam_schedule_dump
  python3 src/beam/test/visualize_schedule.py --binary build/src/beam/beam_schedule_dump --verify
  python3 src/beam/test/visualize_schedule.py --binary build/src/beam/beam_schedule_dump \
          --scenario mixed --att 10,20,30 --map --out-dir /tmp/schedule_images
"""

import argparse
import csv
import datetime
import io
import subprocess
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")  # 화면 없이도 PNG 저장 가능
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import LinearSegmentedColormap, to_rgba
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle

# ----- 설계 값 ----------------------------------------------------------------
BEAM_COUNT = 189
TICK_MS = 10
# 표에서 "오래 기다렸다"고 강조하는 기준(ms). 보여 주기 위한 값일 뿐이며 스케줄러 규칙이 아니다(요청은 폐기되지 않는다).
LONG_WAIT_MS = 30

# ----- 색: 범주 슬롯 1~3 (검증된 팔레트, 모양/글자를 함께 써서 색에만 의존하지 않는다) ---------
C_SEARCH = "#2a78d6"
C_CONFIRM = "#eb6834"
C_TRACK = "#1baf7a"
C_TEXT = "#0b0b0b"
C_MUTED = "#52514e"
C_GRID = "#e4e3df"
C_DOT = "#c9c8c2"
C_SURFACE = "#fcfcfb"
C_WARN = "#f7dfd6"  # 오래 기다린(또는 끝까지 못 나간) 요청 행의 배경

TYPE_STYLE = {
    "SEARCH": (C_SEARCH, "o", "탐색"),
    "CONFIRMATION": (C_CONFIRM, "s", "확인"),
    "TRACKING": (C_TRACK, "^", "추적"),
}

SEQ_BLUE = LinearSegmentedColormap.from_list(
    "seq_blue", ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])

KOREAN_FONTS = ["NanumGothic", "NanumBarunGothic", "Malgun Gothic", "AppleGothic",
                "Noto Sans CJK KR", "Noto Sans KR", "Source Han Sans KR", "UnDotum"]


def setup_font():
    """한글이 나오는 글꼴을 찾아 matplotlib 기본 글꼴로 정한다. 못 찾으면 None."""
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in KOREAN_FONTS:
        if name in available:
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return name
    plt.rcParams["axes.unicode_minus"] = False
    print("경고: 한글 글꼴을 찾지 못했습니다. 글자가 네모로 보입니다. (Ubuntu: sudo apt install fonts-nanum)",
          file=sys.stderr)
    return None


def default_out_dir():
    """aesa-processor 와 같은 위치의 document/testImage/<오늘 날짜>."""
    repo_root = Path(__file__).resolve().parents[3]  # .../aesa-processor/src/beam/test/<이 파일>
    return repo_root.parent / "document" / "testImage" / datetime.date.today().isoformat()


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

    def azimuth(text):
        # C++ 는 방위각을 [0, 360) 으로 준다. 그림이 0도 근처에서 끊기지 않도록 (-180, 180] 으로 바꾼다.
        value = number(text)
        return None if value is None else (value + 180.0) % 360.0 - 180.0

    ticks = [dict(t=int(r["tick_ms"]), status=r["status"], type=r["type"],
                  beam_id=number(r["beamID"], int), count=number(r["commandCount"], int),
                  az=azimuth(r["az_deg"]), el=number(r["el_deg"]),
                  az_w=number(r["az_width_deg"]), el_w=number(r["el_width_deg"]))
             for r in rows("ticks")]
    requests = [dict(name=r["name"], type=r["type"], ts=int(r["timestamp_ms"]),
                     az=azimuth(r["az_deg"]), el=float(r["el_deg"]), sent=int(r["sent_tick_ms"]))
                for r in rows("requests")]
    grid = [dict(idx=int(r["idx"]), beam_id=int(r["beamID"]), az=azimuth(r["az_deg"]), el=float(r["el_deg"]))
            for r in rows("grid")]
    return dict(ticks=ticks, requests=requests, grid=grid)


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
        # 요청 빔의 beamID 는 요청 ID(1 이상)를 그대로 쓴다.
        if tk["type"] in ("CONFIRMATION", "TRACKING") and not (
                tk["beam_id"] >= 1 and tk["az_w"] == 3.0 and tk["el_w"] == 3.0):
            failures.append(f"request beam fields are wrong at {tk['t']}ms")
            break

    for r in requests:
        if r["sent"] < 0:
            continue
        delay = r["sent"] - r["ts"]
        if delay < 0:
            failures.append(f"request {r['name']} sent {-delay}ms before its time")

    # 요청은 폐기되지 않으므로 시나리오 안에서 모두 송신되어야 한다.
    unsent = sorted(r["name"] for r in requests if r["sent"] < 0)
    if unsent:
        failures.append(f"requests never sent {unsent}")

    return failures



# ----- 공통 그리기 도구 -------------------------------------------------------------
def style_axis(ax, title):
    ax.set_facecolor(C_SURFACE)
    ax.set_title(title, loc="left", fontsize=12, color=C_TEXT, pad=8)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(colors=C_MUTED, labelsize=9)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(C_GRID)


def hollow(ax, x, y, beam_type, size=70, zorder=5):
    color, marker, _ = TYPE_STYLE[beam_type]
    ax.scatter(x, y, s=size, marker=marker, facecolor=C_SURFACE, edgecolor=color, linewidth=1.8, zorder=zorder)


def filled(ax, x, y, beam_type, size=52, zorder=5):
    color, marker, _ = TYPE_STYLE[beam_type]
    ax.scatter(x, y, s=size, marker=marker, color=color, edgecolor=C_SURFACE, linewidth=0.8, zorder=zorder)


def sky_limits(grid):
    az = [g["az"] for g in grid]
    el = [g["el"] for g in grid]
    return (min(az) - 8, max(az) + 8), (min(el) - 8, max(el) + 8)


def tint(color, alpha=0.22):
    return to_rgba(color, alpha)


# ----- mixed: 요청이 끼어들 때 -------------------------------------------------------
def draw_rules(ax):
    ax.axis("off")
    lines = [
        "읽는 방법",
        f"1. 빔은 10ms마다 1개씩 나갑니다.  아래 그림의 한 칸이 10ms입니다.",
        "2. 확인/추적 요청이 있으면 탐색 빔보다 먼저 보냅니다.  요청이 없을 때만 탐색 빔(1번 → 189번 순서)을 보냅니다.",
        "3. 요청은 폐기되지 않습니다.  한꺼번에 몰리면 시각 순서대로 한 칸씩 보내고, 밀린 요청은 그만큼 늦게 나갑니다.",
        "4. 같은 시각에 들어온 요청은 확인 빔이 먼저, 종류가 같으면 먼저 들어온 요청이 먼저 나갑니다.",
    ]
    ax.text(0.0, 1.0, lines[0], va="top", fontsize=12, color=C_TEXT, fontweight="bold")
    ax.text(0.0, 0.72, "\n".join(lines[1:]), va="top", fontsize=11, color=C_TEXT, linespacing=1.7)


def merge_blocks(ticks, name_at):
    """연속된 탐색 틱과 연속된 '안 나간' 틱을 한 덩어리로 묶는다. 요청 빔은 틱마다 따로 둔다."""
    blocks = []
    for tk in ticks:
        kind = tk["type"]
        key = (kind, tk["status"] if kind == "NONE" else "")
        if kind in ("SEARCH", "NONE") and blocks and blocks[-1]["key"] == key:
            blocks[-1]["end"] = tk["t"]
            if kind == "SEARCH":
                blocks[-1]["last_id"] = tk["beam_id"]
            continue
        blocks.append(dict(key=key, kind=kind, status=tk["status"], start=tk["t"], end=tk["t"],
                           first_id=tk["beam_id"], last_id=tk["beam_id"], name=name_at.get(tk["t"], "")))
    return blocks


def block_label(block):
    kind = block["kind"]
    if kind == "SEARCH":
        first, last = block["first_id"], block["last_id"]
        return f"탐색\n{first}번" if first == last else f"탐색\n{first}~{last}번"
    if kind == "NONE":
        return "꺼짐" if block["status"] == "OFF" else "자세\n대기"
    return f"{TYPE_STYLE[kind][2]}\n{block['name']}"


def draw_blocks(ax, data):
    ticks, requests = data["ticks"], data["requests"]
    style_axis(ax, "시간 순서대로 나간 빔  (한 칸 = 10ms)")
    ax.grid(False)
    name_at = {r["sent"]: r["name"] for r in requests if r["sent"] >= 0}

    # 아래 줄: 실제로 나간 빔
    for block in merge_blocks(ticks, name_at):
        left = block["start"] - TICK_MS / 2
        width = block["end"] - block["start"] + TICK_MS
        if block["kind"] == "NONE":
            face, edge = C_GRID, C_DOT
        else:
            edge = TYPE_STYLE[block["kind"]][0]
            face = tint(edge)
        ax.add_patch(Rectangle((left, 0.0), width, 1.0, facecolor=face, edgecolor=edge, linewidth=1.4, zorder=3))
        ax.text(left + width / 2, 0.5, block_label(block), ha="center", va="center", fontsize=9,
                color=C_MUTED if block["kind"] == "NONE" else C_TEXT, linespacing=1.15, zorder=4)

    # 위 줄: 요청이 들어온 시각. 같은 시각 근처에 몰린 요청은 겹치지 않게 위로 쌓는다.
    ordered = sorted(requests, key=lambda item: item["ts"])
    levels = {r["name"]: sum(1 for q in ordered[:i] if r["ts"] - q["ts"] < TICK_MS)
              for i, r in enumerate(ordered)}
    for r in requests:
        y = 1.55 + 0.42 * levels[r["name"]]
        hollow(ax, r["ts"], y, r["type"])
        ax.annotate(r["name"], (r["ts"], y), xytext=(8, -1), textcoords="offset points",
                    fontsize=10, color=C_TEXT, va="center", zorder=6)

    # 맨 아래 줄: 요청이 들어온 뒤 나갈 때까지 기다린 구간. 겹치는 구간은 아래로 쌓는다.
    waits = sorted((r for r in requests if r["sent"] > r["ts"]), key=lambda item: item["ts"])
    level_end, wait_level = [], {}
    for r in waits:
        level = next((i for i, end in enumerate(level_end) if end + 22 <= r["ts"]), len(level_end))
        if level == len(level_end):
            level_end.append(r["sent"])
        else:
            level_end[level] = r["sent"]
        wait_level[r["name"]] = level
    for r in waits:
        y = -0.35 - 0.3 * wait_level[r["name"]]
        color = TYPE_STYLE[r["type"]][0]
        ax.plot([r["ts"], r["sent"]], [y, y], color=color, linewidth=3.0, solid_capstyle="butt", zorder=5)
        filled(ax, r["sent"], y, r["type"], size=40, zorder=6)
        ax.annotate(f"{r['name']} {r['sent'] - r['ts']}ms", (r["sent"], y), xytext=(8, 0), textcoords="offset points",
                    fontsize=9, color=C_TEXT, va="center", zorder=6)
    wait_rows = max(len(level_end), 1)
    wait_bottom = -0.35 - 0.3 * (wait_rows - 1) - 0.3

    ax.set_ylim(wait_bottom, 1.55 + 0.42 * (max(levels.values()) + 1))
    ax.set_xlim(ticks[0]["t"] - 8, ticks[-1]["t"] + 8)
    ax.set_xticks(range(0, ticks[-1]["t"] + 1, 50))
    ax.set_yticks([-0.35 - 0.3 * (wait_rows - 1) / 2, 0.5, 1.55 + 0.42 * max(levels.values()) / 2])
    ax.set_yticklabels(["기다린 구간", "실제로\n나간 빔", "요청이\n들어온 시각"], fontsize=10, color=C_MUTED)
    ax.tick_params(axis="y", length=0)
    ax.set_xlabel("시간 (ms)", color=C_MUTED, fontsize=10)

    handles = [
        Patch(facecolor=tint(C_SEARCH), edgecolor=C_SEARCH, linewidth=1.4),
        Patch(facecolor=tint(C_CONFIRM), edgecolor=C_CONFIRM, linewidth=1.4),
        Patch(facecolor=tint(C_TRACK), edgecolor=C_TRACK, linewidth=1.4),
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor=C_SURFACE, markeredgecolor=C_MUTED,
               markeredgewidth=1.6, markersize=8),
        Line2D([0], [0], color=C_MUTED, linewidth=3.0),
        Patch(facecolor=C_GRID, edgecolor=C_DOT, linewidth=1.4),
    ]
    names = ["탐색 빔이 나감", "확인 빔이 나감 (칸 안의 글자 = 요청 이름)", "추적 빔이 나감 (칸 안의 글자 = 요청 이름)",
             "요청이 들어온 시각 (빈 도형, 모양은 종류별)", "들어온 뒤 나갈 때까지 기다린 구간 (끝의 숫자 = 기다린 시간)",
             "아직 보내지 않는 구간"]
    ax.legend(handles, names, loc="upper left", bbox_to_anchor=(0.0, -0.22), ncol=3, fontsize=9.5,
              frameon=False, labelcolor=C_MUTED, columnspacing=1.8, handletextpad=0.5)


def explain_request(r, data):
    """요청 한 건이 어떻게 처리됐는지 평이한 문장으로 만든다."""
    ticks, requests = data["ticks"], data["requests"]
    name_at = {q["sent"]: q["name"] for q in requests if q["sent"] >= 0}
    end = r["sent"] if r["sent"] >= 0 else ticks[-1]["t"] + TICK_MS
    ahead = [name_at[tk["t"]] for tk in ticks if r["ts"] <= tk["t"] < end and tk["t"] in name_at]

    if r["sent"] < 0:
        text = f"시나리오가 끝날 때까지({ticks[-1]['t']}ms) 나가지 못함"
        if ahead:
            text += f"\n그 사이 먼저 나간 요청: {', '.join(ahead)}"
        return text

    delay = r["sent"] - r["ts"]
    if delay == 0:
        text = "들어오자마자 바로 나감"
    elif ahead:
        text = f"먼저 나간 요청({', '.join(ahead)}) 때문에 {delay}ms 기다린 뒤 나감"
    else:
        text = f"다음 송신 시점까지 {delay}ms 기다린 뒤 나감"
    later_same_time = [q["name"] for q in requests
                       if q is not r and q["ts"] == r["ts"] and q["type"] != "CONFIRMATION"
                       and (q["sent"] < 0 or q["sent"] > r["sent"])]
    if r["type"] == "CONFIRMATION" and later_same_time:
        text += f"\n확인 빔이라 같은 시각의 추적 요청({', '.join(later_same_time)})보다 먼저 나감"
    return text


def draw_request_table(ax, data):
    requests = data["requests"]
    ax.set_facecolor(C_SURFACE)
    ax.set_title("요청별로 무슨 일이 있었나", loc="left", fontsize=12, color=C_TEXT, pad=8)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])

    columns = [("요청", 0.01), ("종류", 0.075), ("들어온 시각", 0.165), ("나간 시각", 0.27),
               ("기다린 시간", 0.365), ("처리 결과와 이유", 0.47)]
    rows = len(requests)
    ax.set_xlim(0, 1)
    ax.set_ylim(rows + 1, 0)
    for label, x in columns:
        ax.text(x, 0.5, label, fontsize=10, color=C_MUTED, va="center")
    ax.plot([0, 1], [1, 1], color=C_DOT, linewidth=1.0)

    for i, r in enumerate(requests, start=1):
        waited_long = r["sent"] < 0 or r["sent"] - r["ts"] >= LONG_WAIT_MS
        if waited_long:
            ax.add_patch(Rectangle((0, i), 1, 1, facecolor=C_WARN, edgecolor="none", zorder=0))
        ax.plot([0, 1], [i + 1, i + 1], color=C_GRID, linewidth=0.8)
        color, marker, kind_name = TYPE_STYLE[r["type"]]
        ax.scatter([0.075], [i + 0.5], s=40, marker=marker, color=color, zorder=3)
        values = [
            (r["name"], 0.01, "bold" if waited_long else "normal"),
            (kind_name, 0.09, "normal"),
            (f"{r['ts']}ms", 0.165, "normal"),
            ("-" if r["sent"] < 0 else f"{r['sent']}ms", 0.27, "normal"),
            ("-" if r["sent"] < 0 else f"{r['sent'] - r['ts']}ms", 0.365, "normal"),
            (explain_request(r, data), 0.47, "bold" if waited_long else "normal"),
        ]
        for text, x, weight in values:
            ax.text(x, i + 0.5, text, fontsize=10.5, color=C_TEXT, va="center", fontweight=weight, linespacing=1.35)


def draw_mixed(data, att):
    rows = len(data["requests"])
    fig = plt.figure(figsize=(16, 7.2 + 0.52 * rows), layout="constrained")
    fig.patch.set_facecolor(C_SURFACE)
    gs = fig.add_gridspec(3, 1, height_ratios=[1.0, 2.1, 0.52 * rows + 0.6])
    draw_rules(fig.add_subplot(gs[0]))
    draw_blocks(fig.add_subplot(gs[1]), data)
    draw_request_table(fig.add_subplot(gs[2]), data)
    fig.suptitle("빔 스케줄러 동작 ①  확인/추적 요청이 끼어들 때", x=0.01, ha="left", fontsize=15, color=C_TEXT)
    return fig


def search_runs(ticks):
    """연속해서 나간 탐색 빔 묶음. 요청 빔이 끼어들면 새 묶음이 시작된다."""
    runs, current = [], []
    for tk in ticks:
        if tk["type"] == "SEARCH":
            current.append(tk)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return runs


def draw_sky_mixed(data, att):
    ticks, requests, grid = data["ticks"], data["requests"], data["grid"]
    fig, ax = plt.subplots(figsize=(11, 7.8), layout="constrained")
    fig.patch.set_facecolor(C_SURFACE)
    style_axis(ax, "빔이 가리키는 방향과 나간 시각 (ENU 기준)")
    ax.set_xlabel("방위각 (deg)", color=C_MUTED, fontsize=10)
    ax.set_ylabel("고각 (deg)", color=C_MUTED, fontsize=10)
    ax.set_aspect("equal", adjustable="box")

    ax.scatter([g["az"] for g in grid], [g["el"] for g in grid], s=10, color=C_DOT, zorder=1)

    # 나간 순서대로 이어 그린다. 탐색끼리는 파란 실선, 요청 빔이 끼어든 구간은 가는 회색 점선이고 끝에 화살표를 붙인다.
    sent = [tk for tk in ticks if tk["type"] != "NONE"]
    for previous, current in zip(sent, sent[1:]):
        xs, ys = [previous["az"], current["az"]], [previous["el"], current["el"]]
        if previous["type"] == "SEARCH" and current["type"] == "SEARCH":
            if current["az"] > previous["az"]:  # 줄 바꿈(다음 고각 줄의 왼쪽으로 이동)에서는 선을 끊는다
                ax.plot(xs, ys, color=C_SEARCH, linewidth=1.0, alpha=0.55, zorder=2)
        else:
            ax.plot(xs, ys, color=C_MUTED, linewidth=0.6, linestyle=(0, (2, 2.5)), alpha=0.85, zorder=2)
            # 도착하는 빔 쪽 끝에 화살촉만 붙여서 어느 쪽으로 이어지는지 보이게 한다(마커에 가려지지 않게 조금 앞에서 끝낸다).
            tail = (xs[1] - 0.08 * (xs[1] - xs[0]), ys[1] - 0.08 * (ys[1] - ys[0]))
            ax.annotate("", xy=(xs[1], ys[1]), xytext=tail,
                        arrowprops=dict(arrowstyle="-|>", color=C_MUTED, linewidth=0.6, alpha=0.85,
                                        mutation_scale=9, shrinkA=0, shrinkB=6), zorder=2)

    searches = [tk for tk in ticks if tk["type"] == "SEARCH"]
    scatter = ax.scatter([tk["az"] for tk in searches], [tk["el"] for tk in searches], c=[tk["t"] for tk in searches],
                         cmap=SEQ_BLUE, s=38, edgecolor=C_SURFACE, linewidth=0.6, zorder=3,
                         vmin=ticks[0]["t"], vmax=ticks[-1]["t"])
    colorbar = fig.colorbar(scatter, ax=ax, shrink=0.8, pad=0.02)
    colorbar.set_label("탐색 빔이 나간 시각 (ms)", fontsize=9, color=C_MUTED)
    colorbar.ax.tick_params(labelsize=9, colors=C_MUTED)
    colorbar.outline.set_visible(False)

    # 탐색 묶음마다 나간 시각의 범위를 첫 빔 아래에 쓴다
    for run in search_runs(ticks):
        first, last = run[0], run[-1]
        text = f"{first['t']}ms" if first is last else f"{first['t']}~{last['t']}ms"
        ax.annotate(text, (first["az"], first["el"]), xytext=(0, -13), textcoords="offset points",
                    fontsize=8.5, color=C_MUTED, ha="center", va="top", zorder=7)

    # 가까이 있는 요청끼리 글자가 겹치지 않도록 이웃한 요청은 위/아래로 번갈아 쓴다.
    placed = []
    for r in requests:
        label = f"{r['name']}  {r['sent']}ms"
        if r["sent"] < 0:
            hollow(ax, r["az"], r["el"], r["type"], size=90, zorder=6)
            label = f"{r['name']}  (미송신)"
        else:
            filled(ax, r["az"], r["el"], r["type"], size=90, zorder=6)
            if r["sent"] > r["ts"]:
                label += f"\n(+{r['sent'] - r['ts']}ms 대기)"
        near = [slot for (az, el, slot) in placed if abs(az - r["az"]) <= 8 and abs(el - r["el"]) <= 3]
        slot = (near[-1] + 1) % 2 if near else 0
        placed.append((r["az"], r["el"], slot))
        if slot == 0:
            ax.annotate(label, (r["az"], r["el"]), xytext=(8, 8), textcoords="offset points",
                        fontsize=9, color=C_TEXT, va="bottom", linespacing=1.2, zorder=7)
        else:
            ax.annotate(label, (r["az"], r["el"]), xytext=(8, -8), textcoords="offset points",
                        fontsize=9, color=C_TEXT, va="top", linespacing=1.2, zorder=7)

    xlim, ylim = sky_limits(grid)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.text(0.0, -0.1, "회색 점: SearchGrid의 탐색 빔 189개   파란 점: 나간 탐색 빔 (색 = 나간 시각, 아래 숫자 = 그 묶음이 나간 시각)\n"
                       "사각형/삼각형: 나간 확인/추적 빔 (글자 = 요청 이름과 나간 시각)\n"
                       "파란 실선: 탐색이 이어지는 순서   회색 점선 화살표: 요청 빔이 끼어들어 갔다가 탐색으로 돌아오는 순서(화살표 방향 = 나간 순서)\n"
                       f"안테나 자세 roll,pitch,yaw = {att[0]:g},{att[1]:g},{att[2]:g}",
            transform=ax.transAxes, fontsize=9, color=C_MUTED, va="top")
    return fig


# ----- tracks: 추적 빔이 탐색을 느리게 하는 정도 ---------------------------------------
def draw_tiles(ax, data):
    ticks, requests = data["ticks"], data["requests"]
    ax.axis("off")

    cycle = first_cycle_ms(ticks)
    ideal = BEAM_COUNT * TICK_MS
    sent = sum(1 for r in requests if r["sent"] >= 0)
    longest_wait = max((r["sent"] - r["ts"] for r in requests if r["sent"] >= 0), default=0)
    hold = 0
    if cycle:
        t_start = next(tk["t"] for tk in ticks if tk["type"] == "SEARCH" and tk["beam_id"] == 1)
        hold = sum(1 for tk in ticks if tk["type"] == "TRACKING" and t_start <= tk["t"] < t_start + cycle)
    tiles = [
        (f"{cycle}ms" if cycle else "해당 없음",
         f"탐색 한 바퀴(1번 → 189번)에\n걸린 시간  (추적이 없으면 {ideal}ms)"),
        (f"+{(cycle / ideal - 1) * 100:.0f}%" if cycle else "해당 없음",
         f"추적 빔 {hold}개가 먼저 나가서\n탐색이 {hold * TICK_MS}ms 멈춤" if cycle else ""),
        (f"{sent} / {len(requests)}", "나간 추적 요청 수\n(전체 요청 중)"),
        (f"{longest_wait}ms", "요청이 나가기까지 가장 오래\n기다린 시간 (요청이 50ms 간격이라 겹치지 않음)"),
    ]
    for i, (value, caption) in enumerate(tiles):
        x = 0.01 + 0.25 * i
        ax.text(x, 0.95, value, transform=ax.transAxes, fontsize=32, color=C_TEXT, va="top", fontweight="bold")
        ax.text(x, 0.45, caption, transform=ax.transAxes, fontsize=11, color=C_MUTED, va="top", linespacing=1.5)


def draw_cycle(ax, data):
    ticks = data["ticks"]
    style_axis(ax, "탐색 빔 번호가 시간에 따라 올라가는 모습  (1번에서 189번까지가 한 바퀴)")
    ax.set_xlabel("시간 (ms)", color=C_MUTED, fontsize=10)
    ax.set_ylabel("탐색 빔 번호", color=C_MUTED, fontsize=10)

    searches = [tk for tk in ticks if tk["type"] == "SEARCH"]
    t_first = searches[0]["t"]

    # 요청이 없을 때의 이상적인 진행: 틱마다 한 칸씩
    t_ideal = np.arange(t_first, ticks[-1]["t"] + 1, TICK_MS)
    ideal = (t_ideal - t_first) // TICK_MS % BEAM_COUNT + 1
    ideal_plot = ideal.astype(float)
    ideal_plot[np.where(np.diff(ideal) < 0)[0] + 1] = np.nan
    ax.plot(t_ideal, ideal_plot, color=C_MUTED, linewidth=1.3, linestyle=(0, (4, 3)), zorder=2)

    # 실제 진행: 한 바퀴가 끝나는 지점에서는 선을 끊는다
    xs = np.array([tk["t"] for tk in searches], dtype=float)
    ys = np.array([tk["beam_id"] for tk in searches], dtype=float)
    ys_plot = ys.copy()
    ys_plot[np.where(np.diff(ys) < 0)[0] + 1] = np.nan
    ax.plot(xs, ys_plot, color=C_SEARCH, linewidth=2.2, zorder=3)

    # 추적 빔이 나가서 탐색이 멈춘 틱
    hold = [tk["t"] for tk in ticks if tk["type"] == "TRACKING"]
    ax.scatter(hold, [-14] * len(hold), s=34, marker="^", color=C_TRACK, edgecolor=C_SURFACE, linewidth=0.6,
               zorder=4)

    # 189번에 도착하는 시각의 차이
    t_ideal_end = t_first + (BEAM_COUNT - 1) * TICK_MS
    t_real_end = next((tk["t"] for tk in searches if tk["beam_id"] == BEAM_COUNT), None)
    if t_real_end is not None:
        ax.annotate("", xy=(t_real_end, BEAM_COUNT + 6), xytext=(t_ideal_end, BEAM_COUNT + 6),
                    arrowprops=dict(arrowstyle="<->", color=C_TEXT, linewidth=1.3), zorder=5)
        ax.text((t_ideal_end + t_real_end) / 2, BEAM_COUNT + 12, f"{t_real_end - t_ideal_end}ms 늦게 도착",
                ha="center", va="bottom", fontsize=10.5, color=C_TEXT)

    ax.set_ylim(-28, 215)
    ax.set_xlim(ticks[0]["t"] - 20, ticks[-1]["t"] + 20)

    handles = [
        Line2D([0], [0], color=C_SEARCH, linewidth=2.4),
        Line2D([0], [0], color=C_MUTED, linewidth=1.3, linestyle=(0, (4, 3))),
        Line2D([0], [0], marker="^", linestyle="", color=C_TRACK, markersize=8),
    ]
    ax.legend(handles, ["실제 탐색 진행 (추적 빔 때문에 중간중간 멈춤)", "추적 요청이 없었다면의 진행 (10ms마다 1칸)",
                        "추적 빔이 나간 시각 (이때 탐색은 멈춤)"],
              loc="upper left", bbox_to_anchor=(0.0, -0.18), ncol=3, fontsize=9.5, frameon=False,
              labelcolor=C_MUTED, columnspacing=1.8)


def draw_tracks(data, att):
    fig = plt.figure(figsize=(16, 8.2), layout="constrained")
    fig.patch.set_facecolor(C_SURFACE)
    gs = fig.add_gridspec(2, 1, height_ratios=[0.8, 2.4])
    draw_tiles(fig.add_subplot(gs[0]), data)
    draw_cycle(fig.add_subplot(gs[1]), data)
    fig.suptitle("빔 스케줄러 동작 ②  추적 20개를 초당 1번씩 요청하면 탐색이 얼마나 느려지나",
                 x=0.01, ha="left", fontsize=15, color=C_TEXT)
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
    ap.add_argument("--att", type=parse_att, default=(0.0, 0.0, 0.0),
                    help="자세 roll,pitch,yaw (기본 0,0,0). 빔 방향 그림(--map)에만 영향을 준다")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="저장 폴더 (기본: <aesa-processor 상위>/document/testImage/<오늘 날짜>)")
    ap.add_argument("--out", default=None, help="예전 방식: 저장할 PNG 경로의 접두어 (<접두어>_<시나리오>.png)")
    ap.add_argument("--map", action="store_true", help="빔이 가리키는 방향 그림(mixed 시나리오만, <이름>_map.png)도 함께 저장")
    ap.add_argument("--verify", action="store_true", help="출력이 설계 규칙과 맞는지 검사 (어긋나면 exit 1)")
    args = ap.parse_args()

    setup_font()
    if args.out is not None:
        prefix = Path(args.out)
    else:
        prefix = (args.out_dir or default_out_dir()) / "beam_schedule"
    prefix.parent.mkdir(parents=True, exist_ok=True)

    scenarios = ["mixed", "tracks"] if args.scenario == "both" else [args.scenario]
    drawers = {"mixed": draw_mixed, "tracks": draw_tracks}
    map_drawers = {"mixed": draw_sky_mixed}  # tracks 는 방향도를 만들지 않는다

    failed = False
    for scenario in scenarios:
        data = run_dump(args.binary, scenario, args.att)
        sent = sum(1 for tk in data["ticks"] if tk["type"] != "NONE")
        unsent = sum(1 for r in data["requests"] if r["sent"] < 0)
        line = (f"[{scenario}] ticks={len(data['ticks'])} beams sent={sent} "
                f"requests={len(data['requests'])} unsent={unsent}")
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

        outputs = [(f"{prefix}_{scenario}.png", drawers[scenario](data, args.att))]
        if args.map and scenario in map_drawers:
            outputs.append((f"{prefix}_{scenario}_map.png", map_drawers[scenario](data, args.att)))
        for path, fig in outputs:
            fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
            plt.close(fig)
            print(f"saved: {path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
