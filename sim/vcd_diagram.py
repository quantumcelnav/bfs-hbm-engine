#!/usr/bin/env python3
"""
Parse bfs_waves.vcd and render a publication-quality timing diagram.

Symbol map (from VCD header, bfs_tb scope):
  A → clk          B → rst_n       D → start       2 → done
  3 → arvalid      4 → arready     * → rvalid
  + → rready       , → rlast       0 → out_valid
  ) → visited_count[31:0]          F → total_cycles[31:0]
  P → rd_req_valid  b → bm_op_valid  c → bm_op_set

Timescale: 1 ps. Clock period: 2000 ps (500 MHz).
Total simulation: 0 – 2,102,000 ps (1051 cycles).
BFS completes at 514 cycles = 1,028,000 ps per RTL.
"""

from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ── Symbol → display name ─────────────────────────────────────────────────
SYM_MAP = {
    "A": "clk",
    "B": "rst_n",
    "D": "start",
    "2": "done",
    "3": "arvalid",
    "4": "arready",
    "*": "rvalid",
    "+": "rready",
    ",": "rlast",
    "0": "out_valid",
    ")": "visited_count",
    "P": "rd_req_valid",
    "b": "bm_op_valid",
    "c": "bm_op_set",
    "F": "total_cycles",
}

CLK_PERIOD_PS = 2000   # 500 MHz
PLOT_SIGNALS = [
    "clk", "rst_n", "start", "done",
    "arvalid", "arready",
    "rvalid", "rready", "rlast",
    "rd_req_valid",
    "bm_op_valid", "bm_op_set",
    "out_valid",
]
ANALOG_SIGNALS = ["visited_count"]   # plotted as step ramp


def parse_vcd(path: Path) -> dict[str, list[tuple[int, int | None]]]:
    """
    Return {signal_name: [(time_ps, int_value)]} for signals in SYM_MAP.
    """
    waves: dict[str, list[tuple[int, int | None]]] = {n: [] for n in SYM_MAP.values()}
    current_time = 0

    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("$"):
                continue
            if line.startswith("#"):
                try:
                    current_time = int(line[1:])
                except ValueError:
                    pass
                continue
            # Scalar: 0A, 1B, xC …
            if len(line) >= 2 and line[0] in "01xzX" and not line.startswith("b"):
                val_ch, sym = line[0], line[1:]
                if sym in SYM_MAP:
                    v = {"0": 0, "1": 1}.get(val_ch)
                    waves[SYM_MAP[sym]].append((current_time, v))
                continue
            # Vector: bXXX sym
            if line.startswith("b"):
                parts = line.split(None, 1)
                if len(parts) == 2:
                    bits, sym = parts[0][1:], parts[1].strip()
                    if sym in SYM_MAP:
                        try:
                            v = int(bits, 2)
                        except ValueError:
                            v = None
                        waves[SYM_MAP[sym]].append((current_time, v))

    return waves


def step_xy(events: list[tuple[int, int | None]], end_t: int):
    """Expand event list into step-function x/y for plotting."""
    if not events:
        return [], []
    ts = [t for t, _ in events]
    vs = [v if v is not None else 0 for _, v in events]
    xs, ys = [], []
    for i, (t, v) in enumerate(zip(ts, vs)):
        xs.append(t)
        ys.append(v)
        if i + 1 < len(ts):
            xs.append(ts[i + 1])
            ys.append(v)
    xs.append(end_t)
    ys.append(ys[-1] if ys else 0)
    return xs, ys


COLORS = {
    "clk":          "#58a6ff",
    "rst_n":        "#f0883e",
    "start":        "#3fb950",
    "done":         "#ff7b72",
    "arvalid":      "#d2a8ff",
    "arready":      "#79c0ff",
    "rvalid":       "#ffa657",
    "rready":       "#56d364",
    "rlast":        "#e3b341",
    "rd_req_valid": "#bc8cff",
    "bm_op_valid":  "#ff79c6",
    "bm_op_set":    "#50fa7b",
    "out_valid":    "#8be9fd",
    "visited_count":"#3fb950",
}

BG  = "#0d1117"
ROW = "#161b22"
TXT = "#8b949e"


def render(vcd_path: Path, out_path: Path) -> None:
    waves = parse_vcd(vcd_path)

    # ── Determine end time ────────────────────────────────────────────────
    done_events = [(t, v) for t, v in waves["done"] if v == 1]
    t_done = done_events[0][0] if done_events else 1_100_000
    end_t = t_done + CLK_PERIOD_PS * 10

    # ── Find rst_n deassertion (simulation start) ─────────────────────────
    rst_rise = next((t for t, v in waves["rst_n"] if v == 1), 0)

    n_sig = len(PLOT_SIGNALS)
    n_rows = n_sig + 1   # +1 for visited_count

    fig, axes = plt.subplots(
        n_rows, 1, figsize=(18, n_rows * 0.52 + 1.6),
        gridspec_kw={"height_ratios": [1] * n_sig + [1.8]},
    )
    fig.patch.set_facecolor(BG)

    for i, sname in enumerate(PLOT_SIGNALS):
        ax = axes[i]
        ax.set_facecolor(ROW)
        ax.set_xlim(0, end_t)
        ax.set_ylim(-0.1, 1.1)
        ax.set_yticks([])
        ax.spines[:].set_visible(False)
        ax.set_xticks([])
        ax.set_ylabel(sname, rotation=0, ha="right", va="center",
                      color=TXT, fontsize=7.5, labelpad=6,
                      fontfamily="monospace")

        evs = [(t, v) for t, v in waves[sname] if t <= end_t]
        if not evs:
            ax.text(0.5, 0.5, "—", transform=ax.transAxes,
                    color=TXT, ha="center", va="center", fontsize=8)
            continue

        col = COLORS.get(sname, "#c9d1d9")
        xs, ys = step_xy(evs, end_t)

        ax.fill_between(xs, ys, alpha=0.20, color=col, step=None)
        ax.step(xs, ys, where="post", color=col, linewidth=1.3)

        # Mark every rising edge with a faint vertical
        prev_v = 0
        for t, v in evs:
            if v == 1 and prev_v == 0 and sname not in ("clk", "rst_n"):
                ax.axvline(t, color=col, alpha=0.20, linewidth=0.4, linestyle=":")
            prev_v = v if v is not None else prev_v

        # Annotate done assertion time
        if sname == "done" and done_events:
            ax.text(t_done, 1.0, f"  BFS\n  done\n  {t_done//1000} ns",
                    color=COLORS["done"], fontsize=6, va="top")

    # ── visited_count ramp ────────────────────────────────────────────────
    ax_vc = axes[-1]
    ax_vc.set_facecolor(ROW)
    ax_vc.set_xlim(0, end_t)
    ax_vc.spines[:].set_visible(False)
    ax_vc.tick_params(colors=TXT, labelsize=7)
    ax_vc.set_ylabel("visited\ncount", rotation=0, ha="right", va="center",
                      color=TXT, fontsize=7.5, labelpad=6)

    vc_evs = [(t, v) for t, v in waves["visited_count"] if t <= end_t and v is not None]
    if vc_evs:
        xs2, ys2 = step_xy(vc_evs, end_t)
        max_v = max(v for _, v in vc_evs)
        ax_vc.set_ylim(0, max_v * 1.18)
        col_vc = COLORS["visited_count"]
        ax_vc.fill_between(xs2, ys2, alpha=0.30, color=col_vc, step=None)
        ax_vc.step(xs2, ys2, where="post", color=col_vc, linewidth=1.4)
        ax_vc.axhline(max_v, color=col_vc, linewidth=0.6, linestyle="--", alpha=0.6)
        ax_vc.text(end_t * 0.98, max_v * 1.05,
                   f"  final={max_v} vertices", color=col_vc, fontsize=7, ha="right")
        # Annotate each jump
        prev_vc = 0
        for t, v in vc_evs:
            if v != prev_vc and v > prev_vc:
                ax_vc.annotate(
                    f"+{v - prev_vc}",
                    xy=(t, v), xytext=(t + CLK_PERIOD_PS * 3, v + 0.3),
                    color=col_vc, fontsize=5.5, alpha=0.8,
                )
            prev_vc = v

    # ── X-axis ticks in ns ───────────────────────────────────────────────
    tick_step = 50_000   # every 50 ns
    x_ticks = np.arange(0, end_t + tick_step, tick_step)
    x_ticks = x_ticks[x_ticks <= end_t]
    ax_vc.set_xticks(x_ticks)
    ax_vc.set_xticklabels([f"{t//1000}" for t in x_ticks], color=TXT, fontsize=7)
    ax_vc.set_xlabel("time (ns)  ·  clock = 500 MHz (2 ns period)", color=TXT, fontsize=8)

    # ── Global vertical reference lines ──────────────────────────────────
    refs = [
        (rst_rise, "#f0883e", "rst_n↑"),
    ]
    start_events = [(t, v) for t, v in waves["start"] if v == 1]
    if start_events:
        refs.append((start_events[0][0], "#3fb950", "start↑"))
    if done_events:
        refs.append((t_done, "#ff7b72", "done↑"))

    for t_ref, col_ref, lbl in refs:
        for ax in axes:
            ax.axvline(t_ref, color=col_ref, linewidth=0.7, linestyle="--", alpha=0.35)

    # ── Annotations: BFS phases ───────────────────────────────────────────
    # Phase label strip on first axis
    ax0 = axes[0]
    if start_events and done_events:
        t_s = start_events[0][0]
        t_d = done_events[0][0]
        ax0.annotate("", xy=(t_d, 1.08), xytext=(t_s, 1.08),
                     arrowprops=dict(arrowstyle="<->", color="#8b949e", lw=0.8))
        ax0.text((t_s + t_d) / 2, 1.12, f"BFS traversal ({(t_d - t_s)//1000} ns / 514 cycles)",
                 color="#8b949e", fontsize=7, ha="center", va="bottom")

    # ── Title ─────────────────────────────────────────────────────────────
    fig.suptitle(
        "BFS-HBM Engine — AXI4 Bus Waveform  ·  8-vertex reference graph  ·  514 cycles @ 500 MHz",
        color="#f0f6fc", fontsize=11, y=1.00, fontweight="semibold",
    )
    fig.text(0.5, 0.985,
             "Signals: AXI4 read channel (arvalid/arready/rvalid/rlast), bitmap R-M-W (bm_op), "
             "frontier output (out_valid), visited vertex count",
             color=TXT, fontsize=7.5, ha="center", va="top")

    plt.tight_layout(rect=[0.08, 0, 1, 0.98])
    out_path.parent.mkdir(exist_ok=True)
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"Saved → {out_path}")


if __name__ == "__main__":
    vcd = Path(__file__).parent / "bfs_waves.vcd"
    out = Path(__file__).parent / "figures" / "bfs_waveform.png"
    render(vcd, out)
