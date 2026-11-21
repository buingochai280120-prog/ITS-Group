#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_time_space.py - Vẽ biểu đồ không gian - thời gian (Hình 2.6 / Hình 3.3) từ quỹ đạo mô phỏng.

    python plot_time_space.py                   # tự chạy KB1 và KB2 rồi vẽ so sánh
    python plot_time_space.py --t0 900 --t1 1400
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from corridor import Config
from runner import run_simulation


def draw(ax, outdir: Path, t0: float, t1: float, title: str) -> None:
    info = json.loads((outdir / "scenario.json").read_text(encoding="utf-8"))
    C = info["config"]["cycle"]
    gs = info["green_split"]
    xs, ids, offs = info["node_x"], info["node_ids"], info["offsets"]

    # Tín hiệu trục chính tại từng nút: xanh [O, O+G), vàng [O+G, O+G+Y), còn lại đỏ
    for x, nid, o in zip(xs, ids, offs):
        ax.hlines(x, t0, t1, colors="#d9534f", linewidth=5, zorder=1)
        n0 = int((t0 - o) // C) - 1
        n1 = int((t1 - o) // C) + 1
        for n in range(n0, n1 + 1):
            s = o + n * C
            ax.hlines(x, s, s + gs["main_green"], colors="#2e9e4f", linewidth=5, zorder=2)
            ax.hlines(x, s + gs["main_green"], s + gs["main_green"] + gs["yellow"],
                      colors="#f0ad4e", linewidth=5, zorder=2)
        ax.text(t0 + 2, x + 25, f"{nid} (O={o}s)", fontsize=8, va="bottom")

    df = pd.read_csv(outdir / "trajectories.csv")
    df = df[(df.time >= t0 - 200) & (df.time <= t1)]
    for _, g in df.groupby("veh"):
        ax.plot(g["time"], g["x"], color="#1f4e9c", linewidth=0.5, alpha=0.55, zorder=3)

    ax.set_xlim(t0, t1)
    ax.set_ylim(-50, xs[-1] + 80)
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Khoảng cách dọc tuyến (m)")
    ax.set_title(title)
    ax.grid(alpha=0.25)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--t0", type=float, default=900)
    ap.add_argument("--t1", type=float, default=1300)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="time_space.png")
    a = ap.parse_args()

    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5), sharey=True)
    for ax, (mode, label) in zip(axes, [("independent", "KB1 - Đèn độc lập"),
                                        ("greenwave", "KB2 - Làn sóng xanh")]):
        out = Path("runs") / f"ts_{mode}"
        run_simulation(Config(name=label, mode=mode), out, seed=a.seed,
                       save_trajectories=True, verbose=False)
        draw(ax, out, a.t0, a.t1, label)
    fig.suptitle("Biểu đồ không gian - thời gian (xanh/vàng/đỏ: đèn trục chính; đường xanh dương: quỹ đạo xe)")
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)
    print("Đã lưu", a.out)


if __name__ == "__main__":
    main()
