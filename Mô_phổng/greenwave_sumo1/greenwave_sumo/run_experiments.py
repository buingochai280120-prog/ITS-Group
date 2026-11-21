#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_experiments.py - Chạy toàn bộ kịch bản KB1-KB4 (Mục 4.1) và xuất bảng so sánh (Bảng 4.3).

    python run_experiments.py                 # đủ KB1-KB4, 3 seed mỗi trường hợp
    python run_experiments.py --seeds 5       # 5 seed cho kết quả ổn định hơn
    python run_experiments.py --only KB1 KB2  # chỉ chạy một số nhóm
    python run_experiments.py --jobs 4        # chạy song song 4 tiến trình

Mỗi trường hợp được chạy ở CẢ HAI chế độ: "independent" (đối chứng) và "greenwave" (phối hợp) để thấy
mức cải thiện. Mọi cặp đối chứng/phối hợp dùng cùng seed => cùng lịch phát xe, cùng nhiễu Krauss ban đầu.
"""
from __future__ import annotations

import argparse
import shutil
import statistics as st
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Dict, List

import pandas as pd

from corridor import Config
from runner import run_simulation

# ---- Định nghĩa các nhóm kịch bản --------------------------------------------------
def make_cases() -> List[Dict]:
    """Trường hợp cơ sở (1200 xe/h, C = 90 s, 50 km/h) vừa là KB1/KB2, vừa là điểm giữa của KB3, KB4-A, KB4-B,
    nên chỉ chạy một lần."""
    cases = [{"id": "base", "group": "KB1-KB2",
              "case": "KB1 vs KB2 - cơ sở (= KB3 Q=1200, KB4-A C=90, KB4-B V=50)", "kw": {}}]
    for q in (600, 900, 1500):                                           # KB3
        cases.append({"id": f"kb3_q{q}", "group": "KB3", "case": f"KB3 - Q = {q} xe/h",
                      "kw": {"flow_vph": float(q)}})
    for c in (80, 100):                                                  # KB4-A
        cases.append({"id": f"kb4a_c{c}", "group": "KB4A", "case": f"KB4-A - C = {c} s",
                      "kw": {"cycle": c}})
    for v in (40, 60):                                                   # KB4-B
        cases.append({"id": f"kb4b_v{v}", "group": "KB4B", "case": f"KB4-B - V = {v} km/h",
                      "kw": {"speed_kmh": float(v)}})
    return cases


def _job(args):
    case, mode, seed, root = args
    out = Path(root) / "runs" / f"{case['id']}_{mode}_s{seed}"      # tên ASCII: netconvert tách đường dẫn theo dấu phẩy
    cfg = Config(name=case["case"], mode=mode, **case["kw"])
    res = run_simulation(cfg, out, seed=seed, verbose=False)
    res["group"], res["case"] = case["group"], case["case"]
    shutil.rmtree(out, ignore_errors=True)         # xóa tệp trung gian
    return res


METRICS = [("S_stops_per_veh", "S (lần/xe)", 2), ("D_delay_s", "D (s/xe)", 1), ("TT_s", "TT (s/xe)", 1),
           ("Qmax_veh", "Qmax (xe)", 1), ("Qmax_m_max", "Q (m)", 1), ("TH_vph", "TH (xe/h)", 0),
           ("pct_no_stop", "% xe không dừng", 1)]


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    keys = ["group", "case", "mode"]
    agg = df.groupby(keys, sort=False)[[m for m, _, _ in METRICS] + ["cohort_unfinished"]].mean().reset_index()
    return agg


def print_tables(agg: pd.DataFrame) -> None:
    names = {"independent": "Độc lập (đối chứng)", "greenwave": "Làn sóng xanh"}
    for case, g in agg.groupby("case", sort=False):
        print(f"\n### {case}")
        print(f"| Chỉ số | {names['independent']} | {names['greenwave']} | Thay đổi |")
        print("|---|---|---|---|")
        a = g[g["mode"] == "independent"].iloc[0]
        b = g[g["mode"] == "greenwave"].iloc[0]
        for key, label, nd in METRICS:
            va, vb = a[key], b[key]
            if key == "pct_no_stop":
                chg = f"{vb - va:+.1f} đ.%"            # hiệu số điểm phần trăm
            else:
                chg = f"{100 * (vb - va) / va:+.1f}%" if va else "-"
            print(f"| {label} | {va:.{nd}f} | {vb:.{nd}f} | {chg} |")
        if a["cohort_unfinished"] or b["cohort_unfinished"]:
            print(f"| Xe chưa hoàn thành | {a['cohort_unfinished']:.1f} | {b['cohort_unfinished']:.1f} | |")
        tt1, tt2 = a["TT_s"], b["TT_s"]
        print(f"\nMức giảm thời gian hành trình = 100 x (TT1 - TT2) / TT1 = {100 * (tt1 - tt2) / tt1:.1f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only", nargs="*", help="nhóm: KB1-KB2 KB3 KB4A KB4B")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()

    cases = [c for c in make_cases() if not args.only or c["group"] in args.only]
    jobs = [(c, m, s, args.out) for c in cases for m in ("independent", "greenwave")
            for s in range(1, args.seeds + 1)]
    Path(args.out).mkdir(exist_ok=True)
    print(f"Chạy {len(jobs)} lượt mô phỏng ({len(cases)} trường hợp x 2 chế độ x {args.seeds} seed)...")

    if args.jobs > 1:
        with ProcessPoolExecutor(args.jobs) as ex:
            results = list(ex.map(_job, jobs))
    else:
        results = []
        for i, j in enumerate(jobs, 1):
            results.append(_job(j))
            print(f"  [{i}/{len(jobs)}] {j[0]['case']} | {j[1]} | seed {j[2]}")

    df = pd.DataFrame(results)
    df.to_csv(Path(args.out) / "all_runs.csv", index=False)
    agg = summarize(df)
    agg.to_csv(Path(args.out) / "summary.csv", index=False)
    print_tables(agg)
    print(f"\nĐã lưu: {Path(args.out) / 'all_runs.csv'} và {Path(args.out) / 'summary.csv'}")


if __name__ == "__main__":
    main()
