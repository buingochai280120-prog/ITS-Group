#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
runner.py - Chạy một kịch bản mô phỏng SUMO qua TraCI và tính 5 chỉ số MOEs (Mục 2.8, 4.2).

Ví dụ:
    python runner.py --mode independent            # KB1
    python runner.py --mode greenwave              # KB2
    python runner.py --mode greenwave --gui        # xem bằng sumo-gui
    python runner.py --mode greenwave --flow 1500  # KB3-4
    python runner.py --mode greenwave --cycle 100  # KB4-A3
    python runner.py --mode greenwave --speed 60   # KB4-B3

Cách đo (bám Mục 2.8 và 4.2):
  * Mốc đầu = tâm nút N1 (x = 0 m), mốc cuối = tâm nút N5 (x = 2100 m). Tuyến thẳng theo trục x nên
    vị trí x của xe chính là cự ly dọc tuyến; thời điểm qua mốc được nội suy tuyến tính trong bước mô phỏng.
  * Số lần dừng S : đếm 1 lần mỗi khi tốc độ chuyển từ >= 0,1 m/s xuống < 0,1 m/s trong đoạn N1 -> N5.
  * Thời gian hành trình TT = t_ra_N5 - t_vào_N1.
  * Độ trễ D = TT - 2100 / v_des  (v_des = min(maxSpeed, giới hạn * speedFactor) của từng xe).
  * Hàng chờ Q : (a) tổng số xe đang dừng trên các làn tiếp cận N1-N5 ở mỗi bước, lấy max (Qmax, xe);
                 (b) chiều dài hàng chờ lớn nhất tại từng nút, tính từ vạch dừng đến đuôi xe dừng cuối (m).
  * Thông lượng TH = số xe qua N5 trong cửa sổ quan sát / thời gian cửa sổ (xe/giờ).
  * Nhóm xe thống kê (cohort) = xe phát trong [warmup, cohort_end]; theo dõi đến hết hành trình.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import statistics as st
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from corridor import (APPROACH_EDGES, L_TOTAL, MAIN_LANES, NODE_IDS, Config,
                      build_scenario, ensure_sumo_home, sumo_binary)

ensure_sumo_home()
import traci  # noqa: E402  (phải import sau khi thêm SUMO_HOME/tools vào sys.path)

X_START, X_END = 0.0, L_TOTAL
STOP_SPEED = 0.1          # m/s - ngưỡng coi là xe đang dừng (Mục 2.8)


@dataclass
class VehState:
    depart: float
    vdes: Optional[float] = None
    prev_x: Optional[float] = None
    prev_t: Optional[float] = None
    t_in: Optional[float] = None
    t_out: Optional[float] = None
    stops: int = 0
    was_stopped: bool = False


def _cross_time(target: float, x0: Optional[float], t0: Optional[float], x1: float, t1: float) -> float:
    """Nội suy thời điểm xe qua mặt cắt x = target trong khoảng (t0, t1]."""
    if x0 is None or x1 <= x0:
        return t1
    return t0 + (target - x0) / (x1 - x0) * (t1 - t0)


def run_simulation(cfg: Config, outdir, seed: int = 1, gui: bool = False,
                   save_trajectories: bool = False, verbose: bool = True,
                   gui_delay_ms: int = 50) -> Dict:
    outdir = Path(outdir)
    sumocfg = build_scenario(cfg, outdir)
    binary = sumo_binary("sumo-gui" if gui else "sumo")
    cmd = [binary, "-c", str(sumocfg), "--seed", str(seed),
           "--tripinfo-output", str(outdir / "tripinfo.xml")]
    if gui:
        cmd += ["--start", "--quit-on-end", "--delay", str(gui_delay_ms)]

    label = f"run_{os.getpid()}_{time.time_ns()}"
    traci.start(cmd, label=label)
    conn = traci.getConnection(label)

    # --- chuẩn bị đo hàng chờ: các làn tiếp cận N1..N5 theo chiều ưu tiên -------------
    approach_lanes = {nid: [f"{e}_{i}" for i in range(MAIN_LANES)]
                      for nid, e in zip(NODE_IDS, APPROACH_EDGES)}
    lane_len = {ln: conn.lane.getLength(ln) for lanes in approach_lanes.values() for ln in lanes}

    vehs: Dict[str, VehState] = {}
    exit_times: List[float] = []            # mọi xe qua N5 (tính thông lượng)
    arrived_total = 0
    qmax_veh = 0
    qmax_m = {nid: 0.0 for nid in NODE_IDS}
    traj_rows: List[tuple] = []

    n_steps = int(round(cfg.sim_end / cfg.step))
    for _ in range(n_steps):
        conn.simulationStep()
        t = conn.simulation.getTime()
        arrived_total += conn.simulation.getArrivedNumber()

        for vid in conn.simulation.getDepartedIDList():
            if vid.startswith("main"):
                vehs[vid] = VehState(depart=t)

        # ---- theo dõi từng xe thuộc dòng chính -----------------------------------
        for vid in conn.vehicle.getIDList():
            s = vehs.get(vid)
            if s is None or s.t_out is not None:
                continue
            x = conn.vehicle.getPosition(vid)[0]
            v = conn.vehicle.getSpeed(vid)
            if s.vdes is None:
                lane = conn.vehicle.getLaneID(vid)
                s.vdes = min(conn.vehicle.getMaxSpeed(vid),
                             conn.lane.getMaxSpeed(lane) * conn.vehicle.getSpeedFactor(vid))
            if s.t_in is None and x >= X_START:
                s.t_in = _cross_time(X_START, s.prev_x, s.prev_t, x, t)
            if s.t_in is not None:
                if v < STOP_SPEED:
                    if not s.was_stopped:
                        s.stops += 1
                        s.was_stopped = True
                else:
                    s.was_stopped = False
                if x >= X_END:
                    s.t_out = _cross_time(X_END, s.prev_x, s.prev_t, x, t)
                    exit_times.append(s.t_out)
            if save_trajectories and -50 <= x <= X_END + 50:
                traj_rows.append((round(t, 2), vid, round(x, 2), round(v, 2)))
            s.prev_x, s.prev_t = x, t

        # ---- hàng chờ trên các làn tiếp cận -----------------------------------------
        if t >= cfg.warmup:
            total_halt = 0
            for nid, lanes in approach_lanes.items():
                for ln in lanes:
                    h = conn.lane.getLastStepHaltingNumber(ln)
                    if h == 0:
                        continue
                    total_halt += h
                    rear_min = None
                    for vid in conn.lane.getLastStepVehicleIDs(ln):
                        if conn.vehicle.getSpeed(vid) < STOP_SPEED:
                            rear = conn.vehicle.getLanePosition(vid) - conn.vehicle.getLength(vid)
                            rear_min = rear if rear_min is None else min(rear_min, rear)
                    if rear_min is not None:
                        qmax_m[nid] = max(qmax_m[nid], lane_len[ln] - rear_min)
            qmax_veh = max(qmax_veh, total_halt)

    conn.close()

    # --- tổng hợp chỉ số ------------------------------------------------------------
    cohort = [s for s in vehs.values() if cfg.warmup <= s.depart <= cfg.cohort_end]
    done = [s for s in cohort if s.t_out is not None]
    tt = [s.t_out - s.t_in for s in done]
    delay = [(s.t_out - s.t_in) - L_TOTAL / s.vdes for s in done]
    stops = [s.stops for s in done]
    window_h = (cfg.sim_end - cfg.warmup) / 3600.0
    th = sum(1 for te in exit_times if cfg.warmup <= te <= cfg.sim_end) / window_h

    res = {
        "name": cfg.name, "mode": cfg.mode, "seed": seed,
        "flow_vph": cfg.flow_vph, "cycle": cfg.cycle, "speed_kmh": cfg.speed_kmh,
        "S_stops_per_veh": st.mean(stops) if stops else float("nan"),
        "D_delay_s": st.mean(delay) if delay else float("nan"),
        "TT_s": st.mean(tt) if tt else float("nan"),
        "TT_free_s": L_TOTAL / (cfg.v_ms),
        "Qmax_veh": qmax_veh,
        "Qmax_m_max": max(qmax_m.values()),
        "Qmax_m_by_node": {k: round(v, 1) for k, v in qmax_m.items()},
        "TH_vph": th,
        "completed_total": arrived_total,
        "cohort_n": len(cohort), "cohort_done": len(done),
        "cohort_unfinished": len(cohort) - len(done),
        "pct_no_stop": 100.0 * sum(1 for k in stops if k == 0) / len(stops) if stops else float("nan"),
    }
    (outdir / "metrics.json").write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
    if save_trajectories:
        with open(outdir / "trajectories.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "veh", "x", "speed"])
            w.writerows(traj_rows)

    if verbose:
        print_report(res, outdir)
    return res


def print_report(r: Dict, outdir) -> None:
    print("=" * 62)
    print(f" {r['name']} | chế độ: {r['mode']} | seed {r['seed']}")
    print(f" Q = {r['flow_vph']:.0f} xe/h | C = {r['cycle']} s | V = {r['speed_kmh']:.0f} km/h")
    print("-" * 62)
    print(f" Tổng số xe hoàn thành hành trình : {r['completed_total']} xe")
    print(f" Số xe dừng cực đại (các làn N1-N5): {r['Qmax_veh']} xe")
    print(f" Hàng chờ dài nhất                 : {r['Qmax_m_max']:.1f} m  {r['Qmax_m_by_node']}")
    print("-" * 62)
    print(f" Số lần dừng S      : {r['S_stops_per_veh']:.2f} lần/xe   (xe không dừng: {r['pct_no_stop']:.1f}%)")
    print(f" Độ trễ TB D        : {r['D_delay_s']:.1f} s/xe")
    print(f" Thời gian hành trình: {r['TT_s']:.1f} s/xe   (tự do: {r['TT_free_s']:.1f} s)")
    print(f" Thông lượng TH     : {r['TH_vph']:.0f} xe/h")
    print(f" Nhóm thống kê      : {r['cohort_done']}/{r['cohort_n']} xe hoàn thành "
          f"({r['cohort_unfinished']} chưa hoàn thành)")
    print(f" Dữ liệu chi tiết   : {Path(outdir) / 'tripinfo.xml'}")
    print("=" * 62)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Mô phỏng làn sóng xanh N1-N5 bằng SUMO + TraCI")
    p.add_argument("--mode", choices=["independent", "greenwave"], default="greenwave",
                   help="independent = KB1 (đối chứng), greenwave = KB2 (phối hợp)")
    p.add_argument("--flow", type=float, default=1200, help="lưu lượng N1->N5 (xe/h)")
    p.add_argument("--reverse-flow", type=float, default=0, help="lưu lượng chiều ngược (xe/h), 0 = không xét")
    p.add_argument("--cycle", type=int, default=90, help="chu kỳ đèn (s)")
    p.add_argument("--speed", type=float, default=50, help="tốc độ thiết kế (km/h)")
    p.add_argument("--kb1-offsets", choices=["random", "zero"], default="random")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--end", type=int, default=3600, help="thời gian mô phỏng (s)")
    p.add_argument("--step", type=float, default=1.0)
    p.add_argument("--gui", action="store_true", help="dùng sumo-gui")
    p.add_argument("--trajectories", action="store_true", help="ghi quỹ đạo xe (để vẽ biểu đồ không gian-thời gian)")
    p.add_argument("--out", default=None, help="thư mục đầu ra (mặc định runs/<mode>)")
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    cfg = Config(name="KB1" if a.mode == "independent" else "KB2", mode=a.mode,
                 flow_vph=a.flow, reverse_flow_vph=a.reverse_flow, cycle=a.cycle,
                 speed_kmh=a.speed, kb1_offsets=a.kb1_offsets, sim_end=a.end, step=a.step)
    out = a.out or os.path.join("runs", a.mode)
    run_simulation(cfg, out, seed=a.seed, gui=a.gui, save_trajectories=a.trajectories)


if __name__ == "__main__":
    main()
