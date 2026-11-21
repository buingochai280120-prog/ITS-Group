#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
corridor.py - Sinh bộ tệp cấu hình SUMO cho tuyến 5 nút giao N1-N5 và tính offset làn sóng xanh.

Đề tài 9: Xây dựng mô phỏng phối hợp đèn tín hiệu theo nguyên tắc làn sóng xanh.

Các tệp được sinh ra (mục 3.8 của báo cáo):
    corridor.nod.xml   - tọa độ các nút
    corridor.edg.xml   - các đoạn đường, số làn, tốc độ thiết kế
    corridor.net.xml   - mạng đường (do netconvert tạo)
    corridor.rou.xml   - loại xe Krauss + dòng xe
    corridor.add.xml   - chương trình đèn + offset tại 5 nút
    corridor.sumocfg   - cấu hình tổng
"""
from __future__ import annotations

import json
import math
import os
import random
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

# --------------------------------------------------------------------------
# 1. Hình học tuyến (Bảng 3.1)
# --------------------------------------------------------------------------
SEGMENTS_M = [450.0, 600.0, 500.0, 550.0]            # N1-N2, N2-N3, N3-N4, N4-N5
NODE_IDS = ["N1", "N2", "N3", "N4", "N5"]
NODE_X = [0.0]
for _L in SEGMENTS_M:
    NODE_X.append(NODE_X[-1] + _L)                    # [0, 450, 1050, 1550, 2100]
L_TOTAL = NODE_X[-1]                                  # 2100 m

STUB_UP_M = 300.0        # đoạn đường dẫn vào N1 (để xe xếp hàng không tràn ra ngoài mô hình)
STUB_DOWN_M = 300.0      # đoạn đường thoát sau N5
SIDE_LEN_M = 300.0       # chiều dài đường nhánh mỗi phía
MAIN_LANES = 3           # 3 làn/chiều, mỗi làn 3.5 m
SIDE_LANES = 2
LANE_WIDTH = 3.5

WEST_END, EAST_END = "W0", "E0"
MAIN_CHAIN = [WEST_END] + NODE_IDS + [EAST_END]       # W0 N1 N2 N3 N4 N5 E0

EAST_EDGES = [f"{a}_{b}" for a, b in zip(MAIN_CHAIN[:-1], MAIN_CHAIN[1:])]
WEST_EDGES = [f"{b}_{a}" for a, b in zip(MAIN_CHAIN[:-1], MAIN_CHAIN[1:])][::-1]
APPROACH_EDGES = EAST_EDGES[:-1]                      # cạnh đi vào N1..N5 (chiều ưu tiên)


# --------------------------------------------------------------------------
# 2. Cấu hình một lần chạy
# --------------------------------------------------------------------------
@dataclass
class Config:
    name: str = "KB2"
    mode: str = "greenwave"          # "independent" (KB1) | "greenwave" (KB2)
    flow_vph: float = 1200.0         # lưu lượng chiều N1->N5 (xe/giờ)
    reverse_flow_vph: float = 0.0    # lưu lượng chiều ngược N5->N1 (0 = không xét)
    cycle: int = 90                  # chu kỳ đèn chung (s)
    yellow: int = 3                  # thời gian vàng (s)
    main_green: Optional[int] = None  # None -> tự tính theo tỷ lệ 50/90
    speed_kmh: float = 50.0          # tốc độ thiết kế/giới hạn tốc độ trục chính
    kb1_offsets: str = "random"      # KB1: "random" (cố định theo kb1_seed) | "zero"
    kb1_seed: int = 2025
    offsets_override: Optional[List[int]] = None
    sim_end: int = 3600              # tổng thời gian mô phỏng (s)
    step: float = 1.0                # bước mô phỏng (s)
    warmup: int = 300                # giai đoạn khởi động bị loại khỏi thống kê (s)
    cohort_end: int = 3000           # chỉ theo dõi xe phát trong [warmup, cohort_end]
    # Thông số xe con (Bảng 3.2)
    veh_length: float = 4.5
    min_gap: float = 2.5
    accel: float = 2.5
    decel: float = 4.5
    tau: float = 1.0
    sigma: float = 0.5
    speed_dev: float = 0.1           # mặc định của SUMO

    # ---- tiện ích -------------------------------------------------------
    @property
    def v_ms(self) -> float:
        return self.speed_kmh / 3.6

    def green_split(self) -> Dict[str, int]:
        """Chia chu kỳ thành: xanh chính - vàng - xanh phụ - vàng.
        C = 90: 50 + 3 + 34 + 3  (đỏ trục chính = 34 + 3 = 37 s, đúng Mục 3.4)."""
        g_main = self.main_green if self.main_green is not None else round(self.cycle * 50 / 90)
        g_side = self.cycle - g_main - 2 * self.yellow
        if g_side <= 0:
            raise ValueError("Chu kỳ quá ngắn so với pha xanh chính + 2 pha vàng")
        return {"main_green": g_main, "yellow": self.yellow, "side_green": g_side,
                "main_red": g_side + self.yellow}


# --------------------------------------------------------------------------
# 3. Thuật toán tính thời gian di chuyển và offset (Mục 3.5)
# --------------------------------------------------------------------------
def round_half_up(x: float) -> int:
    return int(math.floor(x + 0.5))


def compute_offsets(v_kmh: float, cycle: int,
                    segments: List[float] = SEGMENTS_M) -> Dict[str, list]:
    """Giả mã 3.5.7:  t = L/V ; T_i = T_{i-1} + t ; O_i = T_i mod C ; làm tròn."""
    v = v_kmh / 3.6
    t_seg = [L / v for L in segments]
    T = [0.0]
    for t in t_seg:
        T.append(T[-1] + t)
    O = [Ti % cycle for Ti in T]
    O_cfg = [round_half_up(o) % cycle for o in O]
    return {"t_segment": t_seg, "T_cum": T, "offset_calc": O, "offset_cfg": O_cfg}


def resolve_offsets(cfg: Config) -> List[int]:
    if cfg.offsets_override is not None:
        return list(cfg.offsets_override)
    if cfg.mode == "greenwave":
        return compute_offsets(cfg.speed_kmh, cfg.cycle)["offset_cfg"]
    # KB1: các nút hoạt động độc lập, không phối hợp
    if cfg.kb1_offsets == "zero":
        return [0] * len(NODE_IDS)
    rng = random.Random(cfg.kb1_seed)
    return [0] + [rng.randrange(cfg.cycle) for _ in range(len(NODE_IDS) - 1)]


# --------------------------------------------------------------------------
# 4. Tìm SUMO
# --------------------------------------------------------------------------
def ensure_sumo_home() -> str:
    """Kiểm tra SUMO_HOME (Mục 3.7). Hỗ trợ cả bản cài bằng `pip install eclipse-sumo`."""
    home = os.environ.get("SUMO_HOME")
    if not home:
        try:
            import sumo  # type: ignore
            home = sumo.SUMO_HOME
            os.environ["SUMO_HOME"] = home
        except Exception:
            sys.exit("LỖI: chưa thiết lập biến môi trường SUMO_HOME "
                     "(hoặc chạy: pip install eclipse-sumo traci sumolib).")
    tools = os.path.join(home, "tools")
    if os.path.isdir(tools) and tools not in sys.path:
        sys.path.append(tools)
    return home


def sumo_binary(name: str) -> str:
    home = ensure_sumo_home()
    p = os.path.join(home, "bin", name)
    if os.path.exists(p):
        return p
    w = shutil.which(name)
    if w:
        return w
    sys.exit(f"LỖI: không tìm thấy '{name}' trong {home}/bin hoặc PATH")


# --------------------------------------------------------------------------
# 5. Ghi các tệp XML
# --------------------------------------------------------------------------
def _write(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def write_nodes(path: Path) -> None:
    rows = [f'    <node id="{WEST_END}" x="{-STUB_UP_M}" y="0" type="dead_end"/>',
            f'    <node id="{EAST_END}" x="{L_TOTAL + STUB_DOWN_M}" y="0" type="dead_end"/>']
    for nid, x in zip(NODE_IDS, NODE_X):
        rows.append(f'    <node id="{nid}" x="{x}" y="0" type="traffic_light"/>')
        k = nid[1:]
        rows.append(f'    <node id="U{k}" x="{x}" y="{SIDE_LEN_M}" type="dead_end"/>')
        rows.append(f'    <node id="D{k}" x="{x}" y="{-SIDE_LEN_M}" type="dead_end"/>')
    _write(path, '<?xml version="1.0" encoding="UTF-8"?>\n<nodes>\n' + "\n".join(rows) + "\n</nodes>\n")


def write_edges(path: Path, speed_ms: float) -> None:
    rows = []
    for e in EAST_EDGES + WEST_EDGES:
        a, b = e.split("_")
        rows.append(f'    <edge id="{e}" from="{a}" to="{b}" numLanes="{MAIN_LANES}" '
                    f'speed="{speed_ms:.2f}" priority="2" width="{LANE_WIDTH}"/>')
    for nid in NODE_IDS:
        k = nid[1:]
        for side in ("U", "D"):
            s = f"{side}{k}"
            rows.append(f'    <edge id="{s}_{nid}" from="{s}" to="{nid}" numLanes="{SIDE_LANES}" '
                        f'speed="13.89" priority="1" width="{LANE_WIDTH}"/>')
            rows.append(f'    <edge id="{nid}_{s}" from="{nid}" to="{s}" numLanes="{SIDE_LANES}" '
                        f'speed="13.89" priority="1" width="{LANE_WIDTH}"/>')
    _write(path, '<?xml version="1.0" encoding="UTF-8"?>\n<edges>\n' + "\n".join(rows) + "\n</edges>\n")


def build_net(outdir: Path, speed_ms: float) -> Path:
    nod, edg, net = outdir / "corridor.nod.xml", outdir / "corridor.edg.xml", outdir / "corridor.net.xml"
    write_nodes(nod)
    write_edges(edg, speed_ms)
    cmd = [sumo_binary("netconvert"), "-n", str(nod), "-e", str(edg), "-o", str(net),
           "--no-turnarounds", "true", "--no-warnings", "true",
           "--offset.disable-normalization", "true"]   # giữ nguyên tọa độ: x_TraCI = x_nút
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("netconvert lỗi:\n" + r.stderr)
    return net


def write_routes(path: Path, cfg: Config) -> None:
    period = 3600.0 / cfg.flow_vph                    # công thức quy đổi ở KB3
    v = cfg.v_ms
    xml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<routes>',
        f'    <vType id="car" vClass="passenger" carFollowModel="Krauss" length="{cfg.veh_length}" '
        f'minGap="{cfg.min_gap}" maxSpeed="{v:.2f}" accel="{cfg.accel}" decel="{cfg.decel}" '
        f'tau="{cfg.tau}" sigma="{cfg.sigma}" speedDev="{cfg.speed_dev}"/>',
        f'    <route id="eastbound" edges="{" ".join(EAST_EDGES)}"/>',
        f'    <flow id="main" type="car" route="eastbound" begin="0" end="{cfg.sim_end}" '
        f'period="{period:.4f}" departLane="free" departSpeed="max"/>',
    ]
    if cfg.reverse_flow_vph > 0:
        xml += [f'    <route id="westbound" edges="{" ".join(WEST_EDGES)}"/>',
                f'    <flow id="rev" type="car" route="westbound" begin="0" end="{cfg.sim_end}" '
                f'period="{3600.0 / cfg.reverse_flow_vph:.4f}" departLane="free" departSpeed="max"/>']
    xml.append('</routes>')
    _write(path, "\n".join(xml) + "\n")


def _link_table(net_path: Path) -> Dict[str, List[tuple]]:
    """Đọc corridor.net.xml -> {id nút: [(linkIndex, from_edge, dir), ...]}."""
    table: Dict[str, List[tuple]] = {}
    for c in ET.parse(net_path).getroot().iter("connection"):
        if c.get("tl") is None:
            continue
        table.setdefault(c.get("tl"), []).append(
            (int(c.get("linkIndex")), c.get("from"), c.get("dir")))
    return table


def write_tls(path: Path, net_path: Path, cfg: Config, offsets: List[int]) -> None:
    """Chương trình đèn: xanh chính -> vàng -> xanh phụ -> vàng, kèm offset từng nút."""
    gs = cfg.green_split()
    links = _link_table(net_path)
    main_edges = set(EAST_EDGES + WEST_EDGES)
    rows = ['<?xml version="1.0" encoding="UTF-8"?>', '<additional>']
    for nid, off in zip(NODE_IDS, offsets):
        lk = sorted(links[nid])
        n = lk[-1][0] + 1

        def state(main: str, side: str) -> str:
            s = ["r"] * n
            for idx, frm, d in lk:
                is_main = frm in main_edges
                ch = main if is_main else side
                if ch == "G" and d == "l":      # rẽ trái nhường đường (không dùng trong KB cơ sở)
                    ch = "g"
                s[idx] = ch
            return "".join(s)

        # offset: chương trình đèn dịch đi `off` giây -> pha xanh chính bắt đầu tại t = off (mod C)
        rows += [
            f'    <tlLogic id="{nid}" type="static" programID="greenwave" offset="{off}">',
            f'        <phase duration="{gs["main_green"]}" state="{state("G", "r")}"/>',
            f'        <phase duration="{gs["yellow"]}" state="{state("y", "r")}"/>',
            f'        <phase duration="{gs["side_green"]}" state="{state("r", "G")}"/>',
            f'        <phase duration="{gs["yellow"]}" state="{state("r", "y")}"/>',
            '    </tlLogic>',
        ]
    rows.append('</additional>')
    _write(path, "\n".join(rows) + "\n")


def write_sumocfg(path: Path, cfg: Config) -> None:
    _write(path, f'''<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="corridor.net.xml"/>
        <route-files value="corridor.rou.xml"/>
        <additional-files value="corridor.add.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{cfg.sim_end}"/>
        <step-length value="{cfg.step}"/>
    </time>
    <processing>
        <time-to-teleport value="-1"/>
    </processing>
    <report>
        <no-step-log value="true"/>
        <no-warnings value="true"/>
    </report>
</configuration>
''')


def build_scenario(cfg: Config, outdir) -> Path:
    """Sinh toàn bộ tệp đầu vào vào thư mục outdir, trả về đường dẫn corridor.sumocfg."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    net = build_net(outdir, cfg.v_ms)
    offsets = resolve_offsets(cfg)
    write_routes(outdir / "corridor.rou.xml", cfg)
    write_tls(outdir / "corridor.add.xml", net, cfg, offsets)
    write_sumocfg(outdir / "corridor.sumocfg", cfg)
    info = {"config": asdict(cfg), "offsets": offsets, "green_split": cfg.green_split(),
            "node_x": NODE_X, "node_ids": NODE_IDS}
    _write(outdir / "scenario.json", json.dumps(info, indent=2, ensure_ascii=False))
    return outdir / "corridor.sumocfg"


# --------------------------------------------------------------------------
# 6. In bảng offset để kiểm tra trước khi chạy (Bảng 3.5.6)
# --------------------------------------------------------------------------
def print_offset_table(v_kmh: float = 50.0, cycle: int = 90) -> None:
    r = compute_offsets(v_kmh, cycle)
    print(f"V = {v_kmh} km/h, C = {cycle} s")
    print(f"{'Nút':<4}{'L (m)':>8}{'t (s)':>9}{'T tích lũy (s)':>16}{'Offset tính':>13}{'Offset cấu hình':>17}")
    for i, nid in enumerate(NODE_IDS):
        L = 0.0 if i == 0 else SEGMENTS_M[i - 1]
        t = 0.0 if i == 0 else r["t_segment"][i - 1]
        print(f"{nid:<4}{L:>8.0f}{t:>9.1f}{r['T_cum'][i]:>16.1f}{r['offset_calc'][i]:>13.1f}{r['offset_cfg'][i]:>17d}")
    g = Config(cycle=cycle).green_split()["main_green"]
    print(f"Dải xanh lý tưởng = {g} s -> E = BW/C = {g / cycle * 100:.1f}%")


if __name__ == "__main__":
    print_offset_table()
