#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仿真器主体，组合各子模块：
- entities / geometry / grid / sensing / control / energy / locks
- 提供 Simulator.run() 与 Simulator.run_stream()
"""
from __future__ import annotations
from typing import Dict, List, Optional, Tuple, Any
import io
import json
import math
import random

from uavsim.entities import USV, UAV, Entity
from uavsim.geometry import Vec2, closest_on_seg, mul, norm, sample_in_poly, sub, to_vec2
from uavsim.grid import Grid
from uavsim.sensing import update_sensing
from uavsim.control import build_redline_posts_and_scans, assign_tasks, step_kinematics
from uavsim.energy import sync_uavs_on_deck, update_uav_energy
from uavsim.locks import update_locks

__all__ = ["Simulator"]


class Simulator:
    def __init__(self, area: Dict[str, List[float]], dt: float, duration: Optional[float], seed: int,
                 lock_success_prob: float = 0.8, uav_land_soc: float = 0.2, grid_cell_km: float = 5.0,
                 assign_period: float = 60.0, assign_horizon: float = 3600.0, guard_offset_km: float = 5.0,
                 uav_cruise_speed: float = 120.0, uav_autolaunch: bool = True, uav_autolaunch_delay: float = 30.0,
                 assign_danger_coeff: float = 600.0, redline_time_horizon: float = 3600.0):
        self.dt = float(dt)
        self.duration = None if duration is None else float(duration)
        self.t = 0.0
        self.rng = random.Random(seed)

        # 参数
        self.lock_success_prob = float(lock_success_prob)
        self.uav_land_soc = float(uav_land_soc)
        self.uav_cruise_speed = float(uav_cruise_speed)
        self.uav_autolaunch = bool(uav_autolaunch)
        self.uav_autolaunch_delay = float(uav_autolaunch_delay)

        # 几何与区域
        self.A = {f"A{i}": to_vec2(area[f"A{i}"]) for i in range(1, 9)}
        self.poly = [self.A[f"A{i}"] for i in range(1, 9)]
        self.red1, self.red2 = self.A["A1"], self.A["A6"]

        # 实体容器
        self.usvs: Dict[str, USV] = {}
        self.uavs: Dict[str, UAV] = {}

        # 甲板状态
        self.deck_busy: Dict[str, Optional[str]] = {}
        self.deck_reserve: Dict[str, Optional[str]] = {}

        # 指标
        self.metrics: Dict[str, Any] = {
            "intercepted_black": 0,
            "intercept_times": [],
            "white_lock_success": 0,
            "black_lock_success": 0,
            "white_locked_count": 0,         # 白方被击退出的 USV 数
            "white_locked_by_black": 0,      # ✨黑方锁定成功总次数（对白）
            "black_locked_by_white": 0,      # ✨白方锁定成功总次数（对黑）
            "collisions": 0,
            "black_penetrations": 0,
            "t_first_penetration": None,     # ✨黑方首次突防时间
            "per_black": {},
            "per_white_usv": {},
            "per_uav": {},
            "timeline": [],
        }

        # 网格
        self.grid = Grid(max(100.0, grid_cell_km * 1000.0))
        self.grid_cell_km = float(grid_cell_km)

        # 分配
        self.assign_period = float(assign_period)
        self.assign_horizon = float(assign_horizon)
        self.next_assign_time = 0.0
        self.assignments: Dict[str, Optional[str]] = {}

        # 布防
        self.guard_offset_m = max(0.0, guard_offset_km * 1000.0)
        self.white_guard: Dict[str, Vec2] = {}
        self.white_home: Dict[str, Vec2] = {}

        # UAV 巡航状态/感知缓存
        self.uav_scan_state: Dict[str, Dict[str, object]] = {}
        self.sensed_by_uav = set()
        self.sensed_by_white = set()
        self.uav_detect_map: Dict[str, List[str]] = {}

        # 策略权重
        self.assign_danger_coeff = float(assign_danger_coeff)
        self.redline_time_horizon = float(redline_time_horizon)

        # 前推控制（供 control.py 使用）
        self.forward_offset_m: float = 20000.0      # 20 km
        self.forward_activate_time: float = 7200.0  # 2 h
        self.push_eta_thresh: float = 5400.0        # 1.5 h

        # 初始化白方与布防点
        self._init_white()
        build_redline_posts_and_scans(self)

    # ---- 白方初始化 ----
    def _init_white(self) -> None:
        A7, A8 = self.A["A7"], self.A["A8"]
        mid = ((A7[0] + A8[0]) / 2, (A7[1] + A8[1]) / 2)
        dir78 = ((A8[0] - A7[0]), (A8[1] - A7[1]))
        L = max(1.0, norm(dir78))
        dir78 = (dir78[0] / L, dir78[1] / L)
        normal = (-dir78[1], dir78[0])
        for i, k in enumerate([-2, -1, 0, 1, 2], start=1):
            p = (mid[0] + dir78[0] * k * 5000.0, mid[1] + dir78[1] * k * 5000.0)
            name = f"w_usv{i}"
            heading = math.atan2(normal[1], normal[0])
            self.usvs[name] = USV("白", name, p, heading, 10.0)
            self.white_home[name] = p
            self.deck_busy[name] = None
            self.deck_reserve[name] = None
            self.metrics["per_white_usv"][name] = {"locks_started": 0, "locks_success": 0, "frozen_segments": []}
            self.assignments[name] = None
        for i in range(1, 6):
            host = f"w_usv{i}"
            name = f"w_uav{i}"
            pos = self.usvs[host].pos
            self.uavs[name] = UAV("白", name, pos, self.usvs[host].heading, 0.0, on_deck=host, task="charge")
            self.deck_busy[host] = name
            self.metrics["per_uav"][name] = {"air_seconds": 0, "recharge_seconds": 0, "cycles": 0, "last_soc": 1.0}

    # ---- 黑方初始化 ----
    def init_black(self, n: int) -> None:
        trap = [self.A["A2"], self.A["A3"], self.A["A4"], self.A["A5"]]
        for i in range(1, n + 1):
            p = sample_in_poly(trap, self.rng)
            to_red = sub(closest_on_seg(p, self.red1, self.red2), p)
            L = max(1.0, norm(to_red))
            to_red = (to_red[0] / L, to_red[1] / L)
            name = f"b_usv{i}"
            self.usvs[name] = USV("黑", name, p, math.atan2(to_red[1], to_red[0]), 10.0)
            self.metrics["per_black"][name] = {
                "t_first_detect_by_uav": None,
                "t_first_detect_by_usv": None,
                "t_first_lock_start": None,
                "locks_success": 0,
                "t_exit": None,
                "exit_reason": None,
            }

    def init_black_from_plan(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        idx = 1
        for k, seq in data.items():
            if not (isinstance(seq, list) and k.startswith("usv")) or not seq:
                continue
            first = seq[0]
            pt = to_vec2(first.get("point", (0.0, 0.0)))
            speed = float(first.get("speed", 10.0))
            if len(seq) >= 2 and "point" in seq[1]:
                p1 = to_vec2(seq[1]["point"])
                ddir = sub(p1, pt); L = max(1.0, norm(ddir)); ddir = (ddir[0]/L, ddir[1]/L)
            else:
                to_red = sub(closest_on_seg(pt, self.red1, self.red2), pt)
                L = max(1.0, norm(to_red)); ddir = (to_red[0]/L, to_red[1]/L)
            name = f"b_usv{idx}"
            self.usvs[name] = USV("黑", name, pt, math.atan2(ddir[1], ddir[0]), speed)
            self.metrics["per_black"][name] = {
                "t_first_detect_by_uav": None,
                "t_first_detect_by_usv": None,
                "t_first_lock_start": None,
                "locks_success": 0,
                "t_exit": None,
                "exit_reason": None,
            }
            idx += 1
        if idx == 1:
            raise ValueError("black-plan 文件中未找到任何 usv* 列表")

    # ---- 帧输出 ----
    def make_frame(self) -> Dict:
        frame = {"white": {}, "black": {}}
        for name, u in self.usvs.items():
            rec = {
                "pos": [round(u.pos[0], 3), round(u.pos[1], 3), 0],
                "type": "无人艇",
                "group": u.side,
                "state": ("退出" if u.exits else ("冻结" if self.t < u.frozen_until else ("锁定中" if u.lock_target else "移动"))),
                "velocity": [0, 0, 0],  # 简化展示；如需可按 heading 计算
            }
            (frame["white"] if u.side == '白' else frame["black"])[name] = rec
        for name, v in self.uavs.items():
            rec = {
                "pos": [round(v.pos[0], 3), round(v.pos[1], 3), 0],
                "type": "无人机",
                "group": "白",
                "state": ("退出" if v.exits else ("补能" if v.on_deck else ("返航" if v.task == "rtb" else "滞空"))),
                "velocity": [0, 0, 0],
            }
            frame["white"][name] = rec
        return frame

    # ---- 内部工具 ----
    def _alive_black_count(self) -> int:
        return sum(1 for u in self.usvs.values() if u.side == "黑" and not u.exits)

    def _should_stop(self) -> bool:
        time_up = (self.duration is not None) and (self.t >= float(self.duration))
        black_over = (self._alive_black_count() == 0)
        return time_up or black_over

    def _process_penetrations(self, last_pos: Dict[str, Vec2]) -> None:
        for name, u in self.usvs.items():
            if u.side == '黑' and (not u.exits):
                if _seg_intersect(last_pos[name], u.pos, self.red1, self.red2):
                    u.exits = True
                    self.metrics["black_penetrations"] = int(self.metrics["black_penetrations"]) + 1
                    if self.metrics.get("t_first_penetration") is None:
                        self.metrics["t_first_penetration"] = float(self.t)
                    pb = self.metrics["per_black"][name]
                    pb["t_exit"] = self.t
                    pb["exit_reason"] = "penetration"
                    self.metrics["timeline"].append({"t": self.t, "event": "penetration", "actors": [name]})

    def _finalize_metrics(self) -> None:
        # 可在这里补充收尾统计
        pass

    # ---- 主循环 ----
    def _step_all(self) -> None:
        assign_tasks(self)
        for u in list(self.usvs.values()):
            step_kinematics(self, u)
        for v in list(self.uavs.values()):
            step_kinematics(self, v)
        self.grid.rebuild(self.usvs, self.uavs)
        self.sensed_by_uav, self.sensed_by_white, self.uav_detect_map = update_sensing(
            self.usvs, self.uavs, self.grid, self.t, self.metrics
        )
        update_uav_energy(self)
        update_locks(self)

    def run(self) -> Dict[str, Dict]:
        logs: Dict[str, Dict] = {}
        last_pos: Dict[str, Vec2] = {n: u.pos for n, u in self.usvs.items()}
        self.grid.rebuild(self.usvs, self.uavs)
        sync_uavs_on_deck(self)

        while not self._should_stop():
            self._step_all()
            # 碰撞统计（100m）
            for name, u in self.usvs.items():
                if u.exits: continue
                for v in (x for x in self.usvs.values() if (not x.exits) and x.name != name):
                    if v.name <= name: continue
                    if norm(sub(u.pos, v.pos)) < 100.0:
                        self.metrics["collisions"] = int(self.metrics["collisions"]) + 1
                        self.metrics["timeline"].append({"t": self.t, "event": "collision", "actors": [u.name, v.name]})
            # 突防判定
            self._process_penetrations(last_pos)

            key = str(int(self.t))
            logs[key] = self.make_frame()
            for n, u in self.usvs.items():
                last_pos[n] = u.pos
            self.t += self.dt

        self._finalize_metrics()
        return logs

    def run_stream(self, fp: io.TextIOBase) -> None:
        self.grid.rebuild(self.usvs, self.uavs)
        sync_uavs_on_deck(self)
        fp.write('{')
        first = True
        last_pos: Dict[str, Vec2] = {n: u.pos for n, u in self.usvs.items()}
        try:
            while not self._should_stop():
                self._step_all()
                # 碰撞
                for name, u in self.usvs.items():
                    if u.exits: continue
                    for v in (x for x in self.usvs.values() if (not x.exits) and x.name != name):
                        if v.name <= name: continue
                        if norm(sub(u.pos, v.pos)) < 100.0:
                            self.metrics["collisions"] = int(self.metrics["collisions"]) + 1
                            self.metrics["timeline"].append({"t": self.t, "event": "collision", "actors": [u.name, v.name]})
                # 突防
                self._process_penetrations(last_pos)
                # 写一帧
                key = str(int(self.t))
                frame = self.make_frame()
                if not first:
                    fp.write(',\n')
                fp.write(json.dumps({key: frame}, ensure_ascii=False, indent=2)[1:-1])
                fp.flush()
                for n, u in self.usvs.items():
                    last_pos[n] = u.pos
                self.t += self.dt
                first = False
        finally:
            fp.write('\n}')
            fp.flush()


def _seg_intersect(a1: Vec2, a2: Vec2, b1: Vec2, b2: Vec2) -> bool:
    def ccw(p1, p2, p3):
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])
    return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))