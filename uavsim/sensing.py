#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
感知模块：
- 基于物理探测更新：sensed_by_uav, sensed_by_white, uav_detect_map
- 修复冻结期 KeyError：对仍在场的黑方统一预建键
- 提供邻域查询与简单探测几何
"""
from __future__ import annotations
from typing import Dict, Iterable, List, Set, Tuple
import math

from uavsim.geometry import Vec2, dot, sub, norm, unit
from uavsim.grid import Grid
from uavsim.entities import USV, UAV

# 常量（题面）
WHITE_USV_DETECT = 20000.0  # 白艇探测半径
BLACK_USV_DETECT = 30000.0  # 黑艇探测半径
WHITE_USV_LOCK = 40000.0    # 锁定最大距离
WHITE_UAV_DETECT = 60000.0  # 无人机探测半径
UAV_FOV_COS = math.cos(math.radians(30.0))  # ±30° 扇形

__all__ = [
    'WHITE_USV_DETECT','BLACK_USV_DETECT','WHITE_USV_LOCK','WHITE_UAV_DETECT','UAV_FOV_COS',
    'nearby_usvs','nearby_uavs','detect_black_by_white_usv','detect_white_by_black_usv',
    'detect_black_by_white_uav','update_sensing'
]

# ---- 邻域 ----

def nearby_usvs(grid: Grid, pos: Vec2, max_range: float, usvs: Dict[str, USV]) -> Iterable[USV]:
    r = int(math.ceil(max_range / grid.cell))
    seen = set()
    for typ, name in grid.neighbors(pos, r):
        if typ != 'usv' or name in seen:
            continue
        u = usvs[name]
        if u.exits:
            continue
        if norm(sub(u.pos, pos)) <= max_range:
            seen.add(name)
            yield u

def nearby_uavs(grid: Grid, pos: Vec2, max_range: float, uavs: Dict[str, UAV]) -> Iterable[UAV]:
    r = int(math.ceil(max_range / grid.cell))
    seen = set()
    for typ, name in grid.neighbors(pos, r):
        if typ != 'uav' or name in seen:
            continue
        v = uavs[name]
        if v.exits or v.on_deck:
            continue
        if norm(sub(v.pos, pos)) <= max_range:
            seen.add(name)
            yield v

# ---- 探测判定 ----

def detect_black_by_white_usv(w: USV, b: USV) -> bool:
    return norm(sub(b.pos, w.pos)) <= WHITE_USV_DETECT

def detect_white_by_black_usv(b: USV, w: USV) -> bool:
    return norm(sub(w.pos, b.pos)) <= BLACK_USV_DETECT

def detect_black_by_white_uav(u: UAV, b: USV) -> bool:
    if u.on_deck:
        return False
    vec = sub(b.pos, u.pos)
    d = norm(vec)
    if d > WHITE_UAV_DETECT:
        return False
    fwd = (math.cos(u.heading), math.sin(u.heading))
    return dot(unit(vec), fwd) >= UAV_FOV_COS

# ---- 主更新 ----

def update_sensing(usvs: Dict[str, USV], uavs: Dict[str, UAV], grid: Grid, t: float, metrics: Dict) -> Tuple[Set[str], Set[str], Dict[str, List[str]]]:
    blacks_all = [x for x in usvs.values() if x.side == '黑' and (not x.exits)]
    whites_active = [x for x in usvs.values() if x.side == '白' and (not x.exits) and t >= x.frozen_until]

    sensed_by_uav: Set[str] = set()
    sensed_by_white: Set[str] = set()
    uav_detect_map: Dict[str, List[str]] = {b.name: [] for b in blacks_all}

    # UAV 对黑方的扇形探测
    for u in uavs.values():
        if u.exits or u.on_deck:
            continue
        for b in nearby_usvs(grid, u.pos, WHITE_UAV_DETECT, usvs):
            if b.side != '黑':
                continue
            if detect_black_by_white_uav(u, b):
                sensed_by_uav.add(b.name)
                uav_detect_map.setdefault(b.name, []).append(u.name)  # 兜底，避免 KeyError
                # 记录首探测时刻
                pb = metrics.get('per_black', {}).get(b.name)
                if pb is not None and pb.get('t_first_detect_by_uav') is None:
                    pb['t_first_detect_by_uav'] = t

    # 白艇对黑方的圆形探测（白艇冻结时无探测能力）
    for w in whites_active:
        for b in nearby_usvs(grid, w.pos, WHITE_USV_DETECT, usvs):
            if b.side != '黑':
                continue
            if detect_black_by_white_usv(w, b):
                sensed_by_white.add(b.name)
                pb = metrics.get('per_black', {}).get(b.name)
                if pb is not None and pb.get('t_first_detect_by_usv') is None:
                    pb['t_first_detect_by_usv'] = t

    return sensed_by_uav, sensed_by_white, uav_detect_map
