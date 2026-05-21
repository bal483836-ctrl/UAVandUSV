#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
邻域网格哈希（从大文件抽离）。
用于在感知/碰撞等场景下加速“附近实体”查询。
API：Grid(cell_size_m).rebuild(usvs, uavs) / neighbors(pos, radius_cells)
"""
from __future__ import annotations
from typing import Dict, Iterable, List, Tuple
from collections import defaultdict
import math
from uavsim.entities import USV, UAV
from uavsim.geometry import Vec2

class Grid:
    def __init__(self, cell_size_m: float):
        self.cell = float(cell_size_m)
        self.map: Dict[Tuple[int, int], List[Tuple[str, str]]] = defaultdict(list)

    def _key(self, p: Vec2) -> Tuple[int, int]:
        return (int(math.floor(p[0] / self.cell)), int(math.floor(p[1] / self.cell)))

    def rebuild(self, usvs: Dict[str, 'USV'], uavs: Dict[str, 'UAV']):
        """重建索引：USV 全部入表；UAV 仅在空中时入表（on_deck 的不入表）。"""
        self.map.clear()
        for name, u in usvs.items():
            if getattr(u, 'exits', False):
                continue
            self.map[self._key(u.pos)].append(("usv", name))
        for name, v in uavs.items():
            if getattr(v, 'exits', False) or getattr(v, 'on_deck', None):
                continue
            self.map[self._key(v.pos)].append(("uav", name))

    def neighbors(self, pos: Vec2, radius_cells: int) -> Iterable[Tuple[str, str]]:
        kx, ky = self._key(pos)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                yield from self.map.get((kx + dx, ky + dy), [])

__all__ = ["Grid"]
