#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实体定义模块（USV / UAV）
从旧大文件抽离的数据类与公共别名，供其它子模块引用。
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Union


Vec2 = Tuple[float, float]

@dataclass
class USV:
    side: str        # "白" / "黑"
    name: str
    pos: Vec2
    heading: float   # 弧度
    speed: float     # m/s
    # 规则/状态
    rmin: float = 20.0
    frozen_until: float = 0.0
    exits: bool = False
    lock_target: Optional[str] = None
    lock_timer: float = 0.0
    lock_successes: int = 0

@dataclass
class UAV:
    side: str        # 固定 "白"
    name: str
    pos: Vec2
    heading: float
    speed: float
    # 规则/状态
    rmin: float = 100.0
    energy: float = 1.0       # [0,1]
    on_deck: Optional[str] = None  # 落舰宿主 USV 名称
    exits: bool = False
    task: str = "patrol"           # patrol | rtb | charge(=on_deck)
    rtb_host: Optional[str] = None # 返航目标甲板（预留）

Entity = Union[USV, UAV]

__all__ = [
    "Vec2", "USV", "UAV", "Entity",
]
