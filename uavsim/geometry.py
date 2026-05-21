#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
几何/基础工具函数（从大文件抽离）。
与原脚本 API 兼容：to_vec2 / dot / sub / add / mul / norm / unit / clamp /
closest_on_seg / point_in_poly / seg_intersect。
后续模块可直接 from uavsim.geometry import * 引用。
"""
from __future__ import annotations
from typing import Sequence, Tuple, List
import math

Vec2 = Tuple[float, float]

# ---- 基础向量运算 ----

def to_vec2(a: Sequence[float]) -> Vec2:
    return (float(a[0]), float(a[1]))

def dot(a: Vec2, b: Vec2) -> float:
    return a[0]*b[0] + a[1]*b[1]

def sub(a: Vec2, b: Vec2) -> Vec2:
    return (a[0]-b[0], a[1]-b[1])

def add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0]+b[0], a[1]+b[1])

def mul(a: Vec2, s: float) -> Vec2:
    return (a[0]*s, a[1]*s)

def norm(a: Vec2) -> float:
    return math.hypot(a[0], a[1])

def unit(a: Vec2) -> Vec2:
    L = norm(a)
    return (a[0]/L, a[1]/L) if L > 1e-9 else (1.0, 0.0)

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

# ---- 线段/多边形工具 ----

def closest_on_seg(p: Vec2, a: Vec2, b: Vec2) -> Vec2:
    ap, ab = sub(p, a), sub(b, a)
    L2 = dot(ab, ab)
    if L2 <= 0:
        return a
    t = max(0.0, min(1.0, dot(ap, ab) / L2))
    return add(a, mul(ab, t))

def point_in_poly(pt: Vec2, poly: List[Vec2]) -> bool:
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        # 经典奇偶规则
        if (y1 > y) != (y2 > y):
            x_int = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1
            if x < x_int:
                inside = not inside
    return inside

def seg_intersect(a1: Vec2, a2: Vec2, b1: Vec2, b2: Vec2) -> bool:
    def ccw(p1: Vec2, p2: Vec2, p3: Vec2) -> bool:
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])
    return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))

# ---- 采样（用于在多边形内随机初始化） ----

def sample_in_poly(poly: List[Vec2], rng) -> Vec2:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    for _ in range(20000):
        p = (rng.uniform(minx, maxx), rng.uniform(miny, maxy))
        if point_in_poly(p, poly):
            return p
    return poly[0]

__all__ = [
    'Vec2', 'to_vec2', 'dot', 'sub', 'add', 'mul', 'norm', 'unit', 'clamp',
    'closest_on_seg', 'point_in_poly', 'seg_intersect', 'sample_in_poly',
]
