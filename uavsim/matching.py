#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hungarian (Kuhn-Munkres) for rectangular non-negative cost matrices.
- Input: cost: List[List[float]] of size m x n (m rows = workers, n cols = jobs)
- Output: assignment list of length m: for each row i -> column j (or -1 if none)
We pad to square with large cost to handle m!=n.
"""
from __future__ import annotations
from typing import List
import math

INF = 1e18

def hungarian(cost: List[List[float]]) -> List[int]:
    m = len(cost)
    n = len(cost[0]) if m else 0
    s = max(m, n)
    # build square matrix
    a = [[0.0]*s for _ in range(s)]
    big = max(1.0, max((c for row in cost for c in row), default=1.0)) * 1e6
    for i in range(s):
        for j in range(s):
            if i < m and j < n:
                a[i][j] = cost[i][j]
            else:
                a[i][j] = big
    # KM (minimization)
    u = [0.0]* (s+1)
    v = [0.0]* (s+1)
    p = [0]* (s+1)
    way = [0]* (s+1)
    for i in range(1, s+1):
        p[0] = i
        j0 = 0
        minv = [INF]*(s+1)
        used = [False]*(s+1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = 0
            for j in range(1, s+1):
                if used[j]:
                    continue
                cur = a[i0-1][j-1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(0, s+1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        # Augmenting
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    # Build assignment
    ans = [-1]*m
    for j in range(1, s+1):
        i = p[j]
        if 1 <= i <= m and 1 <= j <= n:
            # ignore padded matches to huge cost
            if a[i-1][j-1] < big*0.5:
                ans[i-1] = j-1
    return ans

__all__ = ["hungarian"]
