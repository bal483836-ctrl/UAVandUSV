#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静态博弈分配模型（完全信息、离散时间、白方多玩家）
- 每个时间步生成白方策略集（锁定某黑/保持/规避）
- 构造支付函数（锁定收益、未覆盖惩罚、距离代价）
- 先求纯策略纳什均衡；若无，则用复制子动态近似混合策略NE
- 输出：每个白USV的一步动作（Strategy）

坐标单位：米。速度：m/s。
"""
from __future__ import annotations
import itertools
import logging
import math
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("uavsim.game")

# ------------ 数据结构 ------------
class Strategy:
    __slots__ = ("action", "target_id")
    def __init__(self, action: str, target_id: Optional[str] = None):
        self.action = action  # 'lock' | 'maintain' | 'evade'
        self.target_id = target_id
    def __repr__(self) -> str:
        return f"Lock({self.target_id})" if self.action == "lock" else self.action


class StrategyGenerator:
    def __init__(self, detection_range: Optional[float] = None) -> None:
        self.detection_range = detection_range  # 若为None，则不以距离裁剪

    def generate(self, state: Dict) -> Dict[str, List[Strategy]]:
        strategies: Dict[str, List[Strategy]] = {}
        whites = state.get("whites", [])
        blacks = state.get("blacks", [])
        for w in whites:
            wid = w["id"]
            wpos = tuple(w.get("pos", (0.0, 0.0)))  # type: ignore
            cand: List[Strategy] = []
            for b in blacks:
                bid = b["id"]
                if self.detection_range is not None:
                    bpos = tuple(b.get("pos", (0.0, 0.0)))  # type: ignore
                    dx, dy = bpos[0] - wpos[0], bpos[1] - wpos[1]
                    if math.hypot(dx, dy) > self.detection_range:
                        continue
                cand.append(Strategy("lock", target_id=bid))
            cand.append(Strategy("maintain"))
            cand.append(Strategy("evade"))
            strategies[wid] = cand
        return strategies


class PayoffCalculator:
    def __init__(self, *, lock_reward: float = 120.0, penalty_uncovered: float = -160.0,
                 cost_distance_factor: float = 1e-4) -> None:
        self.lock_reward = lock_reward
        self.penalty_uncovered = penalty_uncovered
        self.cost_distance_factor = cost_distance_factor

    def compute(self, profile: Dict[str, Strategy], state: Dict) -> Dict[str, float]:
        pay: Dict[str, float] = {w["id"]: 0.0 for w in state.get("whites", [])}
        # 统计被锁定目标
        locked_by: Dict[str, List[str]] = {}
        for wid, st in profile.items():
            if st.action == "lock" and st.target_id is not None:
                locked_by.setdefault(st.target_id, []).append(wid)
        # 个体项
        for wid, st in profile.items():
            if st.action == "lock" and st.target_id is not None:
                wpos = next((tuple(w.get("pos", (0.0, 0.0))) for w in state.get("whites", []) if w["id"] == wid), (0.0, 0.0))
                bpos = next((tuple(b.get("pos", (0.0, 0.0))) for b in state.get("blacks", []) if b["id"] == st.target_id), (0.0, 0.0))
                dist = math.hypot(bpos[0]-wpos[0], bpos[1]-wpos[1])
                n_locker = max(1, len(locked_by.get(st.target_id, [])))
                reward = self.lock_reward / n_locker
                pay[wid] += reward - self.cost_distance_factor * dist
        # 全局未覆盖惩罚（对所有白平均施加）
        for b in state.get("blacks", []):
            bid = b["id"]
            if bid not in locked_by:
                for wid in list(pay.keys()):
                    pay[wid] += self.penalty_uncovered
        return pay


class GameSolver:
    def find_pure_NE(self, strategies: Dict[str, List[Strategy]], calc: PayoffCalculator, state: Dict) -> Optional[Tuple[Dict[str, Strategy], Dict[str, float]]]:
        players = list(strategies.keys())
        pure: List[Tuple[Dict[str, Strategy], Dict[str, float]]] = []
        for combo in itertools.product(*[strategies[p] for p in players]):
            prof = {players[i]: combo[i] for i in range(len(players))}
            pay = calc.compute(prof, state)
            # 单边偏离检验
            ok = True
            for p in players:
                cur = prof[p]
                cur_pay = pay[p]
                for alt in strategies[p]:
                    if alt is cur:
                        continue
                    prof2 = dict(prof)
                    prof2[p] = alt
                    pay2 = calc.compute(prof2, state)
                    if pay2[p] > cur_pay + 1e-9:
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                pure.append((prof, pay))
        if not pure:
            return None
        # 选总收益最大的一个
        best = max(pure, key=lambda t: sum(t[1].values()))
        return best

    def find_mixed_NE(self, strategies: Dict[str, List[Strategy]], calc: PayoffCalculator, state: Dict,
                       *, max_iter: int = 300, tol: float = 1e-6) -> Dict[str, Dict[str, float]]:
        players = list(strategies.keys())
        dist: Dict[str, List[float]] = {p: [1.0/len(strategies[p])] * len(strategies[p]) for p in players}
        for _ in range(max_iter):
            prev = {p: dist[p][:] for p in players}
            for p in players:
                acts = strategies[p]
                m = len(acts)
                # 期望收益
                exp = [0.0]*m
                others = [q for q in players if q != p]
                grids = [range(len(strategies[q])) for q in others]
                for i, a in enumerate(acts):
                    tot = 0.0
                    for idxs in itertools.product(*grids):
                        prob = 1.0
                        prof = {p: a}
                        for j, q in enumerate(others):
                            prof[q] = strategies[q][idxs[j]]
                            prob *= dist[q][idxs[j]]
                        if prob == 0:
                            continue
                        tot += prob * calc.compute(prof, state)[p]
                    exp[i] = tot
                base = sum(dist[p][i]*exp[i] for i in range(m))
                for i in range(m):
                    dist[p][i] = dist[p][i] * (exp[i]/base) if base > 0 else dist[p][i]
                s = sum(dist[p])
                dist[p] = [x/s if s > 0 else 1.0/m for x in dist[p]]
            # 收敛检测
            diff = sum(abs(dist[p][i]-prev[p][i]) for p in players for i in range(len(dist[p])))
            if diff < tol:
                break
        return {p: {str(strategies[p][i]): dist[p][i] for i in range(len(strategies[p]))} for p in players}


class GameModel:
    def __init__(self, *, detection_range: Optional[float] = None,
                 lock_reward: float = 120.0, penalty_uncovered: float = -160.0, cost_distance_factor: float = 1e-4) -> None:
        self.gen = StrategyGenerator(detection_range=detection_range)
        self.calc = PayoffCalculator(lock_reward=lock_reward, penalty_uncovered=penalty_uncovered,
                                     cost_distance_factor=cost_distance_factor)
        self.solver = GameSolver()

    def decide(self, state: Dict) -> Dict[str, Strategy]:
        S = self.gen.generate(state)
        pure = self.solver.find_pure_NE(S, self.calc, state)
        if pure is not None:
            prof, _ = pure
            return prof
        mixed = self.solver.find_mixed_NE(S, self.calc, state)
        # 取最大概率动作落地
        out: Dict[str, Strategy] = {}
        for pid, dist in mixed.items():
            best_key = max(dist.items(), key=lambda kv: kv[1])[0]
            for st in S[pid]:
                if str(st) == best_key:
                    out[pid] = st
                    break
        return out
