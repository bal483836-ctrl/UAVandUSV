#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
锁定 / 冻结 / 退出 判定模块
规则：
- 锁定判定步进式计时：连续 300s 未中断且一次伯努利(p=lock_success_prob)成功 → 记一次“锁定成功”
- 成功一次：被锁方冻结 300s；成功两次：被锁方退出
- 白方发起锁定需满足：目标在 40km 锁距内，且存在“探测链路”（UAV 或 白USV 感知）
- 黑方探测半径 30km、锁距 40km（与白方等同），对白方无人机无探测与锁定能力（本文件只处理 USV↔USV）
事件：
- 各类成功/开始事件写入 sim.metrics["timeline"]
- 计数写入：white_lock_success / black_lock_success、black_locked_by_white / white_locked_by_black
"""
from __future__ import annotations
from typing import Any, List

from uavsim.geometry import norm, sub
from uavsim.entities import USV

# 锁距（白/黑 USV 对 USV）：40 km
from uavsim.sensing import WHITE_USV_LOCK  # 期望为 40000.0

# 黑方 USV 对白方 USV 的探测半径（30 km）
try:
    from uavsim.sensing import BLACK_USV_DETECT  # 期望为 30000.0
except Exception:  # 兼容旧版
    BLACK_USV_DETECT = 30000.0

__all__ = ["update_locks"]


def _clear_out_of_range_locks(sim: Any) -> None:
    """超距或目标退出 → 清锁并清计时。"""
    for u in sim.usvs.values():
        if u.lock_target and not u.exits:
            tgt = sim.usvs.get(u.lock_target)
            # 目标不存在 / 目标退出 / 超出锁距 → 中断
            if (not tgt) or tgt.exits or norm(sub(tgt.pos, u.pos)) > WHITE_USV_LOCK:
                u.lock_target = None
                u.lock_timer = 0.0


def _advance_and_try_commit(attacker: USV, sim: Any, dt: float) -> None:
    """
    已在锁定态：累计计时，满 300s 后做一次伯努利判定并施加效果。
    成功：
      - 被锁者冻结 300s
      - 被锁者累计 lock_successes += 1；累计到 2 则退出
      - 计数与时间线更新
    失败或成功后：清除锁定态（需要重新开始）
    """
    attacker.lock_timer += dt
    if attacker.lock_timer < 300.0:
        return

    # 满 5 分钟，判成败
    if sim.rng.random() < sim.lock_success_prob:
        tgt = sim.usvs.get(attacker.lock_target)
        if tgt and not tgt.exits:
            # 施加冻结
            tgt.frozen_until = max(tgt.frozen_until, sim.t + 300.0)
            tgt.lock_successes += 1

            if attacker.side == "白":
                # 白方成功一次
                sim.metrics["white_lock_success"] += 1
                sim.metrics["black_locked_by_white"] = sim.metrics.get("black_locked_by_white", 0) + 1
                # 黑方个体统计
                pb = sim.metrics["per_black"].get(tgt.name)
                if isinstance(pb, dict):
                    pb["locks_success"] = pb.get("locks_success", 0) + 1
                sim.metrics["timeline"].append({"t": sim.t, "event": "lock_success_white", "actors": [attacker.name, tgt.name]})
            else:
                # 黑方成功一次
                sim.metrics["black_lock_success"] += 1
                sim.metrics["white_locked_by_black"] = sim.metrics.get("white_locked_by_black", 0) + 1
                sim.metrics["timeline"].append({"t": sim.t, "event": "lock_success_black", "actors": [attacker.name, tgt.name]})

            # 成功两次 → 目标退出
            if tgt.lock_successes >= 2 and not tgt.exits:
                tgt.exits = True
                if tgt.side == "黑":
                    # 黑方被白方拦截退出
                    sim.metrics["intercepted_black"] += 1
                    sim.metrics["intercept_times"].append(sim.t)
                    pb = sim.metrics["per_black"].get(tgt.name)
                    if isinstance(pb, dict):
                        pb["t_exit"] = sim.t
                        pb["exit_reason"] = "intercepted"
                else:
                    # 白方艇退出（被黑锁两次）
                    sim.metrics["white_locked_count"] += 1
                    sim.metrics["timeline"].append({"t": sim.t, "event": "white_usv_exit", "actors": [tgt.name]})

    # 不论成功与否，锁定轮次结束 → 清除
    attacker.lock_target = None
    attacker.lock_timer = 0.0


def update_locks(sim: Any) -> None:
    dt = sim.dt

    # 1) 先清理所有“超距/无效”的锁定
    _clear_out_of_range_locks(sim)

    # 2) 过滤出可行动个体（未退出、未冻结）
    whites: List[USV] = [x for x in sim.usvs.values() if x.side == "白" and (not x.exits) and sim.t >= x.frozen_until]
    blacks: List[USV] = [x for x in sim.usvs.values() if x.side == "黑" and (not x.exits) and sim.t >= x.frozen_until]

    # UAV 探测字典： black_name -> [uav_names...]
    uav_detect = getattr(sim, "uav_detect_map", {})

    # 3) 先推进“已在锁定中的人”的计时与结算（避免同一帧又开始新锁）
    for w in whites:
        if w.lock_target:
            _advance_and_try_commit(w, sim, dt)
    for b in blacks:
        if b.lock_target:
            _advance_and_try_commit(b, sim, dt)

    # 4) 发起新的锁定（白方优先执行，体现“交接”）
    #    白方条件：目标在 40km 内 且 (白USV感知 或 UAV感知链路存在)
    for w in whites:
        # 已在锁定中的跳过（上面推进可能清掉；这里确保只发起新的）
        if w.lock_target or w.exits or sim.t < w.frozen_until:
            continue

        # 候选黑方
        cands: List[USV] = []
        for b in blacks:
            if b.exits or sim.t < b.frozen_until:
                continue
            d = norm(sub(b.pos, w.pos))
            # 必须在锁距内
            if d > WHITE_USV_LOCK:
                continue
            # 必须存在“探测链路”：白感知 或 UAV感知
            if (getattr(sim, "sensed_by_white", set()) and (b.name in sim.sensed_by_white)) \
               or (uav_detect.get(b.name)):
                cands.append(b)

        if not cands:
            continue

        # 就近选一个（更复杂的匹配由上层 assign_tasks 保证唯一性）
        b = min(cands, key=lambda x: norm(sub(x.pos, w.pos)))
        w.lock_target = b.name
        w.lock_timer = 0.0

        # 指标与时间线
        per_w = sim.metrics["per_white_usv"].get(w.name)
        if isinstance(per_w, dict):
            per_w["locks_started"] = per_w.get("locks_started", 0) + 1
        per_b = sim.metrics["per_black"].get(b.name)
        if isinstance(per_b, dict) and per_b.get("t_first_lock_start") is None:
            per_b["t_first_lock_start"] = sim.t
        sim.metrics["timeline"].append({"t": sim.t, "event": "lock_start_white", "actors": [w.name, b.name]})

    # 5) 黑方发起锁定（对白 USV）
    #    条件：距离 ≤ 锁距（40km）且 ≤ 黑方探测半径（30km）
    for b in blacks:
        if b.lock_target or b.exits or sim.t < b.frozen_until:
            continue

        cands: List[USV] = []
        for w in whites:
            if w.exits or sim.t < w.frozen_until:
                continue
            d = norm(sub(w.pos, b.pos))
            if d <= WHITE_USV_LOCK and d <= BLACK_USV_DETECT:
                cands.append(w)

        if not cands:
            continue

        w = min(cands, key=lambda x: norm(sub(x.pos, b.pos)))
        b.lock_target = w.name
        b.lock_timer = 0.0
        sim.metrics["timeline"].append({"t": sim.t, "event": "lock_start_black", "actors": [b.name, w.name]})