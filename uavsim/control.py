#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制与机动（重写 step_kinematics 精简版 + 深域巡航航线）
- USV：黑直奔红线；白优先追分配目标，否则回布防点（支持 guard/forward 两线切换）
- UAV：返航优先；有目标追“最易被白艇锁”的黑方；无目标跑闭环航点
"""
from __future__ import annotations
from typing import Any, Dict, List, Tuple
import math
import re

from uavsim.geometry import (
    Vec2, add, clamp, closest_on_seg, dot, mul, norm, point_in_poly, sub, unit
)
from uavsim.entities import USV, UAV, Entity
from uavsim.sensing import WHITE_USV_LOCK, WHITE_UAV_DETECT
from uavsim.game import GameModel, Strategy

__all__ = [
    'build_redline_posts_and_scans', 'steer_towards', 'clamp_to_area',
    'eta_to_lock40', 'assign_tasks', 'step_kinematics'
]

# ---------------- 公用小工具 ----------------

def _unit(v: Vec2) -> Vec2:
    L = max(1e-9, norm(v))
    return (v[0]/L, v[1]/L)

def _num_suffix(name: str, default: int) -> int:
    m = re.search(r'(\d+)$', name)
    return int(m.group(1)) if m else default

# 决策 guard/forward 模式
def _update_push_mode(sim: Any) -> None:
    activate_t = getattr(sim, "forward_activate_time", 7200.0)   # 2h 后可前推
    eta_thresh = getattr(sim, "push_eta_thresh",    5400.0)      # 敌到红线 > 1.5h 才前推

    seen = [b for b in sim.usvs.values()
            if b.side == "黑" and (not b.exits)
            and (b.name in sim.sensed_by_uav or b.name in sim.sensed_by_white)]

    if not seen:
        if sim.t >= activate_t:
            sim.push_mode = "forward"
        return

    def t_to_red(b: USV) -> float:
        d = norm(sub(closest_on_seg(b.pos, sim.red1, sim.red2), b.pos))
        return d / 10.0

    min_eta = min(t_to_red(b) for b in seen)
    sim.push_mode = "forward" if min_eta > eta_thresh else "guard"

# ---------------- 红线布防 + UAV 深域闭环 ----------------

def build_redline_posts_and_scans(sim: Any) -> None:
    """
    生成红线守位点（内偏 guard_offset_m）与前出点（+forward_offset_m），
    **按 A7→A8 的空间顺序**将 5 艘白艇与 5 个守位点一一对应，避免上下交叉。
    同时为每艘白艇对应的 UAV 生成沿红线方向的闭环巡航段。
    """
    A1, A6, A7, A8 = sim.A["A1"], sim.A["A6"], sim.A["A7"], sim.A["A8"]

    # 红线切向 + “向内”法向
    red_vec = sub(A6, A1)
    Lred = max(1.0, norm(red_vec))
    u_t = (red_vec[0]/Lred, red_vec[1]/Lred)

    cen = (sum(p[0] for p in sim.poly)/len(sim.poly), sum(p[1] for p in sim.poly)/len(sim.poly))
    n = (-u_t[1], u_t[0])
    if dot(sub(cen, A1), n) < 0:  # 保证 n 指向任务区内部
        n = (-n[0], -n[1])

    guard_off = getattr(sim, "guard_offset_m", getattr(sim, "guard_offset_km", 5.0) * 1000.0)
    forward_off = getattr(sim, "forward_offset_m", 20000.0)

    # A7→A8 的方向与投影：用于“从下到上”的几何排序
    dir78 = _unit(sub(A8, A7))
    mid78 = ((A7[0] + A8[0]) * 0.5, (A7[1] + A8[1]) * 0.5)
    def proj78(p: Vec2) -> float:
        d = sub(p, mid78)
        return d[0]*dir78[0] + d[1]*dir78[1]

    # 在红线上等距取 5 个点（1/6..5/6），再向内偏移
    posts_guard: List[Vec2] = []
    posts_forward: List[Vec2] = []
    for i in range(1, 6):
        t = i / 6.0
        base = add(A1, mul(u_t, Lred * t))
        g = add(base, mul(n, guard_off))
        f = add(base, mul(n, guard_off + forward_off))
        posts_guard.append(clamp_to_area(sim, g))
        posts_forward.append(clamp_to_area(sim, f))

    # 白艇按 A7→A8 排序；守位点也按同一方向排序 → 一一对应
    whites = [u for u in sim.usvs.values() if u.side == '白' and not u.exits]
    whites_sorted = sorted(whites, key=lambda u: proj78(u.pos))
    guard_sorted = sorted(posts_guard, key=proj78)
    forward_sorted = sorted(posts_forward, key=proj78)

    sim.white_guard = {}
    sim.white_forward = {}
    for u, gp, fp in zip(whites_sorted, guard_sorted, forward_sorted):
        sim.white_guard[u.name] = gp
        sim.white_forward[u.name] = fp

    # 为每艘白艇对应的 UAV（同编号）生成闭环扫描段（±lane，深入到 0.7*探测距）
    gap = Lred / 5.0
    lane_half = 0.3 * gap
    depth = min(40000.0, 0.7 * WHITE_UAV_DETECT)
    back = 3000.0

    if not getattr(sim, "uav_scan_state", None):
        sim.uav_scan_state = {}

    for u in whites_sorted:
        gp = sim.white_guard[u.name]
        left  = add(gp, mul(u_t, -lane_half))
        right = add(gp, mul(u_t, +lane_half))
        near_left  = add(left,  mul(n, -back))
        deep_left  = add(left,  mul(n, +depth))
        deep_right = add(right, mul(n, +depth))
        near_right = add(right, mul(n, -back))
        wps = [near_left, deep_left, deep_right, near_right]
        wps = [clamp_to_area(sim, p) for p in wps]

        u_idx = _num_suffix(u.name, default=1)  # w_usvK → w_uavK
        uav_name = f"w_uav{u_idx}"
        sim.uav_scan_state[uav_name] = {"idx": 0, "waypoints": wps}

# ---------------- 运动学与约束 ----------------

def steer_towards(heading: float, desired_dir: Vec2, vmax_turn: float) -> float:
    target = math.atan2(desired_dir[1], desired_dir[0])
    d = (target - heading + math.pi) % (2 * math.pi) - math.pi
    return heading + (max(-vmax_turn, min(vmax_turn, d)))

def clamp_to_area(sim: Any, p: Vec2) -> Vec2:
    if point_in_poly(p, sim.poly):
        return p
    cx = sum(v[0] for v in sim.poly) / len(sim.poly)
    cy = sum(v[1] for v in sim.poly) / len(sim.poly)
    q = p
    r = (q[0], q[1])
    for _ in range(32):
        mid = ((q[0] + cx) / 2, (q[1] + cy) / 2)
        if point_in_poly(mid, sim.poly):
            q = mid
        else:
            r = mid
    return q if point_in_poly(q, sim.poly) else r

# ---------------- 分配辅助 ----------------

def _vel_vec(u: USV) -> Vec2:
    return (math.cos(u.heading) * u.speed, math.sin(u.heading) * u.speed)

def eta_to_lock40(w: USV, b: USV) -> float:
    d = norm(sub(b.pos, w.pos))
    if d <= WHITE_USV_LOCK:
        return 0.0
    u_bw = unit(sub(w.pos, b.pos))
    vb = _vel_vec(b)
    vb_norm = norm(vb)
    cos_th = dot(unit(vb) if vb_norm > 1e-6 else (0.0, 0.0), u_bw)
    v_rel = 10.0 + max(0.0, 10.0 * cos_th)
    v_rel = max(1.0, v_rel)
    return (d - WHITE_USV_LOCK) / v_rel

# ---------- UAV 一对一分配，避免多机盯同一目标 ----------
def assign_uav_targets(sim: Any) -> None:
    if not hasattr(sim, "uav_assignments"):
        sim.uav_assignments = {}
    if not hasattr(sim, "uav_reserve"):
        sim.uav_reserve = {}
    ttl = getattr(sim, "uav_reserve_ttl", 120.0)

    uavs = [u for u in getattr(sim, "uavs", {}).values()
            if (not u.exits) and (not u.on_deck) and u.task not in ("rtb", "charge")]

    seen = set(sim.sensed_by_uav) | set(sim.sensed_by_white)
    blacks = [b for b in sim.usvs.values()
              if b.side == "黑" and (not b.exits) and b.name in seen]

    now = sim.t
    sim.uav_reserve = {
        k: v for k, v in sim.uav_reserve.items()
        if v and v[1] > now and (k in sim.usvs and not sim.usvs[k].exits)
    }

    if not uavs or not blacks:
        for u in uavs:
            sim.uav_assignments[u.name] = None
        return

    from uavsim.matching import hungarian
    m, n = len(uavs), len(blacks)
    cost = [[0.0] * n for _ in range(m)]

    whites = [w for w in sim.usvs.values() if w.side == "白" and (not w.exits) and sim.t >= w.frozen_until]
    for i, u in enumerate(uavs):
        for j, b in enumerate(blacks):
            dij = norm(sub(b.pos, u.pos))
            eta40 = min((eta_to_lock40(w, b) for w in whites), default=1e6)
            bias = -0.01 * min(eta40, 3600.0)
            if b.name in sim.uav_reserve and sim.uav_reserve[b.name][0] != u.name:
                cost[i][j] = 1e9
            else:
                cost[i][j] = dij + bias

    match = hungarian(cost)
    for k, v in list(sim.uav_reserve.items()):
        if v[0] in [u.name for u in uavs] and v[1] <= now:
            sim.uav_reserve.pop(k, None)

    for i, j in enumerate(match):
        u = uavs[i]
        if 0 <= j < n:
            b = blacks[j]
            sim.uav_assignments[u.name] = b.name
            sim.uav_reserve[b.name] = (u.name, now + ttl)
        else:
            sim.uav_assignments[u.name] = None

# ---------------- 任务分配（含博弈器） ----------------

def assign_tasks(sim: Any) -> None:
    if sim.t < sim.next_assign_time:
        return
    sim.next_assign_time = sim.t + sim.assign_period
    _update_push_mode(sim)

    whites = [w for w in sim.usvs.values()
              if w.side == '白' and (not w.exits) and sim.t >= w.frozen_until]
    seen_names = set(sim.sensed_by_uav) | set(sim.sensed_by_white)
    blacks = [b for b in sim.usvs.values()
              if b.side == '黑' and (not b.exits) and sim.t >= b.frozen_until and b.name in seen_names]

    for w in whites:
        tgt = sim.assignments.get(w.name)
        if tgt and (tgt not in sim.usvs or sim.usvs[tgt].exits):
            sim.assignments[w.name] = None

    if not whites or not blacks:
        assign_uav_targets(sim)
        return

    state = {
        'whites': [{'id': w.name, 'pos': w.pos} for w in whites],
        'blacks': [{'id': b.name, 'pos': b.pos} for b in blacks],
    }

    if not hasattr(sim, 'game_model'):
        sim.game_model = GameModel(
            detection_range=None,
            lock_reward=getattr(sim, 'gm_lock_reward', 120.0),
            penalty_uncovered=getattr(sim, 'gm_penalty_uncovered', -160.0),
            cost_distance_factor=getattr(sim, 'gm_cost_k', 1e-4),
        )

    prof = sim.game_model.decide(state)

    for w in whites:
        st: Strategy = prof.get(w.name, Strategy('maintain'))
        new = st.target_id if st.action == 'lock' else None
        old = sim.assignments.get(w.name)
        if new != old:
            sim.assignments[w.name] = new
            sim.metrics['timeline'].append({
                't': sim.t, 'event': 'assign', 'actors': [w.name, new] if new else [w.name]
            })
    assign_uav_targets(sim)

# ---------------- 推进行为 ----------------

def step_kinematics(sim: Any, ent: Entity) -> None:
    dt = sim.dt

    # 退出/冻结
    if isinstance(ent, USV):
        if ent.exits or sim.t < ent.frozen_until:
            return
    else:  # UAV
        if ent.exits:
            return

    # ---------- UAV ----------
    if isinstance(ent, UAV):
        if ent.on_deck:
            ent.task = "charge"
            host = sim.usvs.get(ent.on_deck)
            if host is not None:
                ent.pos = host.pos
                ent.heading = host.heading
            ent.speed = 0.0
            return

        if ent.task == "rtb":
            host_name = ent.rtb_host
            if (not host_name) or (host_name not in sim.usvs) or sim.usvs[host_name].exits:
                options = [uname for uname in sim.usvs
                           if sim.usvs[uname].side == '白'
                           and (not sim.usvs[uname].exits)
                           and sim.deck_busy.get(uname) is None
                           and sim.deck_reserve.get(uname) in (None, ent.name)]
                if options:
                    host_name = min(options, key=lambda uname: norm(sub(sim.usvs[uname].pos, ent.pos)))
                    ent.rtb_host = host_name
                    sim.deck_reserve[host_name] = ent.name
            if host_name:
                host = sim.usvs[host_name]
                if norm(sub(host.pos, ent.pos)) < 100.0:
                    ent.speed = 20.0
                    ent.on_deck = host_name
                    ent.task = 'charge'
                    ent.pos = host.pos
                    sim.deck_busy[host_name] = ent.name
                    sim.deck_reserve[host_name] = None
                    return
                ddir = unit(sub(host.pos, ent.pos))
            else:
                ddir = (math.cos(ent.heading), math.sin(ent.heading))
        else:
            assigned = getattr(sim, "uav_assignments", {}).get(ent.name)
            if assigned and assigned in sim.usvs and (not sim.usvs[assigned].exits):
                ddir = unit(sub(sim.usvs[assigned].pos, ent.pos))
            else:
                cand_names = list(sim.sensed_by_uav | sim.sensed_by_white)
                best_b = None
                best_eta = float('inf')
                if cand_names:
                    whites = [w for w in sim.usvs.values() if w.side == '白' and (not w.exits) and sim.t >= w.frozen_until]
                    for n in cand_names:
                        b = sim.usvs.get(n)
                        if not b or b.exits:
                            continue
                        eta_min = min((eta_to_lock40(w, b) for w in whites), default=float('inf'))
                        if eta_min < best_eta:
                            best_eta, best_b = eta_min, b
                if best_b is not None and best_eta < float('inf'):
                    ddir = unit(sub(best_b.pos, ent.pos))
                else:
                    st = sim.uav_scan_state.get(ent.name)
                    if st and st.get("waypoints"):
                        wps: List[Vec2] = st["waypoints"]  # type: ignore
                        idx: int = int(st.get("idx", 0))   # type: ignore
                        tgt = wps[idx]
                        if norm(sub(tgt, ent.pos)) < 2000.0:
                            st["idx"] = (idx + 1) % max(1, len(wps))
                            tgt = wps[st["idx"]]
                        ddir = unit(sub(tgt, ent.pos))
                    else:
                        ddir = (math.cos(ent.heading), math.sin(ent.heading))

        if ent.speed < sim.uav_cruise_speed:
            ent.speed = sim.uav_cruise_speed

    # ---------- USV ----------
    else:  # isinstance(ent, USV)
        if ent.side == '黑':
            goal = closest_on_seg(ent.pos, sim.red1, sim.red2)
            ddir = unit(sub(goal, ent.pos))
        else:
            tgt_name = sim.assignments.get(ent.name)
            tgt = sim.usvs.get(tgt_name) if tgt_name else None
            if tgt and (not tgt.exits):
                eta = eta_to_lock40(ent, tgt)
                lead = min(eta, 120.0)
                vb = (math.cos(tgt.heading) * tgt.speed, math.sin(tgt.heading) * tgt.speed)
                pred = add(tgt.pos, mul(vb, lead))
                ddir = unit(sub(pred, ent.pos))
            else:
                home_map = sim.white_forward if getattr(sim, "push_mode", "guard") == "forward" else sim.white_guard
                guard = home_map.get(ent.name, sim.white_home.get(ent.name, ent.pos))
                ddir = unit(sub(guard, ent.pos))

    # ---------- 推进 & 边界 ----------
    vmax_turn = (ent.speed * dt) / ent.rmin
    ent.heading = steer_towards(ent.heading, ddir, vmax_turn)
    vdir = (math.cos(ent.heading), math.sin(ent.heading))
    ent.pos = add(ent.pos, mul(vdir, ent.speed * dt))
    if not point_in_poly(ent.pos, sim.poly):
        ent.pos = clamp_to_area(sim, ent.pos)