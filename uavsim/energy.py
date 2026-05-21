#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV 能量与返航/落舰逻辑（独立模块）。
- 5 小时充满:  dt/18000.0
- 空中放电:    dt/7200.0 (≈2 小时耗尽)
- 在甲板上：每帧同步到宿主 USV 的位置/朝向，速度=0
- 自动起飞：满足能量阈值且过了 uav_autolaunch_delay
"""
from __future__ import annotations
from typing import Any

from uavsim.geometry import clamp, norm, sub

__all__ = ["sync_uavs_on_deck", "update_uav_energy"]


def sync_uavs_on_deck(sim: Any) -> None:
    """把所有 on_deck 的 UAV 与宿主 USV 对齐位姿。"""
    for u in sim.uavs.values():
        if getattr(u, "on_deck", None):
            host = sim.usvs.get(u.on_deck)
            if host is not None:
                u.pos = host.pos
                u.heading = host.heading
                u.speed = 0.0


def update_uav_energy(sim: Any) -> None:
    dt = sim.dt
    for name, u in sim.uavs.items():
        if u.exits:
            continue
        if u.on_deck:
            host = sim.usvs.get(u.on_deck)
            if (not host) or host.exits:
                u.exits = True
                continue
            # 在甲板：持续同步位姿
            u.pos = host.pos
            u.heading = host.heading
            u.speed = 0.0
            # 冻结时补能暂停
            if sim.t < host.frozen_until:
                u.task = "charge"
                continue
            # 充电：5 小时充满
            u.energy = clamp(u.energy + dt / 18000.0, 0.0, 1.0)
            u.task = "charge"
            sim.metrics["per_uav"][name]["recharge_seconds"] += dt
            sim.metrics["per_uav"][name]["last_soc"] = u.energy
            # 自动起飞（可配置）
            if sim.uav_autolaunch and (sim.t >= sim.uav_autolaunch_delay) and (u.energy >= 0.99):
                host_name = u.on_deck
                u.on_deck = None
                u.task = "patrol"
                u.rtb_host = None
                if sim.deck_busy.get(host_name) == name:
                    sim.deck_busy[host_name] = None
                sim.metrics["per_uav"][name]["cycles"] += 1
                # 起飞后巡航速度
                if getattr(u, "speed", 0.0) < sim.uav_cruise_speed:
                    u.speed = sim.uav_cruise_speed
        else:
            # 空中放电：2 小时耗尽
            u.energy = clamp(u.energy - dt / 7200.0, 0.0, 1.0)
            sim.metrics["per_uav"][name]["air_seconds"] += dt
            sim.metrics["per_uav"][name]["last_soc"] = u.energy
            # 触发返航：选择最近的空闲甲板（考虑已预留）
            if u.energy <= sim.uav_land_soc and u.task != "rtb":
                options = [uname for uname in sim.usvs
                           if (sim.usvs[uname].side == '白'
                               and (not sim.usvs[uname].exits)
                               and sim.deck_busy.get(uname) is None
                               and sim.deck_reserve.get(uname) in (None, name))]
                if options:
                    host = min(options, key=lambda uname: norm(sub(sim.usvs[uname].pos, u.pos)))
                    sim.deck_reserve[host] = name
                    u.rtb_host = host
                    u.task = "rtb"
            # 清理无效占用
            for k in list(sim.deck_busy.keys()):
                if sim.deck_busy[k] == name and u.on_deck != k:
                    sim.deck_busy[k] = None
