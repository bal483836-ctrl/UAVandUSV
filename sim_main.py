#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最简命令行入口：
  # 推荐（只给两件必需文件；若文件名用默认，也可都省略）
  python sim_main.py --area ./任务区域.json --black-plan ./方案1-黑方艇数量_3.json
  python sim_main.py
"""
from __future__ import annotations
import argparse, json, os
from typing import Optional
import uavsim.sim as sim_mod
Simulator = sim_mod.Simulator

DEFAULT_AREA = "任务区域.json"
DEFAULT_BLACK_PLAN = "方案1-黑方艇数量_3.json"

def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser()
    # 仅这俩常用；也可省略，自动找默认文件
    p.add_argument("--area", default=None, help="任务区域.json；若省略则尝试 ./任务区域.json")
    p.add_argument("--black-plan", default=None, help="黑方方案 JSON（usv*）；若省略则尝试 ./方案1-黑方艇数量_3.json")

    # 其它全部默认值，用户无需输入；duration 默认 None=直到黑方清零
    p.add_argument("--dt", type=float, default=10.0, help="步长秒")
    p.add_argument("--duration", type=float, default=None, help="总秒数；省略则直到在场黑方为 0 自动结束")
    p.add_argument("--outfile", default="./logDemo_out.json", help="日志输出")
    p.add_argument("--metrics", default="./metrics.json", help="指标输出")
    p.add_argument("--seed", type=int, default=42, help="随机种子")
    p.add_argument("--lock-success-prob", type=float, default=0.8, help="锁定成功概率")
    p.add_argument("--uav-land-soc", type=float, default=0.2, help="UAV 返航阈值 [0,1]")
    p.add_argument("--grid-cell-km", type=float, default=5.0, help="网格格长（km）")
    p.add_argument("--assign-period", type=float, default=60.0, help="分配周期（秒）")
    p.add_argument("--assign-horizon", type=float, default=3600.0, help="分配考虑的最大ETA（秒）")
    p.add_argument("--guard-offset-km", type=float, default=50.0, help="红线内侧布防偏置（km）")
    p.add_argument("--uav-cruise-speed", type=float, default=120.0, help="UAV 巡航速度（m/s）")
    p.add_argument("--uav-autolaunch", dest="uav_autolaunch", action="store_true", default=True, help="UAV 充满后自动起飞")
    p.add_argument("--no-uav-autolaunch", dest="uav_autolaunch", action="store_false")
    p.add_argument("--uav-autolaunch-delay", type=float, default=11800.0, help="UAV 自动起飞延迟（秒）")
    p.add_argument("--stream", dest="stream", action="store_true", default=True, help="启用流式写出（默认开）")
    p.add_argument("--no-stream", dest="stream", action="store_false")

    # 阵型前推 / 威胁权重
    p.add_argument("--assign-danger-coeff", type=float, default=600.0, help="红线威胁权重（秒）")
    p.add_argument("--redline-time-horizon", type=float, default=3600.0, help="红线威胁时间窗口（秒）")
    p.add_argument("--forward-offset-km", type=float, default=20.0, help="前出线相对红线向内偏置（km）")
    p.add_argument("--forward-activate", type=float, default=7200.0, help="允许前推的最早时间（s）")
    p.add_argument("--push-eta-thresh", type=float, default=5400.0, help="黑方到红线ETA阈值（s）")

    args = p.parse_args(argv)

    # 自动补全默认文件路径
    if not args.area:
        guess = os.path.join(os.getcwd(), DEFAULT_AREA)
        if os.path.exists(guess):
            args.area = guess
        else:
            p.error("缺少 --area，且未在当前目录找到默认文件：任务区域.json")
    if not args.black_plan:
        guess = os.path.join(os.getcwd(), DEFAULT_BLACK_PLAN)
        if os.path.exists(guess):
            args.black_plan = guess
        else:
            p.error("缺少 --black-plan，且未在当前目录找到默认文件：方案1-黑方艇数量_3.json")

    return args

def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    with open(args.area, "r", encoding="utf-8") as f:
        area = json.load(f)

    sim = Simulator(
        area=area, dt=args.dt, duration=args.duration, seed=args.seed,
        lock_success_prob=args.lock_success_prob, uav_land_soc=args.uav_land_soc,
        grid_cell_km=args.grid_cell_km, assign_period=args.assign_period,
        assign_horizon=args.assign_horizon, guard_offset_km=args.guard_offset_km,
        uav_cruise_speed=args.uav_cruise_speed,
        uav_autolaunch=args.uav_autolaunch, uav_autolaunch_delay=args.uav_autolaunch_delay,
        assign_danger_coeff=args.assign_danger_coeff, redline_time_horizon=args.redline_time_horizon
    )
    # 前推参数写回（米/秒）
    sim.forward_offset_m = args.forward_offset_km * 1000.0
    sim.forward_activate_time = args.forward_activate
    sim.push_eta_thresh = args.push_eta_thresh

    # 黑方初始化（来自方案文件）
    sim.init_black_from_plan(args.black_plan)

    # 运行与输出
    os.makedirs(os.path.dirname(args.outfile) or ".", exist_ok=True)
    if args.stream:
        with open(args.outfile, "w", encoding="utf-8") as fp:
            sim.run_stream(fp)
    else:
        logs = sim.run()
        with open(args.outfile, "w", encoding="utf-8") as f:
            json.dump(logs, f, ensure_ascii=False, indent=2)

    with open(args.metrics, "w", encoding="utf-8") as f:
        json.dump(sim.metrics, f, ensure_ascii=False, indent=2)

    print(f"[OK] 日志: {args.outfile}\n[OK] 指标: {args.metrics}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
