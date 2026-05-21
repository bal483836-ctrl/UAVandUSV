#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, os, tempfile, unittest
from typing import Dict

from uavsim.sim import Simulator


def _area_rect() -> Dict[str, list]:
    # 100km x 40km；A1-A6 为边界，A7-A8 为白方初始线
    return {
        "A1": [0, 0], "A2": [0, 20000], "A3": [0, 40000],
        "A4": [100000, 40000], "A5": [100000, 20000], "A6": [100000, 0],
        "A7": [0, 0], "A8": [100000, 0]
    }


def _write_black_plan(path: str, x: float = 50000.0, y: float = 30000.0, speed: float = 0.0, n: int = 1) -> str:
    data = {}
    for i in range(1, n + 1):
        data[f"usv{i}"] = [{"point": [x, y], "speed": speed}]
    p = os.path.join(path, "black.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    return p


class SimBasicsTests(unittest.TestCase):
    def test_freeze_keyerror_not_raise(self):
        area = _area_rect()
        sim = Simulator(area=area, dt=10, duration=100, seed=1)
        sim.init_black(3)
        logs = sim.run()
        self.assertIn("0", logs)
        self.assertIsInstance(logs["0"], dict)

    def test_run_stream_json_complete(self):
        area = _area_rect()
        sim = Simulator(area=area, dt=10, duration=400, seed=2)
        sim.init_black(2)
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "out.json")
            with open(p, "w", encoding="utf-8") as fp:
                sim.run_stream(fp)
            with open(p, "r", encoding="utf-8") as fp:
                data = json.load(fp)
            self.assertIn("0", data)

    def test_uav_on_deck_follows_host(self):
        area = _area_rect()
        sim = Simulator(area=area, dt=10, duration=1800, seed=3, uav_autolaunch=False)
        sim.init_black(1)
        sim.uavs["w_uav1"].energy = 0.0  # 明确在甲板充电
        logs = sim.run()
        aligned = False
        for t, frame in logs.items():
            u = frame["white"].get("w_uav1")
            if not u or u.get("state") != "补能":
                continue
            upos = tuple(u.get("pos", [None, None])[:2])
            for name, rec in frame["white"].items():
                if rec.get("type") == "无人艇":
                    if tuple(rec.get("pos", [None, None])[:2]) == upos:
                        aligned = True
                        break
            if aligned:
                break
        self.assertTrue(aligned, "UAV 在补能时未与任一白艇同位移动")

    def test_charge_rate(self):
        # 充电 1800s 应约等于 10%（5h=18000s 充满）
        # 为避免黑方锁定导致的冻结暂停充电，这里禁用锁定成功（p=0）且不初始化黑方
        area = _area_rect()
        sim = Simulator(area=area, dt=10, duration=1800, seed=4, uav_autolaunch=False, lock_success_prob=0.0)
        sim.uavs["w_uav1"].energy = 0.0
        _ = sim.run()
        soc = sim.metrics["per_uav"]["w_uav1"]["last_soc"]
        self.assertTrue(0.09 <= soc <= 0.11, f"expected ~0.10 after 1800s charge, got {soc}")

    def test_two_locks_cause_exit_when_p1(self):
        # dt 太大/起飞延迟会让 2 次锁定来不及完成；设置更细步长 + 起飞无延迟
        area = _area_rect()
        with tempfile.TemporaryDirectory() as td:
            plan = _write_black_plan(td, x=50000.0, y=30000.0, speed=0.0, n=1)
            sim = Simulator(area=area, dt=60, duration=900, seed=5,
                            lock_success_prob=1.0, uav_autolaunch=True, uav_autolaunch_delay=0.0)
            sim.init_black_from_plan(plan)
            _ = sim.run()
            pb = list(sim.metrics["per_black"].values())[0]
            self.assertEqual(pb["exit_reason"], "intercepted")


if __name__ == "__main__":
    unittest.main(verbosity=2)
