#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
白/黑双方规则驱动仿真（10s 一帧，logDemo 结构不变 + 指标/测试 + 邻域加速 + 任务分配 + 可见性约束 + 红线前沿布防 + UAV巡航/返航 + 大时长流式写出）。
- 仅优化“白方策略与仿真”，黑方策略由外部输入（--black-plan）或随机撒点。
- 关键保证：白方只基于物理探测（白USV 20km圆、白UAV 60km ±30°扇形）进行分配/追击/锁定；无探测时白USV回到/前往红线前沿布防位。
- UAV 状态机：patrol→rtb→charge，返航使用“甲板预留”避免冲突；补能满后自动起飞；不再出现“来回传送”；**在甲板上时 UAV 始终与宿主 USV 同步位姿**。
- 充电速率与题面一致：5h 充满 → 每秒充电量 = dt/18000.0（1小时≈20%）。
- 输出 JSON 严格保持题方的 logDemo 结构；指标写入独立 metrics.json；提供最小单元测试。
- 新增 --stream：流式写出长时长日志，避免内存暴涨/报错。

用法：
  python uav_rules_sim.py \
    --area ./任务区域.json \
    --black-plan ./方案1-黑方艇数量_3.json \
    --dt 10 --duration 36000 \
    --assign-period 60 --assign-horizon 3600 --guard-offset-km 5 \
    --uav-cruise-speed 120 \
    --stream \
    --outfile ./logDemo_out.json --metrics ./metrics.json --seed 42

测试：
  python uav_rules_sim.py --run-tests
"""
import argparse, json, math, os, random, sys, tempfile, unittest, io
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union, Iterable, Set
from collections import defaultdict

Vec2 = Tuple[float, float]

# ---- 小工具 ----

def to_vec2(a: Sequence[float]) -> Vec2:
    return (float(a[0]), float(a[1]))

def dot(a: Vec2, b: Vec2) -> float: return a[0]*b[0] + a[1]*b[1]

def sub(a: Vec2, b: Vec2) -> Vec2: return (a[0]-b[0], a[1]-b[1])

def add(a: Vec2, b: Vec2) -> Vec2: return (a[0]+b[0], a[1]+b[1])

def mul(a: Vec2, s: float) -> Vec2: return (a[0]*s, a[1]*s)

def norm(a: Vec2) -> float: return math.hypot(a[0], a[1])

def unit(a: Vec2) -> Vec2:
    L = norm(a)
    return (a[0]/L, a[1]/L) if L > 1e-9 else (1.0, 0.0)

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def closest_on_seg(p: Vec2, a: Vec2, b: Vec2) -> Vec2:
    ap, ab = sub(p, a), sub(b, a)
    L2 = dot(ab, ab)
    t = 0.0 if L2 <= 0 else max(0.0, min(1.0, dot(ap, ab)/L2))
    return add(a, mul(ab, t))

def point_in_poly(pt: Vec2, poly: List[Vec2]) -> bool:
    x, y = pt; inside = False; n = len(poly)
    for i in range(n):
        x1,y1 = poly[i]; x2,y2 = poly[(i+1)%n]
        if ((y1 > y) != (y2 > y)):
            x_int = (x2-x1) * (y-y1) / (y2-y1 + 1e-12) + x1
            if x < x_int: inside = not inside
    return inside

def sample_in_poly(poly: List[Vec2], rng: random.Random) -> Vec2:
    xs=[p[0] for p in poly]; ys=[p[1] for p in poly]
    minx,maxx=min(xs),max(xs); miny,maxy=min(ys),max(ys)
    for _ in range(20000):
        p=(rng.uniform(minx,maxx), rng.uniform(miny,maxy))
        if point_in_poly(p, poly): return p
    return poly[0]

def seg_intersect(a1: Vec2, a2: Vec2, b1: Vec2, b2: Vec2) -> bool:
    def ccw(p1,p2,p3):
        return (p3[1]-p1[1])*(p2[0]-p1[0]) > (p2[1]-p1[1])*(p3[0]-p1[0])
    return (ccw(a1,b1,b2) != ccw(a2,b1,b2)) and (ccw(a1,a2,b1) != ccw(a1,a2,b2))

# ---- 二维网格哈希（邻域加速） ----

class Grid:
    def __init__(self, cell_size_m: float):
        self.cell = float(cell_size_m)
        self.map: Dict[Tuple[int,int], List[Tuple[str,str]]] = defaultdict(list)
    def _key(self, p: Vec2) -> Tuple[int,int]:
        return (int(math.floor(p[0]/self.cell)), int(math.floor(p[1]/self.cell)))
    def rebuild(self, usvs: Dict[str,'USV'], uavs: Dict[str,'UAV']):
        self.map.clear()
        for name,u in usvs.items():
            if u.exits: continue
            self.map[self._key(u.pos)].append(("usv", name))
        for name,v in uavs.items():
            if v.exits or v.on_deck: continue
            self.map[self._key(v.pos)].append(("uav", name))
    def neighbors(self, pos: Vec2, radius_cells: int) -> Iterable[Tuple[str,str]]:
        kx,ky = self._key(pos)
        for dx in range(-radius_cells, radius_cells+1):
            for dy in range(-radius_cells, radius_cells+1):
                yield from self.map.get((kx+dx, ky+dy), [])

# ---- 实体 ----

@dataclass
class USV:
    side: str; name: str; pos: Vec2; heading: float; speed: float
    rmin: float = 20.0; frozen_until: float = 0.0; exits: bool = False
    lock_target: Optional[str] = None; lock_timer: float = 0.0; lock_successes: int = 0

@dataclass
class UAV:
    side: str; name: str; pos: Vec2; heading: float; speed: float
    rmin: float = 100.0; energy: float = 1.0; on_deck: Optional[str] = None; exits: bool = False
    task: str = "patrol"            # patrol | rtb | charge(=on_deck)
    rtb_host: Optional[str] = None   # 返航目标甲板

Entity = Union[USV, UAV]

# ---- 仿真器 ----

class Simulator:
    def __init__(self, area: Dict[str, List[float]], dt: float, duration: float, seed: int,
                 lock_success_prob: float = 0.8, uav_land_soc: float = 0.2, grid_cell_km: float = 5.0,
                 assign_period: float = 60.0, assign_horizon: float = 3600.0, guard_offset_km: float = 5.0,
                 uav_cruise_speed: float = 120.0,
                 uav_autolaunch: bool = True,
                 uav_autolaunch_delay: float = 30.0):
        self.dt = dt; self.duration = duration; self.t = 0.0
        self.rng = random.Random(seed)
        self.lock_success_prob = lock_success_prob
        self.uav_land_soc = uav_land_soc
        self.uav_cruise_speed = uav_cruise_speed
        self.uav_autolaunch = uav_autolaunch
        self.uav_autolaunch_delay = uav_autolaunch_delay
        self.A = {f"A{i}": to_vec2(area[f"A{i}"]) for i in range(1,9)}
        self.poly = [self.A[f"A{i}"] for i in range(1,9)]
        self.red1, self.red2 = self.A["A1"], self.A["A6"]
        self.usvs: Dict[str, USV] = {}; self.uavs: Dict[str, UAV] = {}
        # 甲板占用（已落舰）与预留（返航在途）分离
        self.deck_busy: Dict[str, Optional[str]] = {}
        self.deck_reserve: Dict[str, Optional[str]] = {}
        self.metrics = {
            "intercepted_black": 0,
            "intercept_times": [],
            "white_lock_success": 0,
            "black_lock_success": 0,
            "white_locked_count": 0,
            "collisions": 0,
            "black_penetrations": 0,
            "per_black": {},
            "per_white_usv": {},
            "per_uav": {},
            "timeline": []
        }
        # 网格
        self.grid = Grid(max(100.0, grid_cell_km*1000.0))
        self.grid_cell_km = grid_cell_km
        # 任务分配
        self.assign_period = assign_period
        self.assign_horizon = assign_horizon
        self.next_assign_time = 0.0
        self.assignments: Dict[str, Optional[str]] = {}  # w_usv -> b_usv or None
        # 红线布防
        self.guard_offset_m = max(0.0, guard_offset_km*1000.0)
        self.white_guard: Dict[str, Vec2] = {}
        # UAV 巡航
        self.uav_scan_state: Dict[str, Dict[str, object]] = {}  # {name: {idx:int, waypoints:List[Vec2]}}
        # 感知缓存
        self.sensed_by_uav: Set[str] = set()
        self.sensed_by_white: Set[str] = set()
        self.uav_detect_map: Dict[str, List[str]] = {}
        # 白艇驻位
        self.white_home: Dict[str, Vec2] = {}
        self.init_white()
        self._build_redline_posts_and_scans()

    # ---- 初始化 ----
    def init_white(self):
        A7,A8 = self.A["A7"], self.A["A8"]; mid = ((A7[0]+A8[0])/2, (A7[1]+A8[1])/2)
        dir78 = unit(sub(A8,A7)); normal = unit((-dir78[1], dir78[0]))
        for i,k in enumerate([-2,-1,0,1,2], start=1):
            p = add(mid, mul(dir78, k*5000.0)); name=f"w_usv{i}"
            self.usvs[name] = USV("白", name, p, math.atan2(normal[1], normal[0]), 10.0)
            self.white_home[name] = p
            self.deck_busy[name] = None
            self.deck_reserve[name] = None
            self.metrics["per_white_usv"][name] = {"locks_started":0, "locks_success":0, "frozen_segments": []}
            self.assignments[name] = None
        for i in range(1,6):
            host=f"w_usv{i}"; name=f"w_uav{i}"; pos=self.usvs[host].pos
            self.uavs[name] = UAV("白", name, pos, self.usvs[host].heading, 0.0, on_deck=host, task="charge")
            self.deck_busy[host] = name
            self.metrics["per_uav"][name] = {"air_seconds":0, "recharge_seconds":0, "cycles":0, "last_soc":1.0}

    def init_black(self, n: int):
        trap = [self.A["A2"], self.A["A3"], self.A["A4"], self.A["A5"]]
        for i in range(1, n+1):
            p = sample_in_poly(trap, self.rng); to_red = unit(sub(closest_on_seg(p, self.red1, self.red2), p))
            name=f"b_usv{i}"; self.usvs[name] = USV("黑", name, p, math.atan2(to_red[1], to_red[0]), 10.0)
            self.metrics["per_black"][name] = {"t_first_detect_by_uav":None,"t_first_detect_by_usv":None,
                                                "t_first_lock_start":None,"locks_success":0,"t_exit":None,"exit_reason":None}

    def init_black_from_plan(self, path: str):
        with open(path, "r", encoding="utf-8") as f: data = json.load(f)
        idx = 1
        for k, seq in data.items():
            if not (isinstance(seq, list) and k.startswith("usv")) or not seq: continue
            first = seq[0]; pt = to_vec2(first.get("point", (0.0,0.0))); speed = float(first.get("speed", 10.0))
            if len(seq) >= 2 and "point" in seq[1]: p1 = to_vec2(seq[1]["point"]); ddir = unit(sub(p1, pt))
            else: ddir = unit(sub(closest_on_seg(pt, self.red1, self.red2), pt))
            name=f"b_usv{idx}"; self.usvs[name] = USV("黑", name, pt, math.atan2(ddir[1], ddir[0]), speed)
            self.metrics["per_black"][name] = {"t_first_detect_by_uav":None,"t_first_detect_by_usv":None,
                                                "t_first_lock_start":None,"locks_success":0,"t_exit":None,"exit_reason":None}
            idx += 1
        if idx == 1:
            raise ValueError("black-plan 文件中未找到任何 usv* 列表")

    # ---- 红线布防与UAV巡航点 ----
    def _build_redline_posts_and_scans(self):
        A1, A6 = self.A["A1"], self.A["A6"]
        red_vec = sub(A6, A1)
        L = max(1.0, norm(red_vec))
        u_t = (red_vec[0]/L, red_vec[1]/L)
        cen = (sum(p[0] for p in self.poly)/len(self.poly), sum(p[1] for p in self.poly)/len(self.poly))
        n1 = (-u_t[1], u_t[0])
        if dot(sub(cen, A1), n1) < 0:  # 指向区域内部
            n1 = (-n1[0], -n1[1])
        posts: List[Vec2] = []
        for i in range(1, 6):
            t = (i - 0.5)/5.0
            base = add(A1, mul(u_t, L*t))
            post = add(base, mul(n1, self.guard_offset_m))
            posts.append(post)
        for i in range(1, 6):
            self.white_guard[f"w_usv{i}"] = posts[i-1]
        gap = L/5.0
        for i in range(1, 6):
            base = posts[i-1]
            wp1 = add(base, mul(u_t,  0.25*gap))
            wp2 = add(base, mul(u_t, -0.25*gap))
            self.uav_scan_state[f"w_uav{i}"] = {"idx": 0, "waypoints": [wp1, wp2]}

    # ---- 基本功能 ----
    def inside_area(self, p: Vec2) -> bool: return point_in_poly(p, self.poly)

    def steer_towards(self, heading: float, desired_dir: Vec2, vmax_turn: float) -> float:
        target = math.atan2(desired_dir[1], desired_dir[0]); d = (target - heading + math.pi) % (2*math.pi) - math.pi
        return heading + clamp(d, -vmax_turn, vmax_turn)

    def clamp_to_area(self, p: Vec2) -> Vec2:
        if self.inside_area(p): return p
        cx = sum(v[0] for v in self.poly)/len(self.poly); cy = sum(v[1] for v in self.poly)/len(self.poly)
        q = p; r = (q[0], q[1])
        for _ in range(32):
            mid = ((q[0]+cx)/2, (q[1]+cy)/2)
            if self.inside_area(mid): q = mid
            else: r = mid
        return q if self.inside_area(q) else r

    # ---- 感知（仅基于物理探测） ----
    def _nearby_usvs(self, pos: Vec2, max_range: float) -> Iterable['USV']:
        r = int(math.ceil(max_range / self.grid.cell))
        seen = set()
        for typ,name in self.grid.neighbors(pos, r):
            if typ != 'usv' or name in seen: continue
            u = self.usvs[name]
            if u.exits: continue
            if norm(sub(u.pos, pos)) <= max_range:
                seen.add(name); yield u

    def _nearby_uavs(self, pos: Vec2, max_range: float) -> Iterable['UAV']:
        r = int(math.ceil(max_range / self.grid.cell))
        seen = set()
        for typ,name in self.grid.neighbors(pos, r):
            if typ != 'uav' or name in seen: continue
            v = self.uavs[name]
            if v.exits or v.on_deck: continue
            if norm(sub(v.pos, pos)) <= max_range:
                seen.add(name); yield v

    def detect_black_by_white_usv(self, w: USV, b: USV) -> bool: return norm(sub(b.pos, w.pos)) <= 20000.0
    def detect_white_by_black_usv(self, b: USV, w: USV) -> bool: return norm(sub(w.pos, b.pos)) <= 30000.0
    def detect_black_by_white_uav(self, u: UAV, b: USV) -> bool:
        if u.on_deck: return False
        vec = sub(b.pos, u.pos); d = norm(vec)
        if d > 60000.0: return False
        fwd = (math.cos(u.heading), math.sin(u.heading))
        ok = dot(unit(vec), fwd) >= math.cos(math.radians(30.0))
        if ok and self.metrics["per_black"][b.name]["t_first_detect_by_uav"] is None:
            self.metrics["per_black"][b.name]["t_first_detect_by_uav"] = self.t
        return ok

    def update_sensing(self):
        # 为所有仍在场的黑方预建键，避免冻结期 KeyError
        blacks_all = [x for x in self.usvs.values() if x.side == '黑' and (not x.exits)]
        whites_active = [x for x in self.usvs.values() if x.side == '白' and (not x.exits) and self.t >= x.frozen_until]

        self.sensed_by_uav = set()
        self.sensed_by_white = set()
        self.uav_detect_map = {b.name: [] for b in blacks_all}

        # UAV 探测（对黑方是否冻结不敏感）
        for u in self.uavs.values():
            if u.exits or u.on_deck:
                continue
            for b in self._nearby_usvs(u.pos, 60000.0):
                if b.side != '黑':
                    continue
                if self.detect_black_by_white_uav(u, b):
                    self.sensed_by_uav.add(b.name)
                    self.uav_detect_map.setdefault(b.name, []).append(u.name)  # 兜底
                    if self.metrics["per_black"][b.name]["t_first_detect_by_uav"] is None:
                        self.metrics["per_black"][b.name]["t_first_detect_by_uav"] = self.t

        # 白 USV 探测（冻结时无探测能力）
        for w in whites_active:
            for b in self._nearby_usvs(w.pos, 20000.0):
                if b.side != '黑':
                    continue
                if self.detect_black_by_white_usv(w, b):
                    self.sensed_by_white.add(b.name)
                    if self.metrics["per_black"][b.name]["t_first_detect_by_usv"] is None:
                        self.metrics["per_black"][b.name]["t_first_detect_by_usv"] = self.t


    # ---- 任务分配与拦截几何（近似） ----
    def _vel_vec(self, u: USV) -> Vec2:
        return (math.cos(u.heading)*u.speed, math.sin(u.heading)*u.speed)

    def _eta_to_lock40(self, w: USV, b: USV) -> float:
        d = norm(sub(b.pos, w.pos))
        if d <= 40000.0: return 0.0
        u_bw = unit(sub(w.pos, b.pos))
        vb = self._vel_vec(b)
        vb_norm = norm(vb)
        cos_th = dot(unit(vb) if vb_norm>1e-6 else (0.0,0.0), u_bw)
        v_rel = 10.0 + max(0.0, 10.0 * cos_th)
        v_rel = max(1.0, v_rel)
        return (d - 40000.0) / v_rel

    def _assign_tasks(self):
        if self.t < self.next_assign_time: return
        self.next_assign_time = self.t + self.assign_period
        whites = [w for w in self.usvs.values() if w.side=='白' and (not w.exits) and self.t>=w.frozen_until]
        blacks = [b for b in self.usvs.values() if b.side=='黑' and (not b.exits) and self.t>=b.frozen_until and (b.name in self.sensed_by_uav or b.name in self.sensed_by_white)]
        for w in whites:
            tgt = self.assignments.get(w.name)
            if tgt and (tgt not in self.usvs or self.usvs[tgt].exits):
                self.assignments[w.name] = None
        triples = []  # (cost, w_name, b_name)
        for w in whites:
            for b in blacks:
                c = self._eta_to_lock40(w, b)
                if c <= self.assign_horizon:
                    triples.append((c, w.name, b.name))
        triples.sort(key=lambda x: x[0])
        used_w, used_b = set(), set(); new_assign = {}
        for c, wname, bname in triples:
            if wname in used_w or bname in used_b: continue
            new_assign[wname] = bname
            used_w.add(wname); used_b.add(bname)
        for w in whites:
            old = self.assignments.get(w.name)
            new = new_assign.get(w.name, None)
            if new != old:
                self.assignments[w.name] = new
                self.metrics["timeline"].append({"t": self.t, "event": "assign", "actors": [w.name, new] if new else [w.name]})

    # ---- 辅助：在甲板上 UAV 与宿主同步 ----
    def _sync_uavs_on_deck(self):
        for u in self.uavs.values():
            if u.on_deck:
                host = self.usvs.get(u.on_deck)
                if host:
                    u.pos = host.pos
                    u.heading = host.heading
                    u.speed = 0.0

    # ---- 运动学 ----
    def step_kinematics(self, ent: Entity):
        if isinstance(ent, USV) and (ent.exits or self.t < ent.frozen_until): return
        if isinstance(ent, UAV) and ent.exits: return
        dt = self.dt
        # 在甲板上：每帧硬同步位置/朝向
        if isinstance(ent, UAV) and ent.on_deck:
            ent.task = "charge"
            host = self.usvs.get(ent.on_deck)
            if host is not None:
                ent.pos = host.pos
                ent.heading = host.heading
            ent.speed = 0.0
            return
        if isinstance(ent, USV):
            if ent.side == "黑":
                goal = closest_on_seg(ent.pos, self.red1, self.red2); ddir = unit(sub(goal, ent.pos))
            else:
                target_name = self.assignments.get(ent.name)
                target = self.usvs.get(target_name) if target_name else None
                if target and (not target.exits):
                    eta = self._eta_to_lock40(ent, target)
                    lead = min(eta, 120.0)
                    vb = self._vel_vec(target)
                    pred = add(target.pos, mul(vb, lead))
                    ddir = unit(sub(pred, ent.pos))
                else:
                    guard = self.white_guard.get(ent.name, self.white_home.get(ent.name, ent.pos))
                    ddir = unit(sub(guard, ent.pos))
        else:
            if ent.on_deck:
                ent.task = "charge"; return
            # UAV 任务驱动：优先 RTB
            if ent.task == "rtb":
                host_name = ent.rtb_host
                if (not host_name) or (host_name not in self.usvs) or self.usvs[host_name].exits:
                    options = [uname for uname in self.usvs if self.usvs[uname].side=='白' and (not self.usvs[uname].exits) and self.deck_busy.get(uname) is None and self.deck_reserve.get(uname) in (None, ent.name)]
                    if options:
                        host_name = min(options, key=lambda uname: norm(sub(self.usvs[uname].pos, ent.pos)))
                        ent.rtb_host = host_name; self.deck_reserve[host_name] = ent.name
                if host_name:
                    host = self.usvs[host_name]
                    if norm(sub(host.pos, ent.pos)) < 100.0:
                        ent.speed = 20.0; ent.on_deck = host_name; ent.task = "charge"; ent.pos = host.pos
                        self.deck_busy[host_name] = ent.name; self.deck_reserve[host_name] = None
                        return
                    ddir = unit(sub(host.pos, ent.pos))
                else:
                    ddir = (math.cos(ent.heading), math.sin(ent.heading))
            else:
                candidate_names = list(self.sensed_by_uav | self.sensed_by_white)
                if candidate_names:
                    cand = min((self.usvs[n] for n in candidate_names if (n in self.usvs and (not self.usvs[n].exits))), key=lambda b: norm(sub(b.pos, ent.pos)), default=None)
                    if cand:
                        ddir = unit(sub(cand.pos, ent.pos))
                    else:
                        ddir = (math.cos(ent.heading), math.sin(ent.heading))
                else:
                    st = self.uav_scan_state.get(ent.name)
                    if st:
                        wps: List[Vec2] = st["waypoints"]  # type: ignore
                        idx: int = st["idx"]  # type: ignore
                        tgt = wps[idx]
                        if norm(sub(tgt, ent.pos)) < 2000.0:
                            st["idx"] = 1-idx; tgt = wps[st["idx"]]
                        ddir = unit(sub(tgt, ent.pos))
                    else:
                        ddir = (math.cos(ent.heading), math.sin(ent.heading))
        vmax_turn = (ent.speed * dt) / ent.rmin
        ent.heading = self.steer_towards(ent.heading, ddir, vmax_turn)
        vdir = (math.cos(ent.heading), math.sin(ent.heading))
        ent.pos = add(ent.pos, mul(vdir, ent.speed * dt))
        if not self.inside_area(ent.pos): ent.pos = self.clamp_to_area(ent.pos)

    # ---- 锁定与能量 ----
    def update_locks(self):
        dt = self.dt
        for u in self.usvs.values():
            if u.lock_target and not u.exits:
                tgt = self.usvs.get(u.lock_target)
                if (not tgt) or tgt.exits or norm(sub(tgt.pos, u.pos)) > 40000.0:
                    u.lock_target = None; u.lock_timer = 0.0
        whites = [x for x in self.usvs.values() if x.side=="白" and not x.exits and self.t>=x.frozen_until]
        blacks = [x for x in self.usvs.values() if x.side=="黑" and not x.exits and self.t>=x.frozen_until]
        uav_detect = self.uav_detect_map
        for w in whites:
            if w.lock_target:
                w.lock_timer += dt
                if w.lock_timer >= 300.0:
                    if self.rng.random() < self.lock_success_prob:
                        tgt = self.usvs.get(w.lock_target)
                        if tgt and not tgt.exits:
                            tgt.frozen_until = max(tgt.frozen_until, self.t + 300.0)
                            tgt.lock_successes += 1; self.metrics["white_lock_success"] += 1
                            if tgt.lock_successes >= 2:
                                tgt.exits = True
                                if tgt.side == "黑":
                                    self.metrics["intercepted_black"] += 1; self.metrics["intercept_times"].append(self.t)
                                    pb = self.metrics["per_black"][tgt.name]; pb["t_exit"]=self.t; pb["exit_reason"]="intercepted"
                    w.lock_target = None; w.lock_timer = 0.0
                continue
            cands: List[USV] = []
            for b in blacks:
                if b.name not in self.sensed_by_uav and b.name not in self.sensed_by_white: continue
                if norm(sub(b.pos, w.pos)) <= 40000.0 and ((b.name in self.sensed_by_white) or uav_detect.get(b.name)):
                    cands.append(b)
                    if (b.name in self.sensed_by_white) and (self.metrics["per_black"][b.name]["t_first_detect_by_usv"] is None):
                        self.metrics["per_black"][b.name]["t_first_detect_by_usv"] = self.t
            if cands:
                b = min(cands, key=lambda x: norm(sub(x.pos, w.pos)))
                w.lock_target = b.name; w.lock_timer = 0.0
                self.metrics["per_white_usv"][w.name]["locks_started"] += 1
                if self.metrics["per_black"][b.name]["t_first_lock_start"] is None:
                    self.metrics["per_black"][b.name]["t_first_lock_start"] = self.t
                self.metrics["timeline"].append({"t":self.t,"event":"lock_start","actors":[w.name,b.name]})
        for b in blacks:
            if b.lock_target:
                b.lock_timer += dt
                if b.lock_timer >= 300.0:
                    if self.rng.random() < self.lock_success_prob:
                        tgt = self.usvs.get(b.lock_target)
                        if tgt and not tgt.exits:
                            tgt.frozen_until = max(tgt.frozen_until, self.t + 300.0)
                            tgt.lock_successes += 1; self.metrics["black_lock_success"] += 1
                            if tgt.lock_successes >= 2:
                                tgt.exits = True; self.metrics["white_locked_count"] += 1
                                self.metrics["timeline"].append({"t":self.t,"event":"white_usv_exit","actors":[tgt.name]})
                    b.lock_target = None; b.lock_timer = 0.0
                continue
            cands: List[USV] = []
            for w in whites:
                if norm(sub(w.pos, b.pos)) <= 40000.0 and self.detect_white_by_black_usv(b, w):
                    cands.append(w)
            if cands:
                w = min(cands, key=lambda x: norm(sub(x.pos, b.pos)))
                b.lock_target = w.name; b.lock_timer = 0.0

    def update_uav_energy(self):
        dt = self.dt
        for name,u in self.uavs.items():
            if u.exits: continue
            if u.on_deck:
                host = self.usvs.get(u.on_deck)
                if (not host) or host.exits:
                    u.exits = True; continue
                # 在甲板期间，持续同步位置/朝向，并保持速度为0
                u.pos = host.pos; u.heading = host.heading; u.speed = 0.0
                if self.t < host.frozen_until:  # 冻结时补能暂停
                    u.task = "charge"; continue
                u.energy = clamp(u.energy + dt/18000.0, 0.0, 1.0)
                u.task = "charge"
                self.metrics["per_uav"][name]["recharge_seconds"] += dt
                self.metrics["per_uav"][name]["last_soc"] = u.energy
                # 自动起飞：满足能量且过了自启动延迟，不再依赖“黑方是否存在”
                if self.uav_autolaunch and (self.t >= self.uav_autolaunch_delay) and (u.energy >= 0.99):
                    host_name = u.on_deck
                    u.on_deck = None; u.task = "patrol"; u.rtb_host = None
                    if host_name in self.deck_busy and self.deck_busy[host_name] == name:
                        self.deck_busy[host_name] = None
                    self.metrics["per_uav"][name]["cycles"] += 1
                    u.speed = max(u.speed, self.uav_cruise_speed)
            else:
                u.energy = clamp(u.energy - dt/7200.0, 0.0, 1.0)
                self.metrics["per_uav"][name]["air_seconds"] += dt
                self.metrics["per_uav"][name]["last_soc"] = u.energy
                if u.energy <= self.uav_land_soc and u.task != "rtb":
                    options = [uname for uname in self.usvs if self.usvs[uname].side=='白' and (not self.usvs[uname].exits) and self.deck_busy.get(uname) is None and self.deck_reserve.get(uname) in (None, name)]
                    if options:
                        host = min(options, key=lambda uname: norm(sub(self.usvs[uname].pos, u.pos)))
                        self.deck_reserve[host] = name; u.rtb_host = host; u.task = "rtb"
                for k in list(self.deck_busy.keys()):
                    if self.deck_busy[k] == name and u.on_deck != k:
                        self.deck_busy[k] = None

    # ---- 日志 ----
    def make_frame(self) -> Dict:
        frame = {"white": {}, "black": {}}
        for name,u in self.usvs.items():
            rec = {"pos":[round(u.pos[0],3), round(u.pos[1],3), 0], "type":"无人艇", "group":u.side,
                   "state": ("退出" if u.exits else ("冻结" if self.t < u.frozen_until else ("锁定中" if u.lock_target else "移动"))),
                   "velocity":[round(math.cos(u.heading)*u.speed,3), round(math.sin(u.heading)*u.speed,3), 0]}
            (frame["white"] if u.side=="白" else frame["black"])[name] = rec
        for name,v in self.uavs.items():
            rec = {"pos":[round(v.pos[0],3), round(v.pos[1],3), 0], "type":"无人机", "group":"白",
                   "state": ("退出" if v.exits else ("补能" if v.on_deck else ("返航" if v.task=="rtb" else "滞空"))),
                   "velocity":[round(math.cos(v.heading)*v.speed,3), round(math.sin(v.heading)*v.speed,3), 0]}
            frame["white"][name] = rec
        return frame

    # ---- 主回路（一次性内存版） ----
    def run(self) -> Dict[str, Dict]:
        logs: Dict[str, Dict] = {}; last_pos: Dict[str, Vec2] = {n:u.pos for n,u in self.usvs.items()}
        self.grid.rebuild(self.usvs, self.uavs)
        self._sync_uavs_on_deck()
        steps = int(self.duration / self.dt) + 1
        for _ in range(steps):
            self._assign_tasks()
            for u in list(self.usvs.values()): self.step_kinematics(u)
            for v in list(self.uavs.values()): self.step_kinematics(v)
            self.grid.rebuild(self.usvs, self.uavs)
            self.update_sensing()
            self.update_uav_energy()
            self.update_locks()
            seen_pairs = set()
            for name,u in self.usvs.items():
                if u.exits: continue
                for v in self._nearby_usvs(u.pos, 400.0):
                    if v.name <= name: continue
                    key = (name, v.name)
                    if key in seen_pairs: continue
                    if norm(sub(u.pos, v.pos)) < 100.0:
                        self.metrics["collisions"] += 1
                        self.metrics["timeline"].append({"t":self.t,"event":"collision","actors":[u.name,v.name]})
                    seen_pairs.add(key)
            for name,u in self.usvs.items():
                if u.side=="黑" and (not u.exits):
                    if seg_intersect(last_pos[name], u.pos, self.red1, self.red2):
                        u.exits = True; self.metrics["black_penetrations"] += 1
                        pb = self.metrics["per_black"][name]; pb["t_exit"]=self.t; pb["exit_reason"]="penetration"
                        self.metrics["timeline"].append({"t":self.t,"event":"penetration","actors":[name]})
            key = str(int(self.t)); logs[key] = self.make_frame()
            for name,u in self.usvs.items(): last_pos[name] = u.pos
            self.t += self.dt
        return logs

    def run_stream(self, fp: io.TextIOBase) -> None:
        self.grid.rebuild(self.usvs, self.uavs)
        self._sync_uavs_on_deck()
        steps = int(self.duration / self.dt) + 1

        fp.write('{')
        first = True
        last_pos: Dict[str, Vec2] = {n: u.pos for n, u in self.usvs.items()}

        try:
            for _ in range(steps):
                self._assign_tasks()
                for u in list(self.usvs.values()):
                    self.step_kinematics(u)
                for v in list(self.uavs.values()):
                    self.step_kinematics(v)

                self.grid.rebuild(self.usvs, self.uavs)
                self.update_sensing()
                self.update_uav_energy()
                self.update_locks()

                # 碰撞检测
                seen_pairs = set()
                for name, u in self.usvs.items():
                    if u.exits:
                        continue
                    for v in self._nearby_usvs(u.pos, 400.0):
                        if v.name <= name:
                            continue
                        key = (name, v.name)
                        if key in seen_pairs:
                            continue
                        if norm(sub(u.pos, v.pos)) < 100.0:
                            self.metrics["collisions"] += 1
                            self.metrics["timeline"].append({"t": self.t, "event": "collision", "actors": [u.name, v.name]})
                        seen_pairs.add(key)

                # 突防判断
                for name, u in self.usvs.items():
                    if u.side == "黑" and (not u.exits):
                        if seg_intersect(last_pos[name], u.pos, self.red1, self.red2):
                            u.exits = True
                            self.metrics["black_penetrations"] += 1
                            pb = self.metrics["per_black"][name]
                            pb["t_exit"] = self.t
                            pb["exit_reason"] = "penetration"
                            self.metrics["timeline"].append({"t": self.t, "event": "penetration", "actors": [name]})

                # 流式输出当帧
                key = str(int(self.t))
                frame = self.make_frame()
                if not first:
                    fp.write(',\n')
                fp.write(json.dumps({key: frame}, ensure_ascii=False, indent=2)[1:-1])
                fp.flush()

                for name, u in self.usvs.items():
                    last_pos[name] = u.pos
                self.t += self.dt
                first = False
        finally:
            fp.write('\n}')
            fp.flush()

# ---- CLI / 测试 ----

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--area", default=None, help="任务区域.json；若省略则尝试 ./任务区域.json")
    p.add_argument("--dt", type=float, default=10.0, help="步长秒")
    p.add_argument("--duration", type=float, default=36000.0, help="总秒数（例如 10800=3h, 36000=10h）")
    p.add_argument("--outfile", default="./logDemo_out.json", help="日志输出")
    p.add_argument("--metrics", default="./metrics.json", help="指标输出")
    p.add_argument("--black-n", type=int, default=None, help="黑方数量（仅在未提供 --black-plan 时生效）")
    p.add_argument("--black-plan", type=str, default=None, help="黑方方案 JSON（usv*）")
    p.add_argument("--seed", type=int, default=42, help="随机种子")
    p.add_argument("--lock-success-prob", type=float, default=0.8, help="锁定成功概率")
    p.add_argument("--uav-land-soc", type=float, default=0.2, help="UAV 返航阈值 [0,1]")
    p.add_argument("--grid-cell-km", type=float, default=5.0, help="网格格长（km），影响邻域加速粒度")
    p.add_argument("--assign-period", type=float, default=60.0, help="任务分配周期（秒）")
    p.add_argument("--assign-horizon", type=float, default=3600.0, help="分配考虑的最大ETA（秒）")
    p.add_argument("--guard-offset-km", type=float, default=5.0, help="红线内侧布防偏置（km）")
    p.add_argument("--uav-cruise-speed", type=float, default=120.0, help="UAV 滞空/巡航速度（m/s）")
    p.add_argument("--uav-autolaunch", dest="uav_autolaunch", action="store_true", default=True, help="UAV 充满后自动起飞")
    p.add_argument("--no-uav-autolaunch", dest="uav_autolaunch", action="store_false")
    p.add_argument("--uav-autolaunch-delay", type=float, default=30.0, help="UAV 开始自动起飞的延迟（秒）")
    p.add_argument("--stream", action="store_true", help="大时长流式写出以节省内存")
    p.add_argument("--run-tests", action="store_true", help="运行内置测试")
    args = p.parse_args(argv)
    if not args.area:
        default_area = os.path.join(os.getcwd(), "任务区域.json")
        if os.path.exists(default_area): args.area = default_area
        else: p.error("the following arguments are required: --area (未找到默认 任务区域.json)")
    return args

def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if args.run_tests:
        return run_tests()
    with open(args.area, "r", encoding="utf-8") as f: area = json.load(f)
    sim = Simulator(area=area, dt=args.dt, duration=args.duration, seed=args.seed,
                    lock_success_prob=args.lock_success_prob, uav_land_soc=args.uav_land_soc,
                    grid_cell_km=args.grid_cell_km, assign_period=args.assign_period,
                    assign_horizon=args.assign_horizon, guard_offset_km=args.guard_offset_km,
                    uav_cruise_speed=args.uav_cruise_speed,
                    uav_autolaunch=args.uav_autolaunch, uav_autolaunch_delay=args.uav_autolaunch_delay)
    if args.black_plan: sim.init_black_from_plan(args.black_plan)
    else: sim.init_black(args.black_n if args.black_n is not None else 6)
    os.makedirs(os.path.dirname(args.outfile) or ".", exist_ok=True)
    if args.stream:
        with open(args.outfile, "w", encoding="utf-8") as fp:
            sim.run_stream(fp)
    else:
        logs = sim.run()
        with open(args.outfile, "w", encoding="utf-8") as f: json.dump(logs, f, ensure_ascii=False, indent=2)
    with open(args.metrics, "w", encoding="utf-8") as f: json.dump(sim.metrics, f, ensure_ascii=False, indent=2)
    print(f"[OK] 日志: {args.outfile}\n[OK] 指标: {args.metrics}")
    return 0

# ---- Tests ----
class _SimTests(unittest.TestCase):
    def _write_area(self, d: str) -> str:
        pts = {
            "A1":[0,0], "A2":[0,20000], "A3":[0,40000], "A4":[100000,40000],
            "A5":[100000,20000], "A6":[100000,0], "A7":[0,0], "A8":[100000,0]
        }
        p = os.path.join(d, "area.json"); json.dump(pts, open(p, "w"), ensure_ascii=False)
        return p
    def _write_black_plan(self, d: str, n: int=1) -> str:
        data = {}
        for i in range(1, n+1):
            data[f"usv{i}"] = [
                {"point":[5000*i, 30000], "speed": 10.0},
                {"point":[20000, 30000]}
            ]
        p = os.path.join(d, "black.json"); json.dump(data, open(p, "w"), ensure_ascii=False)
        return p
    def test_missing_area_errors(self):
        with self.assertRaises(SystemExit) as cm:
            parse_args([])
        self.assertEqual(cm.exception.code, 2)
    def test_min_run_with_plan(self):
        with tempfile.TemporaryDirectory() as td:
            area = self._write_area(td); plan = self._write_black_plan(td, n=2)
            out = os.path.join(td, "log.json"); met = os.path.join(td, "m.json")
            rc = main(["--area", area, "--black-plan", plan, "--dt", "10", "--duration", "100", "--outfile", out, "--metrics", met, "--seed", "1", "--lock-success-prob","1.0"])
            self.assertEqual(rc, 0)
            log = json.load(open(out)); metj = json.load(open(met))
            self.assertIn("0", log); self.assertIn("10", log)
            any_black = any(k.startswith("b_usv") for k in log["0"]["black"].keys())
            self.assertTrue(any_black)
            self.assertIn("black_penetrations", metj)
    def test_two_locks_cause_exit(self):
        with tempfile.TemporaryDirectory() as td:
            area = self._write_area(td); plan = self._write_black_plan(td, n=1)
            out = os.path.join(td, "log.json"); met = os.path.join(td, "m.json")
            rc = main(["--area", area, "--black-plan", plan, "--dt", "300", "--duration", "600", "--outfile", out, "--metrics", met, "--seed", "1", "--lock-success-prob","1.0"])
            self.assertEqual(rc, 0)
            metj = json.load(open(met))
            pb = list(metj["per_black"].values())[0]
            self.assertEqual(pb["exit_reason"], "intercepted")
    def test_visibility_constraint(self):
        with tempfile.TemporaryDirectory() as td:
            area = self._write_area(td); plan = self._write_black_plan(td, n=1)
            out = os.path.join(td, "log.json"); met = os.path.join(td, "m.json")
            rc = main(["--area", area, "--black-plan", plan, "--dt", "10", "--duration", "20", "--outfile", out, "--metrics", met, "--seed", "1"])
            self.assertEqual(rc, 0)
            log = json.load(open(out))
            p0 = log["0"]["white"]["w_usv3"]["pos"]
            p1 = log["10"]["white"]["w_usv3"]["pos"]
            self.assertLess(abs(p0[0]-p1[0]) + abs(p0[1]-p1[1]), 200.0)
    def test_uav_rtb_and_charge(self):
        # 高阈值，迅速进入返航并落舰补能
        with tempfile.TemporaryDirectory() as td:
            area = self._write_area(td); plan = self._write_black_plan(td, n=1)
            out = os.path.join(td, "log.json"); met = os.path.join(td, "m.json")
            rc = main(["--area", area, "--black-plan", plan, "--dt", "10", "--duration", "2000", "--uav-land-soc", "0.9999", "--outfile", out, "--metrics", met, "--seed", "2"])
            self.assertEqual(rc, 0)
            with open(out, "r", encoding="utf-8") as f:
                j = json.load(f)
            states = [frame["white"].get("w_uav1", {}).get("state") for t,frame in j.items() if "white" in frame]
            self.assertIn("补能", states)
    def test_uav_on_deck_follows_host(self):
        with tempfile.TemporaryDirectory() as td:
            area = self._write_area(td); plan = self._write_black_plan(td, n=1)
            out = os.path.join(td, "log.json"); met = os.path.join(td, "m.json")
            rc = main(["--area", area, "--black-plan", plan, "--dt", "10", "--duration", "2200", "--uav-land-soc", "0.9999", "--outfile", out, "--metrics", met, "--seed", "3"])
            self.assertEqual(rc, 0)
            j = json.load(open(out, "r", encoding="utf-8"))
            found = False
            for t,frame in j.items():
                white = frame.get("white", {})
                uav = white.get("w_uav1")
                if not uav or uav.get("state") != "补能":
                    continue
                p = tuple(uav.get("pos", [None, None])[:2])
                for k,v in white.items():
                    if v.get("type") == "无人艇":
                        q = tuple(v.get("pos", [None, None])[:2])
                        if p == q:
                            found = True
                            break
                if found:
                    break
            self.assertTrue(found, "UAV 在补能时未与任一白USV 同位移动")
    def test_stream_long_duration(self):
        # 流式写出可在较长时长下运行而不爆内存
        with tempfile.TemporaryDirectory() as td:
            area = self._write_area(td); plan = self._write_black_plan(td, n=2)
            out = os.path.join(td, "log.json"); met = os.path.join(td, "m.json")
            rc = main(["--area", area, "--black-plan", plan, "--dt", "10", "--duration", "36000", "--stream", "--outfile", out, "--metrics", met, "--seed", "5"])
            self.assertEqual(rc, 0)
            with open(out, "r", encoding="utf-8") as f:
                head = f.read(1)
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell()-1))
                # 简单结构校验
            self.assertEqual(head, '{')

def run_tests() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(_SimTests)
    res = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if res.wasSuccessful() else 1

if __name__ == "__main__":
    sys.exit(main())
