#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
log 回放可视化（matplotlib）。
HUD 包含：
- 在场数量（白USV/黑USV/UAV）
- 当前锁定中（白→黑 / 黑→白）  ← 逐帧实时
- 累计锁定成功（白锁黑 / 黑锁白）和首次突防时间（来自 metrics.json，可选）

用法示例：
  python viz_log.py --log ./logDemo_out.json --area ./任务区域.json --metrics ./metrics.json --fps 20
按 Q 暂停/继续。
"""
import argparse, json
from typing import Dict, List, Tuple, Optional
import math
from matplotlib import font_manager
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle

# ---- 中文字体（避免方块/乱码）----
def _setup_chinese_font():
    candidates = ["PingFang SC","Hiragino Sans GB","SimHei","STHeiti",
                  "Noto Sans CJK SC","Microsoft YaHei","Arial Unicode MS"]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            plt.rcParams["axes.unicode_minus"] = False
            return
_setup_chinese_font()

Vec2 = Tuple[float, float]

# --- 逐帧统计 ---
def compute_counts(frame: dict):
    white = frame.get('white', {}); black = frame.get('black', {})
    w_usv = sum(1 for v in white.values() if v.get('type')=='无人艇' and v.get('state')!='退出')
    b_usv = sum(1 for v in black.values() if v.get('type')=='无人艇' and v.get('state')!='退出')
    uav_air = sum(1 for v in white.values()
                  if v.get('type')=='无人机' and v.get('state') not in ('退出'))
    return w_usv, b_usv, uav_air

def compute_current_locks(frame: dict):
    """实时：白→黑 = 本帧中白方无人艇处于'锁定中'的数量；黑→白同理。"""
    white = frame.get('white', {}); black = frame.get('black', {})
    white_locking = sum(1 for v in white.values()
                        if v.get('type')=='无人艇' and v.get('state')=='锁定中')
    black_locking = sum(1 for v in black.values()
                        if v.get('type')=='无人艇' and v.get('state')=='锁定中')
    return white_locking, black_locking

def fmt_hms(sec: Optional[float]) -> str:
    if sec is None: return "—"
    try: sec = float(sec)
    except Exception: return "—"
    if sec < 0: return "—"
    h = int(sec // 3600); m = int((sec % 3600)//60); s = int(sec % 60)
    return f"{h:02d}:{m:02d}:{s:02d}"

# --- IO ---
def load_log(path: str):
    with open(path,'r',encoding='utf-8') as f:
        data = json.load(f)
    if not isinstance(data,dict) or not data:
        raise ValueError('日志为空或格式错误')
    keys = sorted(int(k) for k in data.keys())
    frames = [data[str(k)] for k in keys]
    dt = keys[1]-keys[0] if len(keys)>1 else 10
    return frames, dt, keys

def load_metrics(path: Optional[str]) -> Dict:
    base = {
        "white_locked_by_black": None,
        "black_locked_by_white": None,
        "t_first_penetration": None,
        "timeline": [],
    }
    if not path: return base
    try:
        with open(path,'r',encoding='utf-8') as f:
            m = json.load(f)
        base["white_locked_by_black"] = m.get("white_locked_by_black")
        base["black_locked_by_white"] = m.get("black_locked_by_white")
        base["t_first_penetration"] = m.get("t_first_penetration")
        base["timeline"] = m.get("timeline", [])
    except Exception:
        pass
    return base

# --- area ---
def load_area_poly(path: Optional[str]):
    if not path: return None
    with open(path,'r',encoding='utf-8') as f:
        a = json.load(f)
    try:
        return [tuple(a[f"A{i}"][:2]) for i in range(1,9)]
    except Exception:
        return None

def bounds_from_frames_and_poly(frames: List[Dict], poly: Optional[List[Vec2]]):
    xs, ys = [], []
    for fr in frames:
        for side in ("white","black"):
            for ent in fr.get(side, {}).values():
                p = ent.get("pos",[0,0])
                xs.append(float(p[0])); ys.append(float(p[1]))
    if poly:
        for p in poly:
            xs.append(float(p[0])); ys.append(float(p[1]))
    if not xs: return 0,1,0,1
    minx,maxx = min(xs),max(xs); miny,maxy = min(ys),max(ys)
    pad_x = (maxx-minx)*0.1 or 100; pad_y = (maxy-miny)*0.1 or 100
    return minx-pad_x, maxx+pad_x, miny-pad_y, maxy+pad_y

# --- style ---
def base_color(ent: Dict) -> str:
    return '#5fa4ff' if ent.get('group','白')=='白' else '#e86a6a'

def style(ent: Dict):
    etype = ent.get('type','无人艇'); state = ent.get('state','')
    marker = '^' if etype=='无人机' else 'o'
    size = 50 if etype=='无人机' else 30
    color = base_color(ent)
    alpha = 0.6 if state=='冻结' else 1.0
    if state == '补能':
        marker, size = 's', 45
    return color, marker, size, alpha

# --- main ---
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True, help='logDemo_out.json')
    ap.add_argument('--area', default=None, help='任务区域.json（可选）')
    ap.add_argument('--metrics', default=None, help='metrics.json（可选）')
    ap.add_argument('--fps', type=int, default=20, help='动画帧率')
    args = ap.parse_args()

    frames, sim_dt, keys = load_log(args.log)
    poly = load_area_poly(args.area)
    m = load_metrics(args.metrics)

    minx,maxx,miny,maxy = bounds_from_frames_and_poly(frames, poly)
    fig, ax = plt.subplots(figsize=(9,8))
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(minx, maxx); ax.set_ylim(miny, maxy)

    if poly:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.plot(xs, ys, lw=2, alpha=.6, label='任务区域')
        ax.plot([poly[0][0], poly[5][0]], [poly[0][1], poly[5][1]],
                'r--', lw=1.5, alpha=.8, label='红线 A1–A6')

    # 左上：时间/在场数量；右上：锁定 HUD（实时 + 累计）
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        va='top', ha='left', fontsize=11)
    stat_text = ax.text(0.98, 0.98, '', transform=ax.transAxes,
                        va='top', ha='right', fontsize=11)

    artists: Dict[str, Dict] = {}
    lock_rings: Dict[str, Circle] = {}

    def ensure_artist(name: str, ent: Dict):
        if name in artists: return artists[name]
        color, marker, size, alpha = style(ent)
        sc = ax.scatter([ent['pos'][0]],[ent['pos'][1]], s=size,
                        marker=marker, c=color, alpha=alpha, zorder=3)
        lbl = ax.text(ent['pos'][0], ent['pos'][1], name,
                      fontsize=8, color=color, zorder=4)
        artists[name] = {"sc": sc, "lbl": lbl}
        return artists[name]

    playing = True
    def on_key(e):
        nonlocal playing
        if e.key and e.key.lower() == 'q':
            playing = not playing
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(i):
        fr = frames[i]
        w_usv, b_usv, uav_air = compute_counts(fr)
        w_lock_now, b_lock_now = compute_current_locks(fr)

        time_text.set_text(
            f"t={keys[i]}s (dt={sim_dt}s)\n"
            f"白:{w_usv} 黑:{b_usv} UAV:{uav_air}"
        )

        # 累计（可无 metrics）
        wb = m.get("white_locked_by_black")
        bb = m.get("black_locked_by_white")
        t_first = m.get("t_first_penetration")

        stat_text.set_text(
            "白→黑:{w}  黑→白:{b}\n"
            "白锁黑:{bb}  黑锁白:{wb}\n"
            "首次突防: {t}".format(
                w=w_lock_now, b=b_lock_now,
                bb=("n/a" if bb is None else int(bb)),
                wb=("n/a" if wb is None else int(wb)),
                t=fmt_hms(t_first)
            )
        )

        present = set()
        # 先隐藏所有锁定环
        for ring in lock_rings.values():
            ring.set_visible(False)

        for side in ("white","black"):
            for name, ent in fr.get(side, {}).items():
                state = ent.get('state', '')
                if state in ('退出','突防成功'):
                    if name in artists:
                        artists[name]['sc'].set_visible(False)
                        artists[name]['lbl'].set_visible(False)
                    continue
                present.add(name)
                art = ensure_artist(name, ent)
                x,y = float(ent['pos'][0]), float(ent['pos'][1])
                color, marker, size, alpha = style(ent)
                art['sc'].set_offsets([[x,y]])
                art['sc'].set_color(color)
                art['sc'].set_sizes([size])
                art['sc']._alpha = alpha
                art['lbl'].set_position((x,y))
                art['lbl'].set_text(name)
                art['lbl'].set_color(color)
                art['sc'].set_visible(True); art['lbl'].set_visible(True)

                if state == '锁定中':
                    ring = lock_rings.get(name)
                    if ring is None:
                        ring = Circle((x,y), radius=800.0,
                                      fill=False, ec=color, lw=1.2, alpha=0.8, zorder=2)
                        ax.add_patch(ring); lock_rings[name] = ring
                    ring.center = (x,y); ring.set_visible(True)

        for name, art in artists.items():
            vis = name in present
            if not vis:
                art['sc'].set_visible(False)
                art['lbl'].set_visible(False)

        return [time_text, stat_text] + \
               [d['sc'] for d in artists.values()] + \
               [d['lbl'] for d in artists.values()] + \
               [r for r in lock_rings.values()]

    # 重要：持有 ani 引用，避免“Animation was deleted without rendering”警告
    ani = animation.FuncAnimation(fig, update, frames=len(frames),
                                  interval=int(1000/args.fps), blit=False, repeat=False)
    plt.title('log replay (Q 暂停/继续)')
    plt.legend(loc='lower right')
    plt.show()

if __name__ == '__main__':
    main()