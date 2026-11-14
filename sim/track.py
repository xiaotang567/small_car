# -*- coding: utf-8 -*-
"""赛道几何（可**自己设计**的地图）

设计文件：sim/track_design.json（没有就用内置默认跑道）
设计由三部分组成：

    path    一串**元素**，像开"乌龟"一样一段一段接出来（正是地图编辑器画的那些）
              {"t":"line","len":2.0}                 直线 2m
              {"t":"arc","r":1.3,"deg":45}            左转 45° 圆弧（半径 1.3m，deg 负数 = 右转）
              {"t":"arc","r":0.8,"deg":-90}           右转 90°
              {"t":"pts","p":[[x,y],...]}             直接给一串折线点（编辑器内部用）
    crosses 十字：{"x":3,"y":0,"yaw":1.5708,"half":1.3}  （yaw=支路方向，half=支路半长）
    rings   圆环：{"x":3,"y":6,"r_in":0.75,"r_out":1.35}

坐标系：x 东 / y 北 / z 上，单位米；赛道宽 = 2 × half_w。
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DESIGN_FILE = os.path.join(HERE, 'track_design.json')

GRID_RES = 0.02
GRID_PAD = 0.8

# ---------------- 内置默认跑道（和之前那圈一样）----------------
DEFAULT_DESIGN = {
    'half_w': 0.25,
    'start': [0.15, 0.0, 0.0],
    'path': [
        {'t': 'line', 'len': 6.0},
        {'t': 'arc', 'r': 1.3, 'deg': 90},
        {'t': 'line', 'len': 3.4},
        {'t': 'arc', 'r': 1.3, 'deg': 90},
        {'t': 'line', 'len': 6.0},
        {'t': 'arc', 'r': 1.3, 'deg': 90},
        {'t': 'line', 'len': 3.4},
        {'t': 'arc', 'r': 1.3, 'deg': 90},
    ],
    'crosses': [{'x': 3.0, 'y': 0.0, 'yaw': math.pi / 2, 'half': 1.3}],
    'rings': [{'x': 3.0, 'y': 6.0, 'r_in': 0.75, 'r_out': 1.35}],
}


def _arc_points(cx, cy, r, a0, a1, n=48):
    a = np.linspace(a0, a1, n)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def path_from_elements(elements, start_xy=(0.0, 0.0), start_yaw=0.0, arc_step_deg=5.0):
    """把元素序列走成一条折线（编辑器/设计文件共用）"""
    x, y = float(start_xy[0]), float(start_xy[1])
    yaw = float(start_yaw)
    pts = [np.array([[x, y]], dtype=np.float64)]

    for e in elements:
        t = e.get('t', 'line')
        if t == 'line':
            L = float(e.get('len', 0.0))
            x += L * math.cos(yaw)
            y += L * math.sin(yaw)
            pts.append(np.array([[x, y]]))
        elif t == 'arc':
            r = float(e.get('r', 1.0))
            deg = float(e.get('deg', 45.0))
            n = max(2, int(abs(deg) / arc_step_deg) + 1)
            # 左转(deg>0)：圆心在当前点左侧 r 处
            sign = 1.0 if deg >= 0 else -1.0
            # 圆心：左转在航向左法向，右转在右法向
            cx = x - sign * r * math.sin(yaw)
            cy = y + sign * r * math.cos(yaw)
            # 起点相对圆心的角度
            a0 = math.atan2(y - cy, x - cx)
            a1 = a0 + math.radians(deg)
            arc = _arc_points(cx, cy, r, a0, a1, n)
            pts.append(arc[1:])
            x, y = arc[-1]
            yaw += math.radians(deg)
        elif t == 'pts':
            p = np.asarray(e.get('p', []), dtype=np.float64)
            if len(p) >= 2:
                pts.append(p[1:] if np.allclose(p[0], [x, y]) else p)
                x, y = p[-1]
                yaw = math.atan2(p[-1][1] - p[-2][1], p[-1][0] - p[-2][0])
    return np.vstack(pts)


def _segments(pts):
    p = np.asarray(pts, dtype=np.float64)
    return np.hstack([p[:-1], p[1:]])


class Track(object):
    def __init__(self, design=None):
        if design is None:
            design = load_design()
        self.design = design
        self.half_w = float(design.get('half_w', 0.25))
        self.start = design.get('start', [0.15, 0.0, 0.0])

        el = design.get('path', [])
        if el and isinstance(el[0], dict) and el[0].get('t') == 'pts':
            self.line = path_from_elements(el)
        else:
            self.line = path_from_elements(el, start_xy=self.start[:2], start_yaw=self.start[2])
        self.segs = _segments(self.line)

        self.crosses = design.get('crosses', [])
        self.rings = design.get('rings', [])

        xs = np.concatenate([self.segs[:, 0], self.segs[:, 2]])
        ys = np.concatenate([self.segs[:, 1], self.segs[:, 3]])
        self.x0, self.x1 = xs.min() - GRID_PAD, xs.max() + GRID_PAD
        self.y0, self.y1 = ys.min() - GRID_PAD, ys.max() + GRID_PAD
        self.nx = int(np.ceil((self.x1 - self.x0) / GRID_RES)) + 1
        self.ny = int(np.ceil((self.y1 - self.y0) / GRID_RES)) + 1
        self.gx = self.x0 + np.arange(self.nx) * GRID_RES
        self.gy = self.y0 + np.arange(self.ny) * GRID_RES
        self.mask = self._rasterize()

    # ---------- 栅格化 ----------
    def _rasterize(self):
        X, Y = np.meshgrid(self.gx, self.gy)
        m = np.zeros(X.shape, dtype=bool)
        hw = self.half_w

        for (ax, ay, bx, by) in self.segs:
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 <= 1e-12:
                continue
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0.0, 1.0)
            dx = X - (ax + t * vx)
            dy = Y - (ay + t * vy)
            m |= (dx * dx + dy * dy) <= hw * hw

        # 十字支路（沿线方向铺一段）
        for c in self.crosses:
            cx, cy = float(c['x']), float(c['y'])
            yaw = float(c.get('yaw', math.pi / 2))
            half = float(c.get('half', 1.3))
            dx, dy = math.cos(yaw) * half, math.sin(yaw) * half
            vx, vy = dx * 2, dy * 2
            L2 = vx * vx + vy * vy
            if L2 > 1e-12:
                ax, ay = cx - dx, cy - dy
                t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0.0, 1.0)
                ddx = X - (ax + t * vx)
                ddy = Y - (ay + t * vy)
                m |= (ddx * ddx + ddy * ddy) <= hw * hw

        # 圆环（环带；环心那段中线要挖掉，否则车会直穿环心）
        for g in self.rings:
            rx, ry = float(g['x']), float(g['y'])
            ri, ro = float(g.get('r_in', 0.75)), float(g.get('r_out', 1.35))
            r = np.hypot(X - rx, Y - ry)
            m |= (r >= ri) & (r <= ro)
            m &= ~((r < ri) & (np.abs(X - rx) <= ro) & (np.abs(Y - ry) <= hw + 0.02))
        return m

    # ---------- 查询 ----------
    def is_white(self, x, y):
        ix = np.rint((np.asarray(x) - self.x0) / GRID_RES).astype(np.int32)
        iy = np.rint((np.asarray(y) - self.y0) / GRID_RES).astype(np.int32)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        out = np.zeros(np.shape(ix), dtype=bool)
        out[ok] = self.mask[iy[ok], ix[ok]]
        return out

    def start_pose(self):
        s = self.start
        return float(s[0]), float(s[1]), float(s[2])


# ---------------- 设计文件 ----------------
def load_design(path=None):
    p = path or DESIGN_FILE
    if os.path.exists(p):
        try:
            with open(p, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            pass
    return json.loads(json.dumps(DEFAULT_DESIGN))


def save_design(design, path=None):
    p = path or DESIGN_FILE
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(design, f, ensure_ascii=False, indent=1)
    return p


if __name__ == '__main__':
    t = Track()
    print('元素 %d 段，白面积 %.2f%%，十字 %d 个，圆环 %d 个'
          % (len(t.segs), 100.0 * t.mask.mean(), len(t.crosses), len(t.rings)))
