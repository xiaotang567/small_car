# -*- coding: utf-8 -*-
"""赛道几何（3D 场景的"地面贴图"）

坐标系：x 东 / y 北 / z 上，单位 米（实车场地同一量级：赛道宽 0.5m）
赛道要素（按用户要求）：
  · 一圈闭环：上下两条长直道 + 左右两条竖道 + 四个 R=1.3m 的 90° 弯
  · **十字**：底部直道 x=3 处有一条横向支路穿过（路口白色区域是个"+"）
  · **圆环**：顶部直道中间断开，用一个环形道（内径 0.75 / 外径 1.35）接起来，
            环心是黑岛 ⇒ 车必须沿环走、不能直穿

实现方式：赛道一次性栅格化成 2cm 分辨率的**占用图**（True=白色路面），
之后相机/3D 渲染只做查表，22560 个像素也就几毫秒。
"""
import numpy as np

# ---------------- 场地参数 ----------------
TRACK_HALF_W = 0.25         # 赛道半宽（0.5m 宽 ≈ 真实智能车赛道）
CORNER_R     = 1.3          # 四个角的半径（R=1.0 太紧，实车会切内弯）
RING_INNER   = 0.75         # 圆环内径（黑洞半径）
RING_OUTER   = 1.35         # 圆环外径
RING_CENTER  = (3.0, 6.0)   # 圆环中心（在顶部直道上）
CROSS_CENTER = (3.0, 0.0)   # 十字路口中心（在底部直道上）
CROSS_HALF_L = 1.3          # 十字支路半长
LOOP_W       = 6.0          # 底部直道长度（x 从 0 到 6）
LOOP_H       = 6.0          # 左右竖道高度（y 从 0 到 6）

GRID_RES = 0.02             # 占用图分辨率 2cm
GRID_PAD = 0.8              # 场地外扩（米）


def _arc(cx, cy, r, a0, a1, n=48):
    a = np.linspace(a0, a1, n)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def build_centerline():
    """闭环中心线。
       底部直道 y=0（x 0..6）；右下角圆心 (6,R)；右侧 x=6+R；右上角圆心 (6,6-R)；
       顶部直道 y=6（x 6..0）；左上角圆心 (0,6-R)；左侧 x=-R；左下角圆心 (0,R)。"""
    R, W, H = CORNER_R, LOOP_W, LOOP_H
    return np.vstack([
        np.array([[0.0, 0.0], [W, 0.0]]),
        _arc(W, R, R, -np.pi / 2, 0.0),
        np.array([[W + R, R], [W + R, H - R]]),
        _arc(W, H - R, R, 0.0, np.pi / 2),
        np.array([[W, H], [0.0, H]]),
        _arc(0.0, H - R, R, np.pi / 2, np.pi),
        np.array([[-R, H - R], [-R, R]]),
        _arc(0.0, R, R, np.pi, 1.5 * np.pi),
    ])


def _polyline_segments(pts):
    p = np.asarray(pts, dtype=np.float64)
    return np.hstack([p[:-1], p[1:]])


class Track(object):
    def __init__(self):
        self.line = build_centerline()
        self.segs = _polyline_segments(self.line)
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

        # 1) 所有直道/弧线段：点到线段距离 <= 半宽
        for (ax, ay, bx, by) in self.segs:
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 <= 1e-12:
                continue
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0.0, 1.0)
            dx = X - (ax + t * vx)
            dy = Y - (ay + t * vy)
            m |= (dx * dx + dy * dy) <= TRACK_HALF_W ** 2

        # 2) 十字支路（沿 y 方向穿过底部直道）
        cx, cy = CROSS_CENTER
        m |= (np.abs(X - cx) <= TRACK_HALF_W) & (np.abs(Y - cy) <= CROSS_HALF_L)

        # 3) 圆环环带
        rx, ry = RING_CENTER
        r = np.hypot(X - rx, Y - ry)
        m |= (r >= RING_INNER) & (r <= RING_OUTER)

        # 4) 环心那一段的顶部直道要断开（否则车会直穿环心）
        m &= ~((r < RING_INNER)
               & (np.abs(X - rx) <= RING_OUTER)
               & (np.abs(Y - ry) <= TRACK_HALF_W + 0.02))
        return m

    # ---------- 查询 ----------
    def is_white(self, x, y):
        """世界坐标 → 是否白色路面（越界 = 黑）"""
        ix = np.rint((np.asarray(x) - self.x0) / GRID_RES).astype(np.int32)
        iy = np.rint((np.asarray(y) - self.y0) / GRID_RES).astype(np.int32)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        out = np.zeros(np.shape(ix), dtype=bool)
        out[ok] = self.mask[iy[ok], ix[ok]]
        return out

    def start_pose(self):
        return 0.15, 0.0, 0.0        # x, y, yaw(rad)：底部直道左端，朝东


if __name__ == '__main__':
    t = Track()
    print('grid %dx%d  white %.2f%%' % (t.nx, t.ny, 100.0 * t.mask.mean()))
