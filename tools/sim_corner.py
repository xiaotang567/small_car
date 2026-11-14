# -*- coding: utf-8 -*-
"""弯道里"压不压内侧"的量化工具

对每一拍算出车相对**真实赛道中线**的**带符号横向偏差**：
    dev > 0 = 车在中线**左侧**（左转时=靠外侧）
    dev < 0 = 车在中线**右侧**
再按"是不是在转弯"（|陀螺 gz| > 8°/s）分成两组统计：
    corner_bias = 弯道里的平均符号偏差（**负得越多 = 越压内侧**）
    corner_rms  = 弯道里的偏差均方根（越小 = 贴中线越准）

    python tools\\sim_corner.py            # 默认每组 45 秒
    python tools\\sim_corner.py 60 5 6 7   # 指定秒数 + 多组参数（见 PARAMS）
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim'))

import numpy as np
from sim import Sim
import control as C
import steering

DT = 0.005
GZ_CURVE = 8.0


def signed_dev(track, x, y):
    """到最近线段的距离 + 带符号（左正右负）"""
    seg = np.asarray(track.segs, dtype=np.float64)
    ax, ay, bx, by = seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3]
    vx, vy = bx - ax, by - ay
    L2 = np.maximum(vx * vx + vy * vy, 1e-12)
    t = np.clip(((x - ax) * vx + (y - ay) * vy) / L2, 0.0, 1.0)
    px, py = ax + t * vx, ay + t * vy
    d = np.hypot(x - px, y - py)
    k = int(np.argmin(d))
    # 线段方向 (vx,vy)，车相对投影点的偏移向量
    ox, oy = x - px[k], y - py[k]
    cross = vx[k] * oy - vy[k] * ox          # >0 => 偏移在前进方向左侧
    return (1.0 if cross >= 0 else -1.0) * float(d[k])


def run(secs=45.0, wheel_gain=0.62, w_cur=None, kff=None, kp_cur=None, kd_cur=None,
        controller='gyro', seed=1):
    if w_cur is not None:
        C.STEER_W_CUR = w_cur
    p = {}
    if kff is not None:
        p['kff'] = kff
    if kp_cur is not None:
        p['kp_cur'] = kp_cur
    if kd_cur is not None:
        p['kd_cur'] = kd_cur
    sim = Sim(params=p, controller=controller, servo_gain=wheel_gain, seed=seed)
    sim.start()

    cor = []
    strt = []
    off_ms = 0.0
    for i in range(int(secs / DT)):
        sim.step(DT)
        if i % 4 == 0:
            d = signed_dev(sim.track, sim.x, sim.y)
            if abs(sim.imu.gz) > GZ_CURVE and sim.rpm > 30:
                cor.append(d)
            else:
                strt.append(d)
            if not sim.on_track:
                off_ms += 20.0

    def stat(a):
        if not a:
            return (0.0, 0.0)
        a = np.asarray(a)
        return float(a.mean()), float(np.sqrt((a ** 2).mean()))

    cb, cr = stat(cor)
    sb, sr = stat(strt)
    return dict(w=w_cur, kff=kff, kp=kp_cur, kd=kd_cur,
                cor_bias=cb, cor_rms=cr, str_bias=sb, str_rms=sr,
                off=off_ms / 1000.0, n_cor=len(cor), n_str=len(strt))


def show(tag, r):
    print('%-22s 弯道 偏置 %+6.3fm 均方根 %6.3fm | 直道 偏置 %+6.3fm 均方根 %6.3fm | 出界 %5.2fs'
          % (tag, r['cor_bias'], r['cor_rms'], r['str_bias'], r['str_rms'], r['off']), flush=True)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    track_hw = 0.25
    print('赛道半宽 %.3fm；"偏置"负=压内侧，正=偏外侧' % track_hw)
    print('=' * 118)

    cases = [
        ('基线 w_cur=0.80',      dict(w_cur=0.80)),
        ('w_cur=0.60',           dict(w_cur=0.60)),
        ('w_cur=0.45',           dict(w_cur=0.45)),
        ('w_cur=0.30',           dict(w_cur=0.30)),
        ('w_cur=0.45 kff=100',   dict(w_cur=0.45, kff=100.0)),
        ('w_cur=0.45 kff=0',     dict(w_cur=0.45, kff=0.0)),
        ('w_cur=0.45 kp_cur=3',  dict(w_cur=0.45, kp_cur=3.0)),
        ('w_cur=0.45 kd_cur=14', dict(w_cur=0.45, kd_cur=14.0)),
    ]
    out = []
    for name, kw in cases:
        r = run(secs, **kw)
        r['name'] = name
        out.append(r)
        show(name, r)

    print('=' * 118)
    best = min(out, key=lambda r: r['cor_rms'] + 0.5 * abs(r['cor_bias']))
    print('弯道贴中线最好 -> %s' % best['name'])
    show(best['name'], best)


if __name__ == '__main__':
    main()
