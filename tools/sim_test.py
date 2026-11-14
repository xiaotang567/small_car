# -*- coding: utf-8 -*-
"""无界面跑一圈：看车能不能沿赛道走完、有没有出赛道。

用法：
    python tools\\sim_test.py [秒数]
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim'))

import numpy as np
from sim import Sim
from track import Track


def progress(track, x, y):
    """车在中心线上的弧长位置（用来数圈数）"""
    seg = track.segs
    best = (1e9, 0.0)
    acc = 0.0
    for k in range(len(seg)):
        ax, ay, bx, by = seg[k]
        L = math.hypot(bx - ax, by - ay)
        vx, vy = bx - ax, by - ay
        t = 0.0 if L < 1e-9 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / (L * L)))
        px, py = ax + t * vx, ay + t * vy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            best = (d, acc + t * L)
        acc += L
    return best[1], best[0], acc


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    sim = Sim()
    track = sim.track
    total_len = progress(track, 0, 0)[2]
    sim.start()

    dt = 0.001
    n = int(secs / dt)
    s_prev = 0.0
    s_accum = 0.0
    lap = 0
    max_dev = 0.0
    off_ms = 0.0
    log_every = 500          # 每 0.5s 打一行

    for i in range(n):
        sim.step(dt)
        s, dev, _ = progress(track, sim.x, sim.y)
        max_dev = max(max_dev, dev)
        ds = s - s_prev
        if ds < -total_len / 2:      # 跨过起点
            lap += 1
        if abs(ds) < 2.0:
            s_accum += ds
        s_prev = s
        off_ms = sim.off_track_ms

        if i % log_every == 0:
            t = sim.telemetry()
            print('%6.1fs  x=%5.2f y=%5.2f yaw=%7.1f  v=%3d  servo=%4d  ds=%d res=%3d nc=%3d  '
                  'dev=%.3fm  lap=%d off=%.0fms' % (
                      sim.t_ms / 1000.0, sim.x, sim.y, math.degrees(sim.yaw), t['r'],
                      t['servo'], t['ds'], t['rs'], t['nc'], dev, lap, off_ms))

    t = sim.telemetry()
    covered = abs(s_accum)
    print('-' * 100)
    print('跑了 %.0f 秒：沿赛道 %.2f m（一圈 %.2f m => %.2f 圈）' % (secs, covered, total_len, covered / total_len))
    print('最大偏离中心线 %.3f m（赛道半宽 %.3f m）' % (max_dev, 0.225))
    print('出赛道累计 %.0f ms' % off_ms)
    print('末态：%s' % t)


if __name__ == '__main__':
    main()
