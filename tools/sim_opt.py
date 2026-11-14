# -*- coding: utf-8 -*-
"""20 组传动比，每组自动找最好的参数（自己在后台跑，出表）

    python tools\\sim_opt.py            # 默认 45 秒/次、20 组
    python tools\\sim_opt.py 60 20      # 秒数 + 组数

对每个传动比 g：
  ① 先在 w_cur（弯道权重）里找最好的（0.35 / 0.45 / 0.55）
  ② 再用最好的 w_cur 找前馈 kff（100 / 200 / 300）
  ③ 报告：圈数、出界、弯道偏差 RMS、直道偏差 RMS、以及"这组车能跑多好"

为什么每组都要单独优化：传动比越小，前轮转得越少 ⇒ 同样的视觉偏差打不出足够的舵角，
必须把弯道权重/前馈调大；传动比大的车反过来需要压小，否则会过冲/画龙。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim'))

import numpy as np
from sim import Sim
import control as C

DT = 0.005
GZ_CURVE = 8.0


def signed_dev(track, x, y, seg):
    ax, ay, bx, by = seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3]
    vx, vy = bx - ax, by - ay
    L2 = np.maximum(vx * vx + vy * vy, 1e-12)
    t = np.clip(((x - ax) * vx + (y - ay) * vy) / L2, 0.0, 1.0)
    px, py = ax + t * vx, ay + t * vy
    d = np.hypot(x - px, y - py)
    k = int(np.argmin(d))
    cross = vx[k] * (y - py[k]) - vy[k] * (x - px[k])
    return (1.0 if cross >= 0 else -1.0) * float(d[k])


def evaluate(secs, gain, w_cur, kff, controller='gyro', seed=1):
    C.STEER_W_CUR = w_cur
    sim = Sim(params={'kff': kff}, controller=controller, servo_gain=gain,
              seed=seed, fast=True)
    sim.start()
    seg = np.asarray(sim.track.segs, dtype=np.float64)

    laps = 0
    prev_x = sim.x
    off_ms = 0.0
    cor, strt = [], []
    for i in range(int(secs / DT)):
        sim.step(DT)
        if i % 4 == 0:
            if prev_x < 0.5 <= sim.x and abs(sim.y) < 0.6:
                laps += 1
            prev_x = sim.x
            if not sim.on_track:
                off_ms += 20.0
            d = signed_dev(sim.track, sim.x, sim.y, seg)
            if abs(sim.imu.gz) > GZ_CURVE and sim.rpm > 30:
                cor.append(d)
            else:
                strt.append(d)

    def rms(a):
        return 0.0 if not a else float(np.sqrt(np.mean(np.square(a))))

    laps = max(0, laps - 1)
    score = laps * 60.0 - (off_ms / 1000.0) * 25.0 - rms(cor) * 120.0
    return dict(laps=laps, off=off_ms / 1000.0, cor=rms(cor), strt=rms(strt),
                score=score, w=w_cur, kff=kff)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    n_gain = int(sys.argv[2]) if len(sys.argv) > 2 else 20

    gains = [round(0.30 + 0.04 * k, 2) for k in range(n_gain)]
    print('每组 %.0f 秒；传动比 %s' % (secs, gains))
    print('=' * 122)
    best_rows = []
    for g in gains:
        # ① 找弯道权重
        cands = [0.35, 0.45, 0.55] if g < 0.7 else [0.30, 0.40, 0.50]
        runs = [(evaluate(secs, g, w, 200.0), w, 200.0) for w in cands]
        runs.sort(key=lambda r: -r[0]['score'])
        r1, w1, _ = runs[0]
        # ② 在这组 w 上找前馈
        runs2 = [(evaluate(secs, g, w1, ff), w1, ff) for ff in (100.0, 300.0)]
        runs2.sort(key=lambda r: -r[0]['score'])
        r2, w2, ff2 = runs2[0]
        best = r2 if r2['score'] > r1['score'] else r1
        best['gain'] = g
        best_rows.append(best)
        print('gain=%.2f | 圈 %d 出界 %5.2fs 弯道RMS %.3f m 直道RMS %.3f m | '
              '最好 w=%.2f kff=%.0f | 分 %6.1f'
              % (g, best['laps'], best['off'], best['cor'], best['strt'],
                 best['w'], best['kff'], best['score']), flush=True)

    print('=' * 122)
    ok = [r for r in best_rows if r['off'] < 1.0]
    if ok:
        good = [r['gain'] for r in ok]
        print('能稳定跑（出界<1s）的传动比区间：%.2f ~ %.2f，共 %d 组' % (min(good), max(good), len(good)))
        bestc = min(ok, key=lambda r: r['cor'])
        print('弯道最贴中线：gain=%.2f  w=%.2f kff=%.0f  弯道RMS=%.3f m'
              % (bestc['gain'], bestc['w'], bestc['kff'], bestc['cor']))
    bad = [r['gain'] for r in best_rows if r['off'] >= 1.0]
    if bad:
        print('跑不动的传动比：%s' % bad)


if __name__ == '__main__':
    main()
