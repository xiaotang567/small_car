# -*- coding: utf-8 -*-
"""参数扫描：舵机参数（传动比 / 舵机满行程 / 死区）× 控制器（视觉 / 视觉+陀螺）× 陀螺增益

    python tools\\sim_sweep.py 100        # 每个配置跑 100 秒（默认），两遍取平均

为什么跑这么久：45 秒只能跑到 1 圈的一半，几乎所有配置都"没出界"，区分不出来。
100 秒 ≈ 1.3 圈，能暴露弯道里切内弯/甩出去的问题。

指标（越小越好，除圈数）：
    圈数   跨过起跑线（底部直道 x=0.5）的次数
    出界   车身中心离开白色路面的累计秒数
    偏差   扫线误差 e 的均方根（px，越小说明贴中线贴得越稳）
    丢线   扫线失败（LOST）时间占比
    分     = 圈数×50 − 出界秒×20 − 偏差×2
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim'))

import numpy as np
from sim import Sim
import steering

DT = 0.005


def run_once(secs, controller, params, seed, wheel_gain, servo_max_deg, deadband):
    steering.SERVO_MAX_DEG = servo_max_deg
    steering.WHEEL_DEADBAND = deadband
    sim = Sim(params=dict(params), controller=controller, servo_gain=wheel_gain, seed=seed)
    sim.start()

    laps = 0
    prev_x = sim.x
    off_ms = 0.0
    lost_n = 0
    n = 0
    se = 0.0
    for i in range(int(secs / DT)):
        sim.step(DT)
        if i % 4 == 0:
            n += 1
            if prev_x < 0.5 <= sim.x and abs(sim.y) < 0.6:
                laps += 1
            prev_x = sim.x
            if not sim.on_track:
                off_ms += 20.0
            if sim.state.state == 0:
                lost_n += 1
            if sim.scan.ok:
                se += sim.steer.e * sim.steer.e
    rms = math.sqrt(se / max(n, 1))
    return dict(laps=max(0, laps - 1), off=off_ms / 1000.0, rms=rms,
                lost=lost_n / max(n, 1))


def run(secs, controller, params=None, wheel_gain=None, servo_max_deg=22.0,
        deadband=0.8, seeds=(1, 2, 3)):
    acc = dict(laps=0, off=0.0, rms=0.0, lost=0.0)
    for sd in seeds:
        r = run_once(secs, controller, params or {}, sd, wheel_gain, servo_max_deg, deadband)
        for k in acc:
            acc[k] += r[k]
    m = len(seeds)
    out = {k: v / m for k, v in acc.items()}
    out['score'] = out['laps'] * 50.0 - out['off'] * 20.0 - out['rms'] * 2.0
    out.update(ctrl=controller, g=wheel_gain, smax=servo_max_deg, db=deadband,
               params=dict(params or {}))
    return out


def fmt(r):
    return ('%-6s gain=%.2f smax=%4.1f° db=%.1f | 圈 %2d  出界 %6.2fs  偏差 %5.1fpx  丢线 %4.1f%%  分 %7.1f'
            % (r['ctrl'], r['g'], r['smax'], r['db'], r['laps'], r['off'],
               r['rms'], 100.0 * r['lost'], r['score']))


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    res = []

    print('=' * 118)
    print('第 1 轮：舵机传动比 × 控制器（每个配置 %.0f 秒，2 次取平均）' % secs)
    print('=' * 118)
    for g in (0.55, 0.65, 0.75):
        for ctrl in ('vision', 'gyro'):
            r = run(secs, ctrl, wheel_gain=g)
            res.append(r)
            print(fmt(r), flush=True)

    best = max(res, key=lambda r: r['score'])
    print('-' * 118)
    print('第 1 轮最好 -> %s' % fmt(best), flush=True)

    print('=' * 118)
    print('第 2 轮：舵机满行程 ±18/22/26/30°、死区 0.3/0.8/1.5°（gain=%.2f，只跑较好的控制器）' % best['g'])
    print('=' * 118)
    for smax in (26.0,):
        for db in (0.3, 1.5):
            for ctrl in ('vision', 'gyro'):
                r = run(secs, ctrl, wheel_gain=best['g'], servo_max_deg=smax, deadband=db)
                res.append(r)
                print(fmt(r), flush=True)

    print('=' * 118)
    print('第 3 轮：陀螺阻尼 k_yaw × 弯道前馈 k_curve_ff（gain=%.2f）' % best['g'])
    print('=' * 118)
    for k_yaw in (0.0, 1.5, 3.0, 6.0):
        for k_ff in (0.0,):
            r = run(secs, 'gyro', {'k_yaw': k_yaw, 'k_curve_ff': k_ff}, wheel_gain=best['g'])
            r['k_yaw'] = k_yaw
            r['k_ff'] = k_ff
            res.append(r)
            print('k_yaw=%4.1f k_ff=%3.1f  ' % (k_yaw, k_ff) + fmt(r), flush=True)

    res.sort(key=lambda r: -r['score'])
    print('=' * 118)
    print('总排行（前 10）：')
    for r in res[:10]:
        extra = ''
        if r.get('k_yaw') is not None:
            extra = '  k_yaw=%.1f k_curve_ff=%.1f' % (r['k_yaw'], r['k_ff'])
        print('  ' + fmt(r) + extra)
    print('=' * 118)
    top = res[0]
    print('推荐：controller=%s  wheel_gain=%.2f  SERVO_MAX_DEG=%.1f  WHEEL_DEADBAND=%.1f'
          % (top['ctrl'], top['g'], top['smax'], top['db']))
    print('      参数 = %s' % top['params'])


if __name__ == '__main__':
    main()
