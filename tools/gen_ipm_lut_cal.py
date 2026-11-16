# -*- coding: utf-8 -*-
"""用**标定出来的真实相机姿态**生成固件用的逆透视查表（Core/Src/ipm_lut.c）

和 gen_ipm_lut.py 的区别：
  gen_ipm_lut.py      —— 用"名义参数"（HFOV 60° / 高 0.13m / 低头 18°），是仿真的默认值
  本脚本              —— 用 tools/cal/mark.json 里**手标的正方形 4 个角**反解出来的真实姿态
                        （f、低头角、相机高、正方形位置全部由这 4 个点决定）

俯视图网格（和固件里 ipm_f4.h 一致，**不改**）：
    120 行 × 188 列，前向 0.05~1.55 m，横向 ±0.60 m（6.42 mm/列）
    列 0 = 车体**左**侧（u 小的一侧），列 187 = 右侧   ← 和相机原图同向，不用再翻
    行 0 = 车前 5cm（近），行 119 = 1.55m（远）      ← 画图时再倒过来（近处在下）

用法：
    python tools\\gen_ipm_lut_cal.py --L 0.45
    python tools\\gen_ipm_lut_cal.py --L 0.45 --preview      # 顺便出一张俯视预览图
"""
import io
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAL = os.path.join(HERE, 'cal')
sys.path.insert(0, HERE)

import cal_ipm_fit as C          # 复用：project / square_pts / make_resid / recon_image

OUT_C = os.path.join(ROOT, 'Core', 'Src', 'ipm_lut.c')
OUT_H = os.path.join(ROOT, 'Core', 'Inc', 'ipm_lut.h')

# ---- 俯视图网格（必须和 ipm_f4.h 一致）----
ROWS, COLS = 120, 188
X0, X1 = 0.05, 1.55
HALF_Y = 0.60
RES_Y = (X1 - X0) / (ROWS - 1)          # 12.605 mm/行
RES_X = 2.0 * HALF_Y / (COLS - 1)       # 6.417 mm/列


def fit_pose(L):
    js = json.load(io.open(os.path.join(CAL, 'mark.json'), encoding='utf-8'))
    uv = np.array([[p['u'], p['v']] for p in js['pts']], dtype=float)
    p0 = [162.8, np.deg2rad(9.0), 0.20, 0.45, 0.0, 0.0]
    r = least_squares(C.make_resid(uv, L), p0, method='lm', max_nfev=20000,
                      x_scale=[50, 0.05, 0.05, 0.2, 0.2, 0.1])
    f, tau, h, Xc, Yc, psi = r.x
    err = np.hypot(r.fun[:4], r.fun[4:])
    return (f, tau, h, Xc, Yc, psi), err


def main():
    L = 0.45
    want_prev = False
    for i, a in enumerate(sys.argv):
        if a == '--L' and i + 1 < len(sys.argv):
            L = float(sys.argv[i + 1])
        if a == '--preview':
            want_prev = True

    (f, tau, h, Xc, Yc, psi), err = fit_pose(L)
    print('标定结果（正方形边长 %.2f m）：' % L)
    print('  f = %.2f px   HFOV = %.1f°' % (f, np.rad2deg(2 * np.arctan(COLS * 0.5 / f))))
    print('  低头 = %.2f°   相机高 = %.3f m' % (np.rad2deg(tau), h))
    print('  正方形中心 前 %.3f m 横向 %+.3f m 转角 %.2f°' % (Xc, Yc, np.rad2deg(psi)))
    print('  重投影误差 最大 %.2f px' % err.max())

    # ---- 俯视图每个像素 -> 相机像素 ----
    xs = X0 + np.arange(ROWS) * RES_Y
    ys = HALF_Y - np.arange(COLS) * RES_X        # ★ 列 0 = 左边（y=+0.6）
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    u, v = C.project(np.stack([X, Y], axis=-1), f, tau, h)
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    ok = (u > -0.5) & (u < COLS - 0.5) & (v > -0.5) & (v < ROWS - 0.5)
    idx = np.full((ROWS, COLS), 0xFFFF, dtype=np.uint16)
    idx[ok] = (vi[ok] * COLS + ui[ok]).astype(np.uint16)
    print('  有效查表项 %d / %d  (%.0f%%)' % (ok.sum(), idx.size, 100.0 * ok.sum() / idx.size))

    rows_ok = np.nonzero(ok.any(axis=1))[0]
    if len(rows_ok):
        print('  可见前向范围 %.3f ~ %.3f m（行 %d ~ %d）'
              % (xs[rows_ok[0]], xs[rows_ok[-1]], rows_ok[0], rows_ok[-1]))

    # ---- 写 ipm_lut.c ----
    lines = []
    lines.append('/*******************************************************************************')
    lines.append('* 文件名称          ipm_lut.c   （**自动生成，别手改** -> tools/gen_ipm_lut_cal.py）')
    lines.append('* 说明              逆透视查找表：俯视图 (r,c) -> 相机 188x120 像素 idx = v*188+u')
    lines.append('*                  0xFFFF = 视野外（黑）')
    lines.append('*  ★ 这张表不是名义参数，是**标定出来的真实姿态**（手标正方形 4 角反解）：')
    lines.append('*      f = %.2f px (HFOV %.1f°) / 低头 %.2f° / 相机高 %.3f m' % (f, np.rad2deg(2 * np.arctan(COLS * 0.5 / f)), np.rad2deg(tau), h))
    lines.append('*      标定用正方形：边长 %.3f m，中心 前 %.3f m 横向 %+.3f m 转角 %.2f°（重投影误差 ≤ %.2f px）' % (L, Xc, Yc, np.rad2deg(psi), err.max()))
    lines.append('*    俯视网格：120 行(前 0.05~1.55m) × 188 列(横 ±0.60m)')
    lines.append('*              列 0 = 车体左侧，行 0 = 最近 —— 和相机原图同向，**运行时不再翻**')
    lines.append('*      %d 行 × %d 列 = %d 字节 flash' % (ROWS, COLS, ROWS * COLS * 2))
    lines.append('*******************************************************************************/')
    lines.append('#include "ipm_lut.h"')
    lines.append('')
    lines.append('const uint16_t ipm_lut[IPM_BEV_ROWS][IPM_BEV_COLS] = {')
    for r in range(ROWS):
        row = ','.join('%u' % idx[r, c] for c in range(COLS))
        lines.append('  {%s},' % row)
    lines.append('};')
    lines.append('')
    with io.open(OUT_C, 'w', encoding='utf-8', newline='\n') as fp:
        fp.write('\n'.join(lines))

    hdr = ['#ifndef __IPM_LUT_H', '#define __IPM_LUT_H', '',
           '#include <stdint.h>', '',
           '#define IPM_BEV_ROWS    %d' % ROWS,
           '#define IPM_BEV_COLS    %d' % COLS,
           '#define IPM_SRC_W       %d' % COLS,
           '#define IPM_SRC_H       %d' % ROWS, '',
           'extern const uint16_t ipm_lut[IPM_BEV_ROWS][IPM_BEV_COLS];', '',
           '#endif /* __IPM_LUT_H */', '']
    with io.open(OUT_H, 'w', encoding='utf-8', newline='\n') as fp:
        fp.write('\n'.join(hdr))
    print('\n已写：%s\n      %s' % (OUT_C, OUT_H))

    with io.open(os.path.join(CAL, 'lut_pose.json'), 'w', encoding='utf-8') as fp:
        json.dump({'L': L, 'f': f, 'tau_deg': float(np.rad2deg(tau)), 'h': h,
                   'Xc': Xc, 'Yc': Yc, 'psi_deg': float(np.rad2deg(psi)),
                   'reproj_max_px': float(err.max()), 'valid_pct':
                   float(100.0 * ok.sum() / idx.size)}, fp, ensure_ascii=False, indent=2)

    # ---- 预览：用抓到的相机图按新表生成俯视图（近处在下）----
    if want_prev:
        img, src = C.recon_image()
        bev = np.zeros((ROWS, COLS), dtype=np.uint8)
        bev[ok] = img[vi[ok], ui[ok]]
        show = np.flipud(bev)
        k = 4
        big = Image.fromarray(show).resize((COLS * k, ROWS * k), Image.NEAREST).convert('RGB')
        d = ImageDraw.Draw(big)
        for c in range(0, COLS, 10):
            d.line([(c * k, 0), (c * k, ROWS * k)], fill=(0, 110, 255))
        for r in range(0, ROWS, 10):
            d.line([(0, (ROWS - 1 - r) * k), (COLS * k, (ROWS - 1 - r) * k)], fill=(0, 110, 255))
        d.text((4, 4), 'near(bottom) / left(first col)', fill=(255, 220, 0))
        p = os.path.join(CAL, 'bev_new.png')
        big.save(p)
        print('已存俯视预览：%s （源图 %s）' % (p, os.path.basename(src)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
