# -*- coding: utf-8 -*-
"""用**标定出来的真实相机姿态**生成固件用的逆透视查表（Core/Src/ipm_lut.c）

两个函数给外部用（tools/ipm_cal.py 标定工具调它们）：
    fit_from_uv(uv, L)            4 个角的图像坐标 + 正方形边长 → 相机姿态
    write_lut(L, pose, err, ...)  姿态 → 写 ipm_lut.c/.h（可选出俯视预览图）

命令行用法（不点鼠标时，直接吃 tools/cal/mark.json）：
    python tools\\gen_ipm_lut_cal.py --L 0.45
    python tools\\gen_ipm_lut_cal.py --L 0.45 --preview

俯视图网格（和固件里 ipm_f4.h 一致，**不改**）：
    120 行 × 188 列，前向 0.05~1.55 m，横向 ±0.60 m（6.42 mm/列）
    列 0 = 车体**左**侧，列 187 = 右侧      ← 和相机原图同向，运行时不再翻
    行 0 = 车前 5cm（近），行 119 = 1.55m（远） ← 画图时倒过来（近处在下）
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


def sort_quad(uv):
    """4 个点排成 far_left / far_right / near_right / near_left（v 小=远，u 小=左）"""
    uv = np.asarray(uv, dtype=float).reshape(4, 2)
    order = np.argsort(uv[:, 1])
    far = uv[order[:2]]
    near = uv[order[2:]]
    far = far[np.argsort(far[:, 0])]
    near = near[np.argsort(near[:, 0])]
    return np.array([far[0], far[1], near[1], near[0]])


def fit_from_uv(uv, L):
    """4 个角 + 边长 → ((f, tau, h, Xc, Yc, psi), 每个角的重投影误差)"""
    uv = sort_quad(uv)
    p0 = [162.8, np.deg2rad(9.0), 0.20, 0.45, 0.0, 0.0]
    r = least_squares(C.make_resid(uv, L), p0, method='lm', max_nfev=20000,
                      x_scale=[50, 0.05, 0.05, 0.2, 0.2, 0.1])
    f, tau, h, Xc, Yc, psi = r.x
    err = np.hypot(r.fun[:4], r.fun[4:])
    return (f, tau, h, Xc, Yc, psi), err


def build_index(pose):
    """算每格俯视像素对应的相机像素下标；返回 dict（含 ok / ui / vi / xs）"""
    f, tau, h = pose[0], pose[1], pose[2]
    xs = X0 + np.arange(ROWS) * RES_Y
    ys = HALF_Y - np.arange(COLS) * RES_X        # ★ 列 0 = 左边（y=+0.6）
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    u, v = C.project(np.stack([X, Y], axis=-1), f, tau, h)
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    ok = (u > -0.5) & (u < COLS - 0.5) & (v > -0.5) & (v < ROWS - 0.5)
    idx = np.full((ROWS, COLS), 0xFFFF, dtype=np.uint16)
    idx[ok] = (vi[ok] * COLS + ui[ok]).astype(np.uint16)
    return {'idx': idx, 'ok': ok, 'ui': ui, 'vi': vi, 'xs': xs}


def write_lut(L, pose, err, preview_img=None, preview_path=None):
    """写 Core/Src/ipm_lut.c + Core/Inc/ipm_lut.h；preview_img = 188×120 二值图时顺手出俯视预览"""
    f, tau, h, Xc, Yc, psi = pose
    g = build_index(pose)
    idx, ok = g['idx'], g['ok']
    info = {
        'L': L, 'f': float(f), 'tau_deg': float(np.rad2deg(tau)), 'h': float(h),
        'Xc': float(Xc), 'Yc': float(Yc), 'psi_deg': float(np.rad2deg(psi)),
        'reproj_max_px': float(err.max()),
        'valid_pct': float(100.0 * ok.sum() / idx.size),
        'rows_ok': (int(np.nonzero(ok.any(axis=1))[0][0]), int(np.nonzero(ok.any(axis=1))[0][-1]))
        if ok.any() else (0, 0),
    }

    lines = []
    lines.append('/*******************************************************************************')
    lines.append('* 文件名称          ipm_lut.c   （**自动生成，别手改** -> tools/gen_ipm_lut_cal.py / tools/ipm_cal.py）')
    lines.append('* 说明              逆透视查找表：俯视图 (r,c) -> 相机 188x120 像素 idx = v*188+u')
    lines.append('*                  0xFFFF = 视野外（黑）')
    lines.append('*  ★ 这张表不是名义参数，是**标定出来的真实姿态**（手点正方形 4 角反解）：')
    lines.append('*      f = %.2f px (HFOV %.1f°) / 低头 %.2f° / 相机高 %.3f m'
                 % (f, np.rad2deg(2 * np.arctan(COLS * 0.5 / f)), np.rad2deg(tau), h))
    lines.append('*      标定正方形：边长 %.3f m，中心 前 %.3f m 横向 %+.3f m 转角 %.2f°（重投影误差 ≤ %.2f px）'
                 % (L, Xc, Yc, np.rad2deg(psi), err.max()))
    lines.append('*    俯视网格：120 行(前 0.05~1.55m) × 188 列(横 ±0.60m)')
    lines.append('*              列 0 = 车体左侧，行 0 = 最近 —— 和相机原图同向，运行时不再翻')
    lines.append('*      %d 行 × %d 列 = %d 字节 flash' % (ROWS, COLS, ROWS * COLS * 2))
    lines.append('*******************************************************************************/')
    lines.append('#include "ipm_lut.h"')
    lines.append('')
    lines.append('const uint16_t ipm_lut[IPM_BEV_ROWS][IPM_BEV_COLS] = {')
    for r in range(ROWS):
        lines.append('  {%s},' % ','.join('%u' % idx[r, c] for c in range(COLS)))
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

    try:
        os.makedirs(CAL, exist_ok=True)
        with io.open(os.path.join(CAL, 'lut_pose.json'), 'w', encoding='utf-8') as fp:
            json.dump(info, fp, ensure_ascii=False, indent=2)
    except Exception:
        pass

    if (preview_img is not None) and preview_path:
        bev = np.zeros((ROWS, COLS), dtype=np.uint8)
        bev[ok] = preview_img[g['vi'][ok], g['ui'][ok]]
        show = np.flipud(bev)                 # 近处 -> 下
        k = 4
        big = Image.fromarray(show).resize((COLS * k, ROWS * k), Image.NEAREST).convert('RGB')
        d = ImageDraw.Draw(big)
        for c in range(0, COLS, 10):
            d.line([(c * k, 0), (c * k, ROWS * k)], fill=(0, 110, 255))
        for r in range(0, ROWS, 10):
            d.line([(0, (ROWS - 1 - r) * k), (COLS * k, (ROWS - 1 - r) * k)], fill=(0, 110, 255))
        big.save(preview_path)
        info['preview'] = preview_path

    return info


def main():
    L = 0.45
    want_prev = False
    for i, a in enumerate(sys.argv):
        if a == '--L' and i + 1 < len(sys.argv):
            L = float(sys.argv[i + 1])
        if a == '--preview':
            want_prev = True

    js = json.load(io.open(os.path.join(CAL, 'mark.json'), encoding='utf-8'))
    uv = np.array([[p['u'], p['v']] for p in js['pts']], dtype=float)
    pose, err = fit_from_uv(uv, L)
    print('标定结果（正方形边长 %.2f m）：' % L)
    print('  f = %.2f px   HFOV = %.1f°' % (pose[0], np.rad2deg(2 * np.arctan(COLS * 0.5 / pose[0]))))
    print('  低头 = %.2f°   相机高 = %.3f m' % (np.rad2deg(pose[1]), pose[2]))
    print('  重投影误差 最大 %.2f px' % err.max())

    img = None
    if want_prev:
        img, src = C.recon_image()
        print('  预览源图：%s' % os.path.basename(src))
    info = write_lut(L, pose, err, img, os.path.join(CAL, 'bev_new.png') if want_prev else None)
    print('  有效查表项 %.0f%%，可见行 %d~%d' % (info['valid_pct'], info['rows_ok'][0], info['rows_ok'][1]))
    print('已写：%s\n      %s' % (OUT_C, OUT_H))
    if want_prev:
        print('已存俯视预览：%s' % info['preview'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
