# -*- coding: utf-8 -*-
"""用"地上一个正方形"的 4 个角 反解相机姿态，并生成新的逆透视查表

输入  tools/cal/mark.json        （cal_mark.py 的输出：4 个角的图像坐标）
      tools/cal/*_x8.png         （标过的图，用来"抠"回原始 188x120 二值图）
参数  --L 0.50                   正方形实际边长（米）

输出  打印：f / 低头角 / 相机高 / 正方形位置 / 重投影误差
      tools/cal/bev_preview.png  （用新查表算出来的俯视图，看正方形是不是正方形）
      tools/cal/fit_check.png    （把正方形投回原图，验证 4 个角对得上）
      tools/cal/fit.json         （拟合结果）
"""
import io
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, 'cal')

# ---- 固件的俯视图网格（和 ipm_f4.h 保持一致）----
BEV_ROWS, BEV_COLS = 120, 188
BEV_X0, BEV_X1 = 0.05, 1.55
BEV_HALF_Y = 0.60
RES_Y = (BEV_X1 - BEV_X0) / (BEV_ROWS - 1)
RES_X = 2.0 * BEV_HALF_Y / (BEV_COLS - 1)
U0, V0 = (BEV_COLS - 1) / 2.0, (BEV_ROWS - 1) / 2.0


def project(pts_xy, f, tau, h, u0=U0, v0=V0):
    """车体系地面点 (X 前, Y 左, 米) -> 图像 (u, v)；相机在 (0,0,h)，低头 tau"""
    X = pts_xy[..., 0]
    Y = pts_xy[..., 1]
    cp, sp = np.cos(tau), np.sin(tau)
    z = X * cp + h * sp
    y_up = X * sp - h * cp
    u = u0 - f * Y / z
    v = v0 - f * y_up / z
    return u, v


def square_pts(center, yaw, L):
    """正方形 4 角（车体系）：far_left, far_right, near_right, near_left"""
    c, s = np.cos(yaw), np.sin(yaw)
    e1 = np.array([c, s])        # 远端方向
    e2 = np.array([-s, c])       # 左方向
    h = 0.5 * L
    ctr = np.asarray(center, dtype=float)
    return np.array([ctr + h * e1 + h * e2,    # far_left
                     ctr + h * e1 - h * e2,    # far_right
                     ctr - h * e1 - h * e2,    # near_right
                     ctr - h * e1 + h * e2])   # near_left


def make_resid(uv, L, fixed_h=None, free_L=False):
    u_meas = uv[:, 0]
    v_meas = uv[:, 1]

    def resid(p):
        f, tau = p[0], p[1]
        k = 2
        if fixed_h is None:
            h = p[k]; k += 1
        else:
            h = fixed_h
        Xc, Yc, psi = p[k], p[k + 1], p[k + 2]; k += 3
        if free_L:
            Lv = p[k]; k += 1
        else:
            Lv = L
        u, v = project(square_pts((Xc, Yc), psi, Lv), f, tau, h)
        return np.concatenate([u - u_meas, v - v_meas])
    return resid


def recon_image():
    """从标过的 _x8 图里抠回原始二值图：每个 8x8 块里扔掉红点，取多数"""
    cand = [os.path.join(CAL, f) for f in os.listdir(CAL) if f.endswith('_x8.png')]
    path = max(cand, key=os.path.getmtime)
    a = np.asarray(Image.open(path).convert('RGB')).astype(np.int16)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    red = (r >= 110) & (r - g >= 45) & (r - b >= 45)
    white = a.mean(axis=2) > 127
    w64 = white.reshape(BEV_ROWS, 8, BEV_COLS, 8).transpose(0, 2, 1, 3).reshape(BEV_ROWS, BEV_COLS, 64)
    m64 = red.reshape(BEV_ROWS, 8, BEV_COLS, 8).transpose(0, 2, 1, 3).reshape(BEV_ROWS, BEV_COLS, 64)
    good = ~m64
    nw = (w64 & good).sum(axis=2)
    ng = good.sum(axis=2)
    out = np.where(ng > 0, (nw * 2 > ng), False)
    return out.astype(np.uint8) * 255, path


def main():
    L = 0.50
    for i, a in enumerate(sys.argv):
        if a == '--L' and i + 1 < len(sys.argv):
            L = float(sys.argv[i + 1])
    js = json.load(io.open(os.path.join(CAL, 'mark.json'), encoding='utf-8'))
    uv = np.array([[p['u'], p['v']] for p in js['pts']], dtype=float)
    print('4 个角：\n%s' % np.array2string(uv, precision=2))
    print('假设边长 L = %.3f m\n' % L)

    p0 = [162.8, np.deg2rad(9.0), 0.13, 0.45, 0.0, 0.0]
    r1 = least_squares(make_resid(uv, L), p0, method='lm', max_nfev=20000,
                       x_scale=[50, 0.05, 0.05, 0.2, 0.2, 0.1])
    f, tau, h, Xc, Yc, psi = r1.x
    err = np.hypot(r1.fun[:4], r1.fun[4:])
    print('【拟合 A：L 固定 = %.2f m】cost=%.2f' % (L, r1.cost))
    print('  f      = %.1f px   (HFOV = %.1f°)' % (f, np.rad2deg(2 * np.arctan(BEV_COLS * 0.5 / f))))
    print('  低头角 = %.2f°' % np.rad2deg(tau))
    print('  相机高 = %.3f m' % h)
    print('  正方形中心：前 %.3f m，横向 %+.3f m（左正），转角 %.2f°'
          % (Xc, Yc, np.rad2deg(psi)))
    print('  重投影误差(像素): ' + ', '.join('%.2f' % e for e in err) + '   最大 %.2f' % err.max())

    p0b = [162.8, np.deg2rad(9.0), 0.45, 0.0, 0.0, 0.5]
    r2 = least_squares(make_resid(uv, L, fixed_h=0.13, free_L=True), p0b, method='lm',
                       max_nfev=20000, x_scale=[50, 0.05, 0.2, 0.2, 0.1, 0.2])
    f2, tau2, Xc2, Yc2, psi2, L2 = r2.x
    err2 = np.hypot(r2.fun[:4], r2.fun[4:])
    print('\n【拟合 B：相机高固定 0.130 m，反求边长】')
    print('  ⇒ 反求出的边长 L = %.3f m  (你标的如果是 %.2f m，两者差 %.0f%%)'
          % (L2, L, 100.0 * (L2 - L) / L))
    print('  f = %.1f px, 低头角 = %.2f°, 最大重投影误差 %.2f px'
          % (f2, np.rad2deg(tau2), err2.max()))

    # ---------- 用 A 生成新查表 + 预览 ----------
    img, src = recon_image()
    print('\n原图取自：%s' % os.path.basename(src))

    xs = BEV_X0 + np.arange(BEV_ROWS) * RES_Y
    ys = -BEV_HALF_Y + np.arange(BEV_COLS) * RES_X      # ★ 列 0 = 车体右侧（和旧表一致）
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    u, v = project(np.stack([X, Y], axis=-1), f, tau, h)
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    ok = (ui >= 0) & (ui < BEV_COLS) & (vi >= 0) & (vi < BEV_ROWS)
    bev = np.zeros((BEV_ROWS, BEV_COLS), dtype=np.uint8)
    bev[ok] = img[vi[ok], ui[ok]]
    print('查表有效像素 %d / %d (%.0f%%)' % (ok.sum(), ok.size, 100.0 * ok.sum() / ok.size))

    # 预览：近处在下、左右按真实方向（显示用），放大 4 倍 + 网格
    show = np.flipud(bev)                 # 近处 -> 下方
    show = np.fliplr(show)                # 列 0 = 右 -> 显示时右边
    k = 4
    big = Image.fromarray(show).resize((BEV_COLS * k, BEV_ROWS * k), Image.NEAREST).convert('RGB')
    d = ImageDraw.Draw(big)
    for c in range(0, BEV_COLS, 10):
        d.line([((BEV_COLS - 1 - c) * k, 0), ((BEV_COLS - 1 - c) * k, BEV_ROWS * k)], fill=(0, 110, 255))
    for r in range(0, BEV_ROWS, 10):
        d.line([(0, (BEV_ROWS - 1 - r) * k), (BEV_COLS * k, (BEV_ROWS - 1 - r) * k)], fill=(0, 110, 255))
    big.save(os.path.join(CAL, 'bev_preview.png'))
    print('已存预览：tools\\cal\\bev_preview.png')

    # 验证图：把正方形投回原图（绿）+ 4 个角（红）
    im = Image.open(src).convert('RGB')
    dd = ImageDraw.Draw(im)
    su, sv = project(square_pts((Xc, Yc), psi, L), f, tau, h)
    pts = [(float(su[i]) * 8, float(sv[i]) * 8) for i in range(4)]
    dd.line([pts[0], pts[1], pts[2], pts[3], pts[0]], fill=(0, 255, 0), width=2)
    for i, (x, y) in enumerate(pts):
        dd.ellipse([x - 6, y - 6, x + 6, y + 6], outline=(255, 255, 0), width=2)
    im.save(os.path.join(CAL, 'fit_check.png'))
    print('已存验证图：tools\\cal\\fit_check.png')

    out = {'L': L, 'f': f, 'tau_deg': float(np.rad2deg(tau)), 'h': h,
           'Xc': Xc, 'Yc': Yc, 'psi_deg': float(np.rad2deg(psi)),
           'reproj_max_px': float(err.max()), 'L_if_h_013': L2,
           'grid': {'rows': BEV_ROWS, 'cols': BEV_COLS, 'x0': BEV_X0, 'x1': BEV_X1,
                    'half_y': BEV_HALF_Y}}
    with io.open(os.path.join(CAL, 'fit.json'), 'w', encoding='utf-8') as fp:
        json.dump(out, fp, ensure_ascii=False, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
