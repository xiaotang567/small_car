# -*- coding: utf-8 -*-
"""3D 追车视角（纯 numpy + PIL，不需要 OpenGL）

原理和车载相机一样是**射线投射**：从车后上方一个虚拟相机往外打射线，打到地面 z=0
就查赛道的占用图 ⇒ 得到的透视是真的（近大远小、弯道会拐），不是 2D 俯视图。
车本身用一个立方体画上去：8 个角点做透视投影，再填 3 个可见面（顶/左/右）。
"""
import numpy as np
from PIL import Image, ImageDraw

from track import Track

# 追车相机
CH_FWD   = -0.52        # 在车后（负 = 车后）
CH_HEIGHT = 0.42
CH_PITCH = np.deg2rad(33.0)
CH_HFOV  = np.deg2rad(62.0)

CAR_L = 0.24            # 车长
CAR_W = 0.16            # 车宽
CAR_H = 0.055           # 车高


def _cam_rays(W, H, yaw, pitch, hfov):
    fwd = np.array([np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), -np.sin(pitch)])
    right = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    up = np.cross(fwd, right)
    tx = np.tan(hfov * 0.5)
    ty = tx * H / W
    u = (np.arange(W) + 0.5) / W * 2.0 - 1.0
    v = 1.0 - (np.arange(H) + 0.5) / H * 2.0
    U, V = np.meshgrid(u, v)
    d = (fwd[None, None, :] + (U * tx)[:, :, None] * right[None, None, :]
         + (V * ty)[:, :, None] * up[None, None, :])
    d /= np.linalg.norm(d, axis=2, keepdims=True)
    return d, fwd, right, up


def render(track: Track, x, y, yaw, W=520, H=340, trail=None):
    """返回 PIL.Image（RGB）"""
    ox = x + CH_FWD * np.cos(yaw)
    oy = y + CH_FWD * np.sin(yaw)
    oz = CH_HEIGHT
    d, fwd, right, up = _cam_rays(W, H, yaw, CH_PITCH, CH_HFOV)

    dz = d[:, :, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(dz < -1e-6, -oz / dz, np.inf)
    px = ox + d[:, :, 0] * t
    py = oy + d[:, :, 1] * t
    hit = np.isfinite(t)

    floor = np.empty((H, W, 3), dtype=np.float32)
    white = np.zeros((H, W), dtype=bool)
    white[hit] = track.is_white(px[hit], py[hit])

    # 地面颜色：赛道白 / 场外深灰；远处略带雾
    floor[:] = (34, 38, 44)
    floor[white] = (226, 228, 232)
    fog = np.clip(np.where(hit, t, 40.0) / 6.0, 0.0, 1.0)[:, :, None]
    floor = floor * (1.0 - 0.35 * fog) + np.array([20, 22, 26]) * (0.35 * fog)

    img = Image.fromarray(np.clip(floor, 0, 255).astype(np.uint8), 'RGB')
    dr = ImageDraw.Draw(img)

    # ---- 车：立方体投影 ----
    def proj(pw):
        """世界点 → 屏幕坐标（返回 (u,v) 或 None）"""
        rel = np.asarray(pw, dtype=float) - np.array([ox, oy, oz])
        # 相机基：right / up / fwd
        zc = rel @ fwd
        if zc <= 1e-3:
            return None
        xc = rel @ right
        yc = rel @ up
        tx = np.tan(CH_HFOV * 0.5)
        ty = tx * H / W
        u = (xc / zc / tx + 1.0) * 0.5 * W
        v = (1.0 - yc / zc / ty) * 0.5 * H
        return (u, v)

    c, s = np.cos(yaw), np.sin(yaw)
    corners = []
    for dx, dy in ((-CAR_L / 2, -CAR_W / 2), (CAR_L / 2, -CAR_W / 2),
                   (CAR_L / 2, CAR_W / 2), (-CAR_L / 2, CAR_W / 2)):
        wx = x + dx * c - dy * s
        wy = y + dx * s + dy * c
        corners.append((wx, wy))

    top = [proj((wx, wy, CAR_H)) for (wx, wy) in corners]
    bot = [proj((wx, wy, 0.0)) for (wx, wy) in corners]
    if all(p is not None for p in top):
        dr.polygon(top, fill=(196, 62, 52))
        for i in range(4):
            a, b = bot[i], bot[(i + 1) % 4]
            c1, c2 = top[(i + 1) % 4], top[i]
            if a and b and c1 and c2:
                dr.polygon([a, b, c1, c2], fill=(120, 34, 28))
    # 车头小箭头
    nose = proj((x + (CAR_L / 2 + 0.03) * c, y + (CAR_L / 2 + 0.03) * s, CAR_H))
    if nose:
        dr.ellipse([nose[0] - 2, nose[1] - 2, nose[0] + 2, nose[1] + 2], fill=(255, 220, 120))

    # ---- 走过的轨迹 ----
    if trail:
        pts = [proj((tx_, ty_, 0.005)) for (tx_, ty_) in trail]
        pts = [p for p in pts if p]
        if len(pts) > 1:
            dr.line(pts, fill=(60, 160, 255), width=1)

    return img


if __name__ == '__main__':
    import os
    tr = Track()
    os.makedirs('out', exist_ok=True)
    for name, (x, y, yaw) in (('straight', (1.0, 0.0, 0.0)),
                              ('corner', (6.1, 0.25, np.deg2rad(25))),
                              ('ring', (3.1, 7.1, np.pi / 2))):
        im = render(tr, x, y, yaw)
        im.save('out/chase_%s.png' % name)
        print('saved', name, im.size)
