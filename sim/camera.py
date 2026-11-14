# -*- coding: utf-8 -*-
"""车载相机（3D 透视 → 188×120 灰度图）

真车那颗总钻风是 **188×120、装车头前部、略微低头** 看的。这里用射线投射
（每个像素一条射线 → 打在地面 z=0 上 → 查赛道的占用图）得到**真实透视**的画面：
近处宽、远处窄、弯道会向右/左偏，和实拍完全一个形状。

成像链（尽量贴近实拍）：
  反射率 albedo(白 0.85 / 黑 0.06) → 光照衰减 → 自动曝光(AEC，把亮的顶到 255) →
  镜头暗角 → 模糊(1px) → 噪声 → uint8
选自动曝光是因为实车那颗 MT9V034 的 AEC 只能开、不能关（我们踩过坑），
所以画面亮度总是"白的地方刚好饱和"，固件里默认阈值 255 就是靠这个工作。
"""
import numpy as np
from PIL import Image, ImageFilter

from track import Track

W, H = 188, 120

CAM_FWD   = 0.20      # 相机在车中心**前方**多远（米）
CAM_H     = 0.13      # 相机离地高度（米）
CAM_PITCH = np.deg2rad(18.0)   # 低头角度
CAM_HFOV  = np.deg2rad(60.0)   # 水平视场


def _basis(yaw):
    """相机朝向：世界 z 向上，yaw=0 朝 +x。
       返回 前/右/上 三个基向量（前方向带低头角）"""
    fwd = np.array([np.cos(yaw) * np.cos(CAM_PITCH),
                    np.sin(yaw) * np.cos(CAM_PITCH),
                    -np.sin(CAM_PITCH)])
    right = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    up = np.cross(fwd, right)          # 顺序很重要：right×fwd 得到的是朝下
    return fwd, right, up


def _rays(yaw):
    """每个像素的方向向量 (H, W, 3)，已归一化"""
    tx = np.tan(CAM_HFOV * 0.5)
    ty = tx * (float(H) / float(W))          # 方形像素
    u = (np.arange(W) + 0.5) / W * 2.0 - 1.0
    v = 1.0 - (np.arange(H) + 0.5) / H * 2.0
    U, V = np.meshgrid(u, v)                 # (H, W)
    fwd, right, up = _basis(yaw)
    d = (fwd[None, None, :]
         + (U * tx)[:, :, None] * right[None, None, :]
         + (V * ty)[:, :, None] * up[None, None, :])
    d /= np.linalg.norm(d, axis=2, keepdims=True)
    return d


class Camera(object):
    def __init__(self, track: Track, size=(W, H), seed=12345, noise=0.6,
                 vignette=0.35, blur=1, saturate=True):
        self.track = track
        self.size = size
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.vignette = vignette
        self.blur = blur
        self.saturate = saturate

        # 暗角模板（一次算好）
        u = (np.arange(size[0]) + 0.5) / size[0] * 2.0 - 1.0
        v = (np.arange(size[1]) + 0.5) / size[1] * 2.0 - 1.0
        U, V = np.meshgrid(u, v)
        self.vig = 1.0 - vignette * (U * U + V * V) * 0.5

    def render(self, x, y, yaw):
        """返回 (H,W) uint8：白=255，黑≈20~60"""
        d = _rays(yaw)                                   # (H,W,3)
        ox = x + CAM_FWD * np.cos(yaw)
        oy = y + CAM_FWD * np.sin(yaw)
        oz = CAM_H

        dz = d[:, :, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(dz < -1e-6, -oz / dz, np.inf)   # 打到地面 z=0
        px = ox + d[:, :, 0] * t
        py = oy + d[:, :, 1] * t
        hit = np.isfinite(t)

        alb = np.full(d.shape[:2], 0.06)
        alb[hit] = np.where(self.track.is_white(px[hit], py[hit]), 0.85, 0.06)

        # 远处稍微暗一点（真实镜头的衰减/地面漫反射）
        dist = np.where(hit, t, 50.0)
        att = 1.0 / (1.0 + 0.035 * dist ** 1.2)
        rad = alb * att * self.vig

        # 自动曝光（AEC）：把最亮的（白路面）拉到接近饱和
        p = np.percentile(rad, 97.0)
        if p > 1e-6:
            gain = 1.05 / p                    # 略微过曝 ⇒ 白路面饱和到 255（和实拍一致）
            rad = rad * gain
        img = np.clip(rad * 255.0, 0, 255)

        if self.saturate:
            img = np.clip(img + 25.0, 0, 255)      # 白路面顶到 255（和实拍一样过曝一点点）

        out = Image.fromarray(img.astype(np.uint8), mode='L')
        if self.blur:
            out = out.filter(ImageFilter.GaussianBlur(self.blur))
        a = np.asarray(out, dtype=np.float32)
        a += self.rng.normal(0.0, self.noise, a.shape)
        return np.clip(a, 0, 255).astype(np.uint8)


if __name__ == '__main__':
    import os
    tr = Track()
    cam = Camera(tr)
    poses = [
        ('start',    *tr.start_pose()),
        ('cross',    2.2, 0.0, 0.0),
        ('ring_in',  2.2, 6.0, 0.0),
        ('ring_on',  3.2, 7.15, np.pi / 2),
        ('turn',     6.2, 0.4, np.deg2rad(40)),
    ]
    os.makedirs('out', exist_ok=True)
    for name, x, y, yaw in poses:
        img = cam.render(x, y, yaw)
        Image.fromarray(img).resize((W * 3, H * 3), Image.NEAREST).save('out/cam_%s.png' % name)
        print('%-8s mean=%5.1f  sat=%4.1f%%' % (name, img.mean(), 100.0 * (img >= 250).mean()))
