# -*- coding: utf-8 -*-
"""车载相机（3D 透视 → 188×120 灰度图）

真车那颗总钻风是 **188×120、装车头前部、略微低头** 看的。这里用射线投射
（每个像素一条射线 → 打在地面 z=0 上 → 查赛道的占用图）得到**真实透视**的画面：
近处宽、远处窄、弯道会向右/左偏，和实拍完全一个形状。

成像链（尽量贴近实拍）：
  反射率 albedo(白 0.85 / 黑 0.06) → 光照衰减 → 自动曝光(AEC，把亮的顶到 255) →
  镜头暗角 → 模糊 → 噪声 → uint8
选自动曝光是因为实车那颗 MT9V034 的 AEC 只能开、不能关（我们踩过坑），
所以画面亮度总是"白的地方刚好饱和"，固件里默认阈值 255 就是靠这个工作。

★ 加速（FW 仿真专用）：射线方向在**相机坐标系**里只算一次，每帧只做一次 3×3 旋转；
  模糊改成 numpy 的 3 抽头盒式滤波；`fast=True` 时不清噪（优化/批量跑用）。
"""
import numpy as np

from track import Track

W, H = 188, 120

CAM_FWD   = 0.20      # 相机在车中心**前方**多远（米）
CAM_H     = 0.13      # 相机离地高度（米）
CAM_PITCH = np.deg2rad(18.0)   # 低头角度
CAM_HFOV  = np.deg2rad(60.0)   # 水平视场


def _basis(yaw):
    """相机朝向：世界 z 向上，yaw=0 朝 +x。返回 前/右/上 三个基向量（前方向带低头角）"""
    cp, sp = np.cos(CAM_PITCH), np.sin(CAM_PITCH)
    cy, sy = np.cos(yaw), np.sin(yaw)
    fwd = np.array([cy * cp, sy * cp, -sp])
    right = np.array([-sy, cy, 0.0])
    up = np.cross(fwd, right)
    return fwd, right, up


def _rays_cam(w=W, h=H):
    """相机坐标系下的单位射线 (h,w,3)：x 右 / y 上 / z 前"""
    tx = np.tan(CAM_HFOV * 0.5)
    ty = tx * (float(h) / float(w))
    u = (np.arange(w) + 0.5) / w * 2.0 - 1.0
    v = 1.0 - (np.arange(h) + 0.5) / h * 2.0
    U, V = np.meshgrid(u, v)
    d = np.stack([U * tx, V * ty, np.ones_like(U)], axis=2).astype(np.float32)
    d /= np.linalg.norm(d, axis=2, keepdims=True)
    return d


class Camera(object):
    def __init__(self, track: Track, size=(W, H), seed=12345, noise=0.6,
                 vignette=0.35, blur=1, saturate=True, fast=False):
        self.track = track
        self.size = size
        self.rng = np.random.default_rng(seed)
        self.noise = 0.0 if fast else noise
        self.blur = 0 if fast else blur
        self.saturate = saturate
        self.fast = fast
        self._d_cam = _rays_cam(size[0], size[1])

        # 暗角模板（一次算好）
        u = (np.arange(size[0]) + 0.5) / size[0] * 2.0 - 1.0
        v = (np.arange(size[1]) + 0.5) / size[1] * 2.0 - 1.0
        U, V = np.meshgrid(u, v)
        self.vig = (1.0 - vignette * (U * U + V * V) * 0.5).astype(np.float32)

    def _box3(self, a):
        """3 抽头盒式模糊（横+竖各一次），比 PIL 高斯快一个数量级"""
        a = (a + np.roll(a, 1, axis=1) + np.roll(a, -1, axis=1)) * (1.0 / 3.0)
        a = (a + np.roll(a, 1, axis=0) + np.roll(a, -1, axis=0)) * (1.0 / 3.0)
        return a

    def render(self, x, y, yaw):
        """返回 (H,W) uint8：白=255，黑≈20~60"""
        fwd, right, up = _basis(yaw)
        R = np.stack([right, up, fwd], axis=1).astype(np.float32)     # 列向量
        d = self._d_cam @ R.T                                         # (H,W,3) 世界系

        ox = x + CAM_FWD * np.cos(yaw)
        oy = y + CAM_FWD * np.sin(yaw)
        oz = CAM_H

        dz = d[:, :, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(dz < -1e-6, -oz / dz, np.inf)
        hit = np.isfinite(t)
        px = ox + d[:, :, 0] * t
        py = oy + d[:, :, 1] * t

        alb = np.full(d.shape[:2], 0.06, dtype=np.float32)
        hx, hy = px[hit], py[hit]
        alb[hit] = np.where(self.track.is_white(hx, hy), 0.85, 0.06)

        dist = np.where(hit, t, 50.0)
        att = 1.0 / (1.0 + 0.035 * dist ** 1.2)
        rad = alb * att * self.vig

        p = np.percentile(rad, 97.0)
        if p > 1e-6:
            rad = rad * (1.05 / p)
        img = rad * 255.0
        if self.saturate:
            img = img + 25.0
        np.clip(img, 0, 255, out=img)

        if self.blur:
            img = self._box3(img)
        if self.noise > 0.0:
            img += self.rng.normal(0.0, self.noise, img.shape)
            np.clip(img, 0, 255, out=img)
        return img.astype(np.uint8)


if __name__ == '__main__':
    import os
    import time
    from PIL import Image

    tr = Track()
    cam = Camera(tr)
    camf = Camera(tr, fast=True)
    poses = [('start', *tr.start_pose()), ('cross', 2.2, 0.0, 0.0),
             ('ring', 2.2, 6.0, 0.0), ('turn', 6.2, 0.4, np.deg2rad(40))]
    os.makedirs('out', exist_ok=True)
    for name, x, y, yaw in poses:
        img = cam.render(x, y, yaw)
        Image.fromarray(img).resize((W * 3, H * 3), Image.NEAREST).save('out/cam_%s.png' % name)
        print('%-8s mean=%5.1f  sat=%4.1f%%' % (name, img.mean(), 100.0 * (img >= 250).mean()))

    t0 = time.perf_counter()
    for _ in range(200):
        camf.render(1.0, 0.0, 0.0)
    print('fast 相机 %.2f ms/帧' % ((time.perf_counter() - t0) / 200 * 1000))
    t0 = time.perf_counter()
    for _ in range(200):
        cam.render(1.0, 0.0, 0.0)
    print('标准 相机 %.2f ms/帧' % ((time.perf_counter() - t0) / 200 * 1000))
