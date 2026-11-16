# -*- coding: utf-8 -*-
"""通用逆透视（IPM / Bird's-Eye View）

把**相机透视图像**映射成**车体坐标系下的俯视图**（鸟瞰），赛道在俯视图里是**等宽**的，
所以"切内弯 / 远处变宽"这类几何偏差会自然消失。

"通用"体现在：不写死任何一套相机参数，全都由构造参数给：
    img_wh      相机图像分辨率（本项目 188×120）
    hfov        水平视场（本项目 60°）
    cam_xy      相机在车体系下的位置（x 前 / y 左 / z 上，米）
    pitch       低头角（弧度）
    bev_x       (x_near, x_far) 俯视图覆盖的前向范围（米，车体系）
    bev_y       俯视图覆盖的横向半宽（米）
    res         俯视图分辨率（米/像素）
输出的俯视图：**行 = 前向距离（行越大越远），列 = 横向（列中心 = y=0）**

实现：构造时把每个俯视图像素反算回相机像素坐标（remap 表，一次算好），
之后每帧只做一次查表采样（numpy 花式索引），开销 <1ms。
"""
import numpy as np


class IPM(object):
    def __init__(self, img_wh=(188, 120), hfov=np.deg2rad(60.0),
                 cam_xy=(0.20, 0.0, 0.13), pitch=np.deg2rad(18.0),
                 bev_x=(0.05, 1.60), bev_y=0.60, res=0.01, bilinear=True, res_y=None):
        self.img_w, self.img_h = img_wh
        self.hfov = hfov
        self.cam = np.asarray(cam_xy, dtype=np.float64)     # (x 前, y 左, z 上)
        self.pitch = pitch
        self.res = res                                  # 横向 m/像素
        self.res_y = res if res_y is None else res_y    # 纵向 m/像素（可以不等）
        self.bilinear = bilinear
        self.bev_x0, self.bev_x1 = bev_x
        self.bev_y = bev_y

        # ---- 输出网格（车体系，米）----
        self.nx = int(round((bev_x[1] - bev_x[0]) / self.res_y)) + 1   # 行（前向）
        self.ny = int(round(2.0 * bev_y / res)) + 1                    # 列（横向）
        xs = bev_x[0] + np.arange(self.nx) * self.res_y                # 每行对应的前向距离
        ys = -bev_y + np.arange(self.ny) * res                         # 每列对应的横向位置
        # 车体系：X 前, Y 左, Z 上；BEV 行 = X，列 = Y
        X, Y = np.meshgrid(xs, ys, indexing='ij')

        # ---- 地面点 → 相机坐标 ----
        # 相机基（车体系）：z_cam 沿光轴（前下方），x_cam 向右，y_cam 向上
        cp, sp = np.cos(pitch), np.sin(pitch)
        z_cam = np.array([cp, 0.0, -sp])          # 光轴（前下方）
        x_cam = np.array([0.0, 1.0, 0.0])         # 相机 x 轴 = 车体 +Y（本工程相机 u 增大的方向）
        y_cam = np.cross(z_cam, x_cam)            # 相机 y 轴（上）
        self.R = np.stack([x_cam, y_cam, z_cam], axis=1)    # 列 = 相机轴

        P = np.stack([X - self.cam[0], Y - self.cam[1], np.zeros_like(X) - self.cam[2]],
                     axis=-1)                      # 相对相机位置（车体系）
        Pc = P @ self.R                            # 世界→相机（R^T 作用）

        z = Pc[..., 2]
        self.valid = z > 1e-3                      # 地平线以上 = 无效

        fx = (self.img_w * 0.5) / np.tan(hfov * 0.5)
        fy = fx
        u = Pc[..., 0] / np.where(self.valid, z, 1.0) * fx + self.img_w * 0.5
        v = self.img_h * 0.5 - Pc[..., 1] / np.where(self.valid, z, 1.0) * fy
        self.map_u = u.astype(np.float32)
        self.map_v = v.astype(np.float32)

    # ---------------- 采样 ----------------
    def apply(self, img):
        """img: (img_h, img_w) uint8 → 俯视图 (nx, ny) uint8（无效处=0）"""
        h, w = img.shape
        if self.bilinear:
            u = self.map_u
            v = self.map_v
            u0 = np.floor(u).astype(np.int32)
            v0 = np.floor(v).astype(np.int32)
            inside = self.valid & (u0 >= 0) & (u0 < w - 1) & (v0 >= 0) & (v0 < h - 1)
            u0c = np.clip(u0, 0, w - 2)
            v0c = np.clip(v0, 0, h - 2)
            du = (u - u0c).astype(np.float32)
            dv = (v - v0c).astype(np.float32)
            a = img[v0c, u0c].astype(np.float32)
            b = img[v0c, u0c + 1].astype(np.float32)
            c = img[v0c + 1, u0c].astype(np.float32)
            d = img[v0c + 1, u0c + 1].astype(np.float32)
            top = a + (b - a) * du
            bot = c + (d - c) * du
            out = top + (bot - top) * dv
            out[~inside] = 0.0
            return out.astype(np.uint8)
        else:
            u = np.rint(self.map_u).astype(np.int32)
            v = np.rint(self.map_v).astype(np.int32)
            inside = self.valid & (u >= 0) & (u < w) & (v >= 0) & (v < h)
            out = np.zeros((self.nx, self.ny), dtype=np.uint8)
            out[inside] = img[v[inside], u[inside]]
            return out

    # ---------------- 坐标换算（调试/日志用）----------------
    def x_of_row(self, row):
        """行号 → 前向距离（米，车体系）"""
        return self.bev_x0 + row * self.res

    def y_of_col(self, col):
        """列号 → 横向位置（米，左正）"""
        return -self.bev_y + col * self.res

    def row_of_x(self, x):
        return (x - self.bev_x0) / self.res_y

    def col_of_y(self, y):
        return (y + self.bev_y) / self.res


def make_default():
    return IPM()
