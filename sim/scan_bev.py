# -*- coding: utf-8 -*-
"""在**俯视图（BEV）**上巡线

为什么更好：俯视图里赛道是**等宽**的（实测 0.5m 赛道在 10mm/px 下每行都是 49~50 像素），
所以"弯道里行宽变宽 / 中线被弦中点带偏 / 近处宽远处窄"这些透视误差**根本不存在**，
只需要：每一行找到赛道那一段 → 取中点 → 和本车所在列（画面中心列）比较。

输出和原来的 Scan 保持同样的接口（ok / e_near / e_far / rows_n / k / kn），
单位统一换算成"等效相机像素"（PX_PER_M），这样原来的 PID 增益还能直接沿用。
"""
import numpy as np

from ipm import IPM


class ScanBEV(object):
    def __init__(self, ipm: IPM, near_x=0.70, far_x=1.30, px_per_m=250.0):
        self.ipm = ipm
        self.col0 = ipm.ny // 2
        self.near_row = int(ipm.row_of_x(near_x))
        self.far_row = int(ipm.row_of_x(far_x))
        self.px_per_m = px_per_m
        # 1 行 = res 米 ⇒ 斜率 (等效px/行)
        self.px_per_row = px_per_m * ipm.res

        self.ok = False
        self.curved = False
        self.k = 0.0
        self.b = 0.0
        self.kn = 0.0
        self.res = 0.0
        self.rows_n = 0
        self.e_near = 0.0
        self.e_far = 0.0
        self.center = np.full(ipm.nx, 0xFF, dtype=np.int32)
        self.width = np.zeros(ipm.nx, dtype=np.int32)
        self.e_m = 0.0           # 最近的横向偏差（米，正=车偏左）

    # ---------------- 一行里的白色段 ----------------
    @staticmethod
    def _runs(idx, gap=2):
        if idx.size == 0:
            return []
        out = []
        s = idx[0]
        e = idx[0]
        for k in idx[1:]:
            if k - e - 1 <= gap:
                e = k
            else:
                out.append((int(s), int(e)))
                s = e = k
        out.append((int(s), int(e)))
        return out

    def analyze(self, bev, thresh=250):
        white = bev >= thresh
        self.center[:] = 0xFF
        self.width[:] = 0
        prev = None
        n = 0
        started = False

        for r in range(self.ipm.nx):
            idx = np.flatnonzero(white[r])
            if idx.size == 0:
                if started:
                    break            # 远处断了就停（不猜）
                continue
            started = True
            runs = self._runs(idx)
            pick = None
            if prev is None:
                # 第一行：选包含中心列的那段，否则选最宽的段
                for (a, b) in runs:
                    if a <= self.col0 <= b:
                        pick = (a, b)
                        break
                if pick is None:
                    pick = max(runs, key=lambda ab: ab[1] - ab[0])
            else:
                # 之后：选和上一行中心重叠的段（8 邻域连通），取并集
                cand = [(a, b) for (a, b) in runs if a - 3 <= prev <= b + 3]
                if cand:
                    pick = (min(a for a, _ in cand), max(b for _, b in cand))
                else:
                    break
            a, b = pick
            if (b - a) < 4:
                continue
            self.center[r] = (a + b) // 2
            self.width[r] = b - a + 1
            prev = self.center[r]
            n += 1

        self.rows_n = n
        if n < 8:
            self.ok = False
            return False

        rows = np.flatnonzero(self.center != 0xFF)
        cen = self.center[rows].astype(np.float64)

        # 误差（等效相机像素，右为正 → 和舵机符号约定一致）
        r_n = min(max(self.near_row, rows[0]), rows[-1])
        r_f = min(max(self.far_row, rows[0]), rows[-1])
        c_n = float(self.center[r_n])
        c_f = float(self.center[r_f])
        e_n = (c_n - self.col0) * self.px_per_m * self.ipm.res
        e_f = (c_f - self.col0) * self.px_per_m * self.ipm.res
        # ★ 符号约定要和**相机透视那条链**一致：目标在左 ⇒ e 为正（仿真/固件都是这个约定）
        self.e_near, self.e_far = e_n, e_f
        self.e_m = e_n / self.px_per_m

        # 斜率（等效px/行）与残差
        k, b = np.polyfit(rows.astype(np.float64), cen, 1)
        self.k = float(-k * self.px_per_row)
        self.b = 0.0
        res_px = float(np.max(np.abs(k * rows + b - cen))) * self.px_per_m * self.ipm.res
        self.res = res_px
        self.curved = res_px > 1.6

        # 近处斜率（最近 20 行）
        sel = rows >= max(rows[-1] - 20, rows[0])
        if sel.sum() >= 6:
            kn, _ = np.polyfit(rows[sel].astype(np.float64), cen[sel], 1)
            self.kn = float(-kn * self.px_per_row)
        else:
            self.kn = self.k

        self.ok = True
        return True
