# -*- coding: utf-8 -*-
"""IMU 模型：MPU9050（= MPU9250 兼容片，和固件 Core/Src/imu_f4.c 用的是同一颗）

固件里的配置（照抄 imu_f4.c）：
    ACCEL ±2g      (16384 LSB/g)
    GYRO  ±250dps  (131 LSB/(°/s))
    DLPF  ≈44Hz
    采样  200Hz
    yaw   = 对 gz 积分（带开机零偏标定）

这里模拟的是"你能从这颗片子上读到的数"：
    · 陀螺 z 轴：真实偏航角速度 + 零偏 + 白噪声 + 44Hz 低通
    · 加速度：重力 + 转弯向心加速度 + 噪声
    · yaw：积分（会漂！漂多少取决于零偏，这正好用来验证"能不能靠陀螺闭环"）
"""
import math

import numpy as np

GYRO_LSB_PER_DPS = 131.0
ACC_LSB_PER_G    = 16384.0


class IMU(object):
    def __init__(self, seed=7, bias_dps=0.8, noise_dps=0.12, acc_noise_g=0.01,
                 dlpf_hz=44.0):
        self.rng = np.random.default_rng(seed)
        self.bias = bias_dps                     # 零偏（°/s），随机会让 yaw 漂
        self.noise = noise_dps
        self.acc_noise = acc_noise_g
        # 44Hz 一阶低通系数（在 update 里用 dt 算）
        self.dlpf_hz = dlpf_hz
        self.gz_lp = 0.0
        self.yaw_deg = 0.0
        self.gx = self.gy = self.gz = 0.0        # 低通后的角速度（°/s）
        self.ax = self.ay = self.az = 0.0        # 加速度（g）

    def update(self, dt, true_yaw_rate_dps, v=0.0, accel_forward=0.0):
        """dt 秒；true_yaw_rate_dps = 车实际偏航角速度；v = 车速(m/s)"""
        a = min(1.0, 2.0 * math.pi * self.dlpf_hz * dt)   # 一阶低通
        gz = true_yaw_rate_dps + self.bias + self.rng.normal(0.0, self.noise)
        self.gz_lp += (gz - self.gz_lp) * a
        self.gz = self.gz_lp
        self.gx = self.rng.normal(0.0, 0.05)
        self.gy = self.rng.normal(0.0, 0.05)

        # 加速度：z 轴向上 → 静止时 az ≈ +1g；转弯有向心加速度 ay
        centripetal = v * math.radians(true_yaw_rate_dps)
        self.ax = accel_forward + self.rng.normal(0.0, self.acc_noise)
        self.ay = centripetal + self.rng.normal(0.0, self.acc_noise)
        self.az = 1.0 + self.rng.normal(0.0, self.acc_noise)

        self.yaw_deg += self.gz * dt

    # ---- 按固件的整数格式给（遥测里就是这么发的）----
    def gyro_raw(self):
        return (int(round(self.gx * GYRO_LSB_PER_DPS)),
                int(round(self.gy * GYRO_LSB_PER_DPS)),
                int(round(self.gz * GYRO_LSB_PER_DPS)))

    def acc_raw(self):
        return (int(round(self.ax * ACC_LSB_PER_G)),
                int(round(self.ay * ACC_LSB_PER_G)),
                int(round(self.az * ACC_LSB_PER_G)))

    def yaw_wrap(self):
        y = math.fmod(self.yaw_deg, 360.0)
        return y + 360.0 if y < 0 else y
