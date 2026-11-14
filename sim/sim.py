# -*- coding: utf-8 -*-
"""仿真主体：车 + 相机 + IMU + 控制链（20ms 一拍）+ 5ms 物理积分

和实车的对应关系：
    · 舵机指令 = 固件的舵机位置（1700..3300），中位 2748
    · **轮胎转角 ≠ 舵机角**：中间过一道 sim/steering.py 的传动比（可按实车实测标定）
    · IMU = MPU9050/MPU9250（±250dps / ±2g / 44Hz LPF / 200Hz）
    · 控制链 = control.py（视觉基线 或 视觉+陀螺）
"""
import math
import numpy as np

from track import Track
from camera import Camera
import control as C
import steering
from imu import IMU
from ipm import IPM
from scan_bev import ScanBEV

# ---------------- 车 ----------------
WHEELBASE = 0.17            # m
V_PER_RPM = 0.0035          # 100RPM ≈ 0.35 m/s
RPM_MAX   = 270.0
MOTOR_TAU = 0.12
SERVO_LAG = 0.45            # 每 5ms 追多少（一阶）

DEFAULT_PARAMS = {
    'kp': 5.0, 'ki': 0.50, 'kd': 6.0,
    'kp_cur': 5.0, 'kd_cur': 10.0, 'ki_cur': 1.2,
    'kff': 200.0,
    # 陀螺巡线专用（vision 模式下不用）
    'k_yaw': 1.5,           # 陀螺阻尼（舵机步/ (°/s)）；仿真实测 6.0 太强，会和视觉 P 打架
    'k_hold': 12.0,         # 丢线时按航向误差打舵（步/度）
    'k_curve_ff': 0.0,      # 弯道前馈（步/(°/s)）
    'gz_curve': 8.0,        # |gz| 超过这个就认为在弯里（°/s）
}


class Sim(object):
    def __init__(self, params=None, seed=1, noise=True, controller='gyro',
                 servo_gain=None, imu_bias=0.8, fast=False):
        self.track = Track()
        self.fast = fast
        self.cam = Camera(self.track, seed=seed,
                          noise=0.0 if (fast or not noise) else 0.6, fast=fast)
        self.ipm = IPM() if controller == 'bev' else None
        self.scan = ScanBEV(self.ipm) if controller == 'bev' else C.Scan()
        self.bev = None
        self.state = C.State()
        self.steer = C.Steer() if controller in ('vision', 'bev') else C.SteerGyro()
        self.speed = C.Speed()
        self.imu = IMU(seed=seed + 3, bias_dps=imu_bias)
        self.controller = controller
        self.servo_gain = steering.SERVO_WHEEL_GAIN if servo_gain is None else servo_gain

        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)

        self.x, self.y, self.yaw = self.track.start_pose()
        self.rpm = 0.0
        self.servo_act = float(C.SERVO_CENTER)
        self.servo_cmd = float(C.SERVO_CENTER)
        self.wheel_deg = 0.0
        self.rpm_meas = 0.0
        self.duty = 0
        self.yaw_rate = 0.0

        self.t_ms = 0.0
        self.img = np.zeros((C.CAM_H, C.CAM_W), dtype=np.uint8)
        self.last_cam_ms = -1e9
        self.on_track = True
        self.off_track_ms = 0.0
        self.running = False
        self.laps = 0.0

    # ---------------- 物理 ----------------
    def _physics(self, dt):
        load = 1.0 - 0.15 * abs(self.servo_act - C.SERVO_CENTER) / 552.0
        rpm_ss = self.duty / 10000.0 * RPM_MAX * load
        self.rpm += (rpm_ss - self.rpm) * dt / MOTOR_TAU

        # 舵机本身也会慢慢跟（伺服内部位置环）
        self.servo_act += (self.servo_cmd - self.servo_act) * min(1.0, dt / 0.03)

        # ★ 舵机角 → 前轮角（传动比 + 死区 + 机械限位）
        self.wheel_deg = steering.servo_to_wheel_deg(self.servo_act, gain=self.servo_gain)

        v = self.rpm * V_PER_RPM
        delta = math.radians(self.wheel_deg)
        if abs(v) > 1e-6:
            self.yaw_rate = v * math.tan(delta) / WHEELBASE * 57.29578   # °/s
            self.yaw += math.radians(self.yaw_rate) * dt
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        else:
            self.yaw_rate = 0.0

        self.imu.update(dt, self.yaw_rate, v=v)
        self.rpm_meas = self.rpm + (np.random.normal(0, 1.2) if self.duty > 0 else 0.0)

        inside = bool(self.track.is_white(self.x, self.y))
        self.on_track = inside
        if not inside:
            self.off_track_ms += dt * 1000.0

    # ---------------- 20ms 控制拍 ----------------
    def _control(self):
        self.img = self.cam.render(self.x, self.y, self.yaw)
        if self.controller == 'bev':
            self.bev = self.ipm.apply(self.img)      # ★ 通用逆透视
            self.scan.analyze(self.bev)
        else:
            self.scan.analyze(self.img, thresh=255)

        # ★ 陀螺也能判弯：残差小但车身真的在转 ⇒ 一定在弯里
        if self.g_gyro_curve(self.p['gz_curve']):
            if self.scan.ok:
                self.scan.curved = True

        self.state.update(self.scan, self.p)
        if self.controller in ('vision', 'bev'):
            self.servo_cmd = self.steer.tick(self.scan, self.state, self.p)
        else:
            self.servo_cmd = self.steer.tick(self.scan, self.state, self.p,
                                             imu=self.imu, v=self.rpm * V_PER_RPM)
        self.duty = self.speed.tick(self.rpm_meas, int(self.t_ms), self.state.duty_max)

    def g_gyro_curve(self, thr):
        return (self.rpm > 30) and (abs(self.imu.gz) > thr)

    # ---------------- 主循环 ----------------
    def step(self, dt=0.005):
        self.t_ms += dt * 1000.0
        self._physics(dt)
        if self.t_ms - self.last_cam_ms >= 20.0:
            self.last_cam_ms = self.t_ms
            self._control()

    # ---------------- 便捷 ----------------
    def start(self, rpm=C.RUN_RPM):
        self.running = True
        self.speed.set_target(rpm)

    def stop(self):
        self.running = False
        self.speed.set_target(0)

    def telemetry(self):
        s = self.scan
        return {
            'r': int(round(self.rpm_meas)), 'duty': int(self.duty),
            'servo': int(self.servo_cmd), 'wheel': self.wheel_deg,
            'gz': self.imu.gz, 'yaw_imu': self.imu.yaw_wrap(), 'yaw': math.degrees(self.yaw),
            'e': int(round(self.steer.e)), 'cv': int(s.curved), 'ds': self.state.state,
            'rs': int(round(s.res * 10)), 'nc': s.rows_n, 'kn': s.kn,
            'en': int(round(s.e_near)), 'ef': int(round(s.e_far)),
            'x': self.x, 'y': self.y, 'on_track': self.on_track,
            'off_ms': self.off_track_ms, 'laps': self.laps,
        }
