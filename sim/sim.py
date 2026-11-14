# -*- coding: utf-8 -*-
"""仿真主体：车 + 相机 + 控制链（20ms 一拍）+ 1ms 物理积分

被两个入口用：
  · tools/sim_test.py   无界面跑 N 秒，检查"能不能跑完一圈、会不会出赛道"
  · sim/main.py         开窗口看 3D 追车视角 + 车载相机 + 遥测，并提供和实车一样的 TCP 调参协议
"""
import math
import numpy as np

from track import Track, TRACK_HALF_W
from camera import Camera
import control as C

# ---------------- 车（和实车同量级：轮距 0.17m、100RPM≈0.35m/s）----------------
WHEELBASE   = 0.17          # m
V_PER_RPM   = 0.0035        # 电机 100RPM ⇒ 0.35 m/s
RPM_MAX     = 270.0         # 满占空比空转转速
MOTOR_TAU   = 0.12          # s 电机一阶时间常数
SERVO_DEG_PER_UNIT = 22.0 / 552.0   # 舵机 ±552 步 = ±22°
SERVO_LAG   = 0.45          # 每 20ms 追多少（一阶）

DEFAULT_PARAMS = {
    'kp': 5.0, 'ki': 0.50, 'kd': 6.0,
    'kp_cur': 5.0, 'kd_cur': 10.0, 'ki_cur': 1.2,
    'kff': 200.0,
}


class Sim(object):
    def __init__(self, params=None, seed=1, noise=True):
        self.track = Track()
        self.cam = Camera(self.track, seed=seed, noise=0.6 if noise else 0.0)
        self.scan = C.Scan()
        self.state = C.State()
        self.steer = C.Steer()
        self.speed = C.Speed()
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)

        self.x, self.y, self.yaw = self.track.start_pose()
        self.rpm = 0.0
        self.servo_act = C.SERVO_CENTER
        self.servo_cmd = C.SERVO_CENTER
        self.rpm_meas = 0.0
        self.duty = 0

        self.t_ms = 0.0
        self.img = np.zeros((C.CAM_H, C.CAM_W), dtype=np.uint8)
        self.last_cam_ms = -1e9
        self.on_track = True
        self.off_track_ms = 0.0
        self.running = False

    # ---------------- 物理 ----------------
    def _physics(self, dt):
        # 电机：占空比 → 转速（一阶），加一点负载让弯道掉速更真实
        load = 1.0 - 0.15 * abs(self.servo_act - C.SERVO_CENTER) / 552.0
        rpm_ss = self.duty / 10000.0 * RPM_MAX * load
        self.rpm += (rpm_ss - self.rpm) * dt / MOTOR_TAU

        # 舵机：目标 → 实际（一阶滞后）
        self.servo_act += (self.servo_cmd - self.servo_act) * min(1.0, dt / 0.03)

        # 自行车模型
        v = self.rpm * V_PER_RPM
        delta = (self.servo_act - C.SERVO_CENTER) * SERVO_DEG_PER_UNIT
        delta = math.radians(max(-25.0, min(25.0, delta)))
        if abs(v) > 1e-6:
            self.yaw += v * math.tan(delta) / WHEELBASE * dt
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt

        # 编码器（量化 + 噪声）
        self.rpm_meas = self.rpm + (np.random.normal(0, 1.2) if self.duty > 0 else 0.0)

        # 出界统计
        inside = bool(self.track.is_white(self.x, self.y))
        self.on_track = inside
        if not inside:
            self.off_track_ms += dt * 1000.0

    # ---------------- 20ms 控制拍 ----------------
    def _control(self):
        self.img = self.cam.render(self.x, self.y, self.yaw)
        self.scan.analyze(self.img, thresh=255)
        self.state.update(self.scan, self.p)
        self.servo_cmd = self.steer.tick(self.scan, self.state, self.p)
        self.duty = self.speed.tick(self.rpm_meas, int(self.t_ms), self.state.duty_max)

    def step(self, dt=0.001):
        """推进 dt 秒（内部把控制拍对齐到 20ms）"""
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
            'servo': int(self.servo_cmd), 'yaw': math.degrees(self.yaw),
            'e': int(round(self.steer.e)), 'cv': int(s.curved), 'ds': self.state.state,
            'rs': int(round(s.res * 10)), 'nc': s.rows_n, 'kn': s.kn,
            'en': int(round(s.e_near)), 'ef': int(round(s.e_far)),
            'x': self.x, 'y': self.y, 'on_track': self.on_track,
            'off_ms': self.off_track_ms,
        }
