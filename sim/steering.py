# -*- coding: utf-8 -*-
"""舵机角 → 轮胎（前轮）转角 的传动模型

**为什么需要它**：实车上"舵机转了多少度"和"前轮实际转了多少度"根本不是一回事：
  · 舵机摇臂 → 拉杆 → 转向节，是一套连杆机构，**传动比 < 1**（前轮转得比舵机少）
  · 连杆过中点附近时有**死区/间隙**（齿轮间隙 + 球头旷量）
  · 到极限时**先到机械限位**（前轮比舵机早饱和）
固件里 PID 输出的是**舵机位置**（1700..3300，中位 2748），这里再换算成轮胎角，
仿真出来的车才会有"打舵打了半天、车没转多少"的真实感觉。

参数含义（想改成自己的车，就改这 4 个）：
  SERVO_MAX_DEG     舵机自身的最大转角（±，按 552 步对应多少度算）
  SERVO_WHEEL_GAIN  ★ 传动比：轮胎角 = 舵机角 × 这个系数（实车实测法见下）
  WHEEL_DEADBAND    死区（度）：轮胎角小于它就不动（间隙）
  WHEEL_MAX_DEG     前轮机械限位（±）
"""
import math

SERVO_CENTER = 2748.0
SERVO_RANGE  = 552.0        # 中位到极限的步数（3300-2748）

SERVO_MAX_DEG    = 22.0     # 舵机自身 ±22°
SERVO_WHEEL_GAIN = 0.62     # ★ 舵机角 → 前轮角（0.62 表示前轮只转舵机的 62%）
WHEEL_DEADBAND   = 0.8      # 死区（度）
WHEEL_MAX_DEG    = 26.0     # 前轮机械限位


def servo_deg(units):
    """舵机自身转角（度）"""
    return (float(units) - SERVO_CENTER) * (SERVO_MAX_DEG / SERVO_RANGE)


def servo_to_wheel_deg(units, gain=None, deadband=None, wmax=None):
    """舵机位置 → **前轮实际转角（度）**，带传动比 / 死区 / 机械限位"""
    g = SERVO_WHEEL_GAIN if gain is None else gain
    db = WHEEL_DEADBAND if deadband is None else deadband
    wm = WHEEL_MAX_DEG if wmax is None else wmax

    d = servo_deg(units) * g
    if abs(d) <= db:
        return 0.0
    d -= math.copysign(db, d)          # 死区是"吃掉"一小段，不是整体平移
    return max(-wm, min(wm, d))


def calibrate_gain(wheel_deg_at_full, servo_deg_at_full=None):
    """实车怎么量这个系数：
       把舵机打到最左/最右，用量角器（或手机水平仪）量前轮的转角 wheel_deg_at_full，
       舵机那侧的角 servo_deg_at_full 默认取 SERVO_MAX_DEG。
       ⇒ 传动比 = 前轮角 / 舵机角。
       例：舵机 ±22°，前轮只到 ±13.6° ⇒ gain = 13.6/22 = 0.62（本文件默认值）"""
    s = SERVO_MAX_DEG if servo_deg_at_full is None else servo_deg_at_full
    return float(wheel_deg_at_full) / float(s)
