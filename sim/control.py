# -*- coding: utf-8 -*-
"""控制链（**照固件 1:1 移植**，FW=2214 + PC3）

移植来源（行号是回档后仓库里的位置）：
  · Core/Src/cam_f4.c   : lf_trace_track() / lf_row_runs() / cam_track_eval()
  · Core/Src/servo_f4.c : ctrl_steer_from_cam()（PD + 条件积分 + 走向前馈 + 限幅 + 回转速度）
  · Core/Src/pid_f4.c   : ctrl_speed_tick()（设定值斜坡 + PI + 前馈 + 起转踹 + 占空比上限）
  · Core/Src/state_f4.c : drv_state_update()（直道/弯道/没线 ⇒ 权重、增益、占空比上限）

所有常量都和固件里的一致，改动这里就是"在仿真里调车"，调好了把数字抄回固件。
"""
import numpy as np

# ---- 图像 ----
CAM_W, CAM_H = 188, 120

# ---- 舵机（固件 CTRL_SERVO_POS_*）----
SERVO_CENTER = 2748
SERVO_MIN    = 1700
SERVO_MAX    = 3300

# ---- 舵机 PID 常量（servo_f4.c）----
STEER_R_AHEAD  = 75
STEER_R_NEAR   = 112
STEER_W_STR    = 0.20
STEER_W_CUR    = 0.80
STEER_I_LIMIT  = 200.0
STEER_I_LEAK   = 0.985
STEER_E_LIMIT  = 90.0
STEER_D_LIMIT  = 60.0
STEER_DE_LIMIT = 5.0
STEER_SLEW     = 400

# ---- 速度环常量（ctrl_f4.h）----
RUN_RPM        = 100
KP_X1000       = 2500
KI_X1000       = 450
I_LIMIT        = 3500
FF_PER_1000    = 5000
FF_OFFSET      = 1200
RAMP_RPM_S     = 3000
DUTY_MAX       = 5000
DUTY_MAX_CUR   = 8000
KICK_DUTY      = 4000
KICK_WIN_MS    = 2400
STALL_RPM      = 50
STALL_MS       = 3000
STALL_GRACE_MS = 2500

# ---- 状态机的判据 ----
CURVED_RES     = 1.6


# ===========================================================================================
# 扫线：逐行白段跟踪（lf_row_runs + lf_trace_track）
# ===========================================================================================
LF_GAP_MERGE = 4
LF_MAX_RUNS  = 12
LF_SEED_ROW  = 112          # 算法行号（未翻转时=物理行）


def _row_runs(row):
    """一行 → [(start, end)]；段间黑缝 ≤ LF_GAP_MERGE 合并"""
    idx = np.flatnonzero(row)
    if idx.size == 0:
        return []
    runs = []
    s = idx[0]
    e = idx[0]
    for k in idx[1:]:
        if k - e - 1 <= LF_GAP_MERGE:
            e = k
        else:
            runs.append((int(s), int(e)))
            if len(runs) >= LF_MAX_RUNS:
                return runs
            s = k
            e = k
    runs.append((int(s), int(e)))
    return runs[:LF_MAX_RUNS]


class Scan(object):
    def __init__(self):
        self.ok = False
        self.bound_l = np.full(CAM_H, 0xFF, dtype=np.int32)
        self.bound_r = np.full(CAM_H, 0xFF, dtype=np.int32)
        self.k = 0.0
        self.b = CAM_W * 0.5
        self.res = 0.0
        self.curved = False
        self.rows_n = 0
        self.kn = 0.0
        self.e_near = 0.0
        self.e_far = 0.0

    def analyze(self, img_gray, thresh=255):
        """img_gray: (120,188) uint8；返回 True = 本帧有中线"""
        white = img_gray >= thresh
        self.ok = False
        self.bound_l[:] = 0xFF
        self.bound_r[:] = 0xFF
        self.rows_n = 0

        # ---- 种子：离车最近的那一行 ----
        r0 = min(max(LF_SEED_ROW, 0), CAM_H - 1)
        row = white[r0]
        idx = np.flatnonzero(row)
        if idx.size < 2:
            return False
        l, rg = int(idx[0]), int(idx[-1])
        if rg <= l:
            return False

        self.bound_l[r0] = l
        self.bound_r[r0] = rg

        # ---- 逐行往上爬：只认同"和上一行区间有重叠"的白段，取并集 ----
        for r in range(r0 - 1, -1, -1):
            runs = _row_runs(white[r])
            ul = ur = -1
            for (a, b) in runs:
                ov = min(b, rg) - max(a, l) + 1
                if ov <= 0:
                    continue
                ul = a if (ul < 0 or a < ul) else ul
                ur = b if (ur < 0 or b > ur) else ur
            if ul < 0 or ur < 0:
                break
            l, rg = ul, ur
            self.bound_l[r] = l
            self.bound_r[r] = rg

        # ---- 中线 + 直线拟合（cam_track_eval）----
        rows = np.flatnonzero((self.bound_l != 0xFF) & (self.bound_r != 0xFF))
        if rows.size < 8:
            return False
        cen = (self.bound_l[rows] + self.bound_r[rows]) * 0.5
        k, b = np.polyfit(rows.astype(np.float64), cen, 1)
        res = float(np.max(np.abs(k * rows + b - cen)))
        r_lo, r_hi = int(rows[0]), int(rows[-1])

        self.k = float(k)
        self.b = float(b)
        self.res = res
        self.curved = res > CURVED_RES
        self.rows_n = int(rows.size)
        self.e_near = float(k * r_hi + b - CAM_W * 0.5)
        self.e_far = float(k * r_lo + b - CAM_W * 0.5)

        # 近处斜率（只用离车最近 20 行）
        r0n = max(r_hi - 20, r_lo)
        sel = (rows >= r0n) & (rows <= r_hi)
        if sel.sum() >= 6:
            kn, _ = np.polyfit(rows[sel].astype(np.float64), cen[sel], 1)
            self.kn = float(kn)
        else:
            self.kn = float(k)
        self.ok = True
        return True


# ===========================================================================================
# 行驶状态（state_f4.c）
# ===========================================================================================
class State(object):
    def __init__(self):
        self.state = 0            # 0=LOST 1=STRAIGHT 2=CURVE
        self.w = STEER_W_STR
        self.kp = 5.0
        self.kd = 6.0
        self.ki = 0.50
        self.duty_max = DUTY_MAX

    def update(self, scan, p):
        if (not scan.ok) or (scan.rows_n < 8):
            self.state = 0
        elif scan.curved:
            self.state = 2
        else:
            self.state = 1

        if self.state == 2:
            self.w = STEER_W_CUR
            self.kp = p['kp_cur']
            self.kd = p['kd_cur']
            self.ki = p['ki_cur']
            self.duty_max = DUTY_MAX_CUR
        else:
            self.w = STEER_W_STR
            self.kp = p['kp']
            self.kd = p['kd']
            self.ki = p['ki']
            self.duty_max = DUTY_MAX

    def name(self):
        return ('LOST', 'STR', 'CUR')[self.state]


# ===========================================================================================
# 舵机（servo_f4.c）
# ===========================================================================================
class Steer(object):
    def __init__(self):
        self.i = 0.0
        self.e_prev = 0.0
        self.e_ok = False
        self.last = SERVO_CENTER
        self.target = SERVO_CENTER
        self.e = 0.0
        self.ierr = 0.0

    def tick(self, scan, st, p):
        if not scan.ok:
            # 没线：积分慢慢泄放，舵机保持
            self.i *= STEER_I_LEAK
            self.ierr = self.i
            self.target = self.last
            return self.target

        e = scan.e_near * (1.0 - st.w) + scan.e_far * st.w
        e = max(-STEER_E_LIMIT, min(STEER_E_LIMIT, e))

        if not self.e_ok:
            self.e_prev = e
            self.e_ok = True
        de = e - self.e_prev
        self.e_prev = e

        kp, ki, kd = st.kp, st.ki, st.kd

        i_try = self.i
        if -20.0 < e < 20.0:
            i_try += e
        else:
            i_try *= 0.92
        i_try = max(-STEER_I_LIMIT, min(STEER_I_LIMIT, i_try))

        de = max(-STEER_DE_LIMIT, min(STEER_DE_LIMIT, de))
        dterm = max(-STEER_D_LIMIT, min(STEER_D_LIMIT, kd * de))

        u = kp * e + ki * i_try + dterm
        ff = -p['kff'] * (scan.kn * 60.0) / 100.0
        ff = max(-500.0, min(500.0, ff))
        u += ff

        self.e = e
        target = int(SERVO_CENTER + u)

        if target < SERVO_MIN or target > SERVO_MAX:
            self.i *= 0.95
        else:
            self.i = i_try
        self.ierr = self.i

        target = max(SERVO_MIN, min(SERVO_MAX, target))
        target = max(self.last - STEER_SLEW, min(self.last + STEER_SLEW, target))
        self.last = target
        self.target = target
        return target


# ===========================================================================================
# 速度环（pid_f4.c）
# ===========================================================================================
class Speed(object):
    def __init__(self):
        self.target_x10 = 0
        self.ramp_x10 = 0
        self.i_acc = 0
        self.duty = 0
        self.err_rpm = 0
        self.meas_f_x10 = 0
        self.kick_until = 0
        self.kick_act = False
        self.tgt_prev = 0

    def set_target(self, rpm):
        self.target_x10 = int(rpm * 10)
        self.i_acc = 0

    def tick(self, rpm_meas, t_ms, duty_max):
        if self.target_x10 == 0:
            self.ramp_x10 = 0
            self.i_acc = 0
            self.duty = 0
            self.err_rpm = 0
            self.kick_act = False
            return 0

        step = int(RAMP_RPM_S * 20 / 100)       # 每拍增量（20ms）
        if self.ramp_x10 < self.target_x10:
            self.ramp_x10 = min(self.ramp_x10 + step, self.target_x10)
        elif self.ramp_x10 > self.target_x10:
            self.ramp_x10 = max(self.ramp_x10 - step, self.target_x10)

        meas_x10 = int(rpm_meas * 10)
        self.meas_f_x10 += (meas_x10 - self.meas_f_x10) // 4
        meas_x10 = self.meas_f_x10

        err_x10 = self.ramp_x10 - meas_x10
        self.err_rpm = int(err_x10 / 10)

        p_term = (err_x10 * KP_X1000) // 10000
        i_step = (err_x10 * KI_X1000) // 10000
        self.i_acc += i_step
        self.i_acc = max(-I_LIMIT, min(I_LIMIT, self.i_acc))

        ff_term = (self.ramp_x10 * FF_PER_1000) // 10000
        if self.ramp_x10 > 0:
            ff_term += FF_OFFSET
        elif self.ramp_x10 < 0:
            ff_term -= FF_OFFSET

        out = ff_term + p_term + self.i_acc
        out = min(out, duty_max)
        out = max(out, 0)

        if self.tgt_prev == 0 and self.target_x10 != 0:
            self.kick_until = t_ms + KICK_WIN_MS
            self.kick_act = True
        self.tgt_prev = self.target_x10
        if self.kick_act:
            if t_ms < self.kick_until:
                out = max(out, KICK_DUTY)
            else:
                self.kick_act = False

        self.duty = out
        return out
