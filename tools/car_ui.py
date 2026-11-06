# -*- coding: utf-8 -*-
"""
small_car 控制台  ——  图像 + 仪表 + 按钮控制
================================================================
全部靠按钮和滑条操作，**不用敲任何命令**（后台仍然走文本协议，但界面上看不到）。

  ① 图像：二值图 188×120 / 原图灰度 94×60 两个按钮切换，叠加左右边界红线
  ② 仪表：转速 / 三轴加速度 / 三轴角速度 / 偏航角 / 舵机位置（板子 10Hz 遥测）
  ③ 控制：
       电机1：**[启动] [停止]**（上电默认停，必须点"启动"才转）、[前进]/[后退] 选方向、
              快捷档 300 600 1000 1500、滑条（松手才发）
       舵机：左 / 中 / 右 / −20 / +20 / 滑条
       其它：编码器清零 / 紧急停止
  ④ 事件：右上角一行提示"最后做了什么"，底部日志显示板子的回执

协议见固件 app.c / cam_f4.c：
    AA 55 A5 5A | 0xFF ...   二值整帧（188×120，位打包 24B/行）
    AA 55 A5 5A | 0xFC ...   灰度缩略图（94×60，8 块，每块 752B）
    AA 55 A5 5A | 0xFD ...   左右边界数组（每行 2 字节）
    文本：`T ...` 遥测 / `ACK ...` / `CFG ...`

用法：
    python car_ui.py [端口]        默认 8086
"""

import socket
import sys
import time

import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image, ImageTk, ImageDraw
    HAVE_PIL = True
except Exception as _e:
    HAVE_PIL = False
    print('[错误] 需要 Pillow：pip install pillow  (%s)' % _e)

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8086

MAGIC = b'\xAA\x55\xA5\x5A'

BIN_W, BIN_H = 188, 120        # 二值图
GRAY_W, GRAY_H = 94, 60        # 灰度缩略图（2×2 抽取）
VIEW_W, VIEW_H = 752, 480      # 显示尺寸
SCALE_EDGE = 4                 # 边界坐标 → 显示坐标

# 界面配色
C_BG      = '#1e1e26'
C_PANEL   = '#26262f'
C_FG      = '#e6e6ea'
C_DIM     = '#9aa0aa'
C_BTN     = '#3a3a48'
C_BTN_ACT = '#4c4c60'
C_ON      = '#2e7d4f'          # 选中（图像模式）
C_STOP    = '#7d2a2a'
C_ESTOP   = '#b02020'
C_ACCENT  = '#4aa3ff'

# 快捷档 / 预设
# 电机控制：两种方式
#   'pwm' —— 直接给占空比（默认 30%，开环，不依赖编码器，用来先确认电机装反没/转不转）
#   'rpm' —— 走速度环闭环（需要编码器读数正确）
PWM_PRESETS = (10, 20, 30, 50)          # 占空比 %
RPM_PRESETS = (300, 600, 1000, 1500)
SERVO_LEFT, SERVO_MID, SERVO_RIGHT = 1300, 2048, 2800
SERVO_MIN, SERVO_MAX = 1048, 3048


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        return '??'
    finally:
        s.close()


class App(object):
    def __init__(self):
        self.conn = None
        self.buf = bytearray()

        # ---- 图像 ----
        self.bin_img = bytearray(BIN_W * BIN_H)
        self.gray_img = bytearray(GRAY_W * GRAY_H)
        self.gray_got = set()
        self.edges = None
        self.mode_gray = False
        self.shot = None
        self.photo = None

        # ---- 遥测 ----
        self.tele = {}
        self.t_tele = 0.0
        self.frames = 0
        self.fps = 0.0
        self.t_fps = time.time()
        self.n_fps = 0
        self.nbytes = 0
        self.bps = 0.0
        self.t_bps = time.time()
        self.n_bps = 0
        self.t_draw = 0.0

        # ---- 目标值（界面状态）----
        self.req_rpm = 0            # 想跑多少（带方向）
        self.req_servo = SERVO_MID
        self.running = False        # ★ 电机1 是否"启动"状态：上电默认停
        self.dir = +1               # 方向：+1 前进 / -1 后退
        self.ctrl_mode = 'pwm'      # ★ 默认"占空比"方式，滑条默认 30%
        self.presets = PWM_PRESETS
        self.warn_zero = False      # 只提示一次"运行但输出 0"

        # ---- 窗口 ----
        self.root = tk.Tk()
        self.root.title('small_car 控制台')
        self.root.configure(bg=C_BG)
        self.root.geometry('1180x560')
        self.build_ui()
        self.refresh_buttons()
        self.update_speed_ui()
        self.root.update()

    # ================================================================ 控件小工具
    def _btn(self, parent, text, cmd, bg=C_BTN, fg=C_FG, w=10, h=2, font=('Microsoft YaHei', 10)):
        b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                      activebackground=C_BTN_ACT, activeforeground='white',
                      relief='raised', bd=2, width=w, height=h, font=font,
                      cursor='hand2', disabledforeground='#6a6a72')
        return b

    def _lbl(self, parent, text, font=('Microsoft YaHei', 10), fg=C_FG, **kw):
        return tk.Label(parent, text=text, bg=C_PANEL, fg=fg, font=font, **kw)

    # ================================================================ 界面
    def build_ui(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # ---------------- 左：图像 ----------------
        left = tk.Frame(root, bg=C_BG, padx=8, pady=8)
        left.grid(row=0, column=0, sticky='nsew')
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)

        self.img_lbl = tk.Label(left, bg='black', bd=2, relief='sunken')
        self.img_lbl.grid(row=0, column=0, sticky='nsew')

        self.lbl_stat = tk.Label(left, text='未连接', bg=C_BG, fg=C_DIM,
                                 font=('Consolas', 9), anchor='w')
        self.lbl_stat.grid(row=1, column=0, sticky='ew', pady=(6, 0))

        # ---------------- 右：控制台 ----------------
        right = tk.Frame(root, bg=C_PANEL, padx=10, pady=8)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)

        row = 0

        # ---- 图像源 ----
        f = tk.LabelFrame(right, text=' 图像源 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='ew', pady=(0, 8)); row += 1
        self.btn_bin = self._btn(f, '二值图', lambda: self.do_mode(1), w=10)
        self.btn_bin.grid(row=0, column=0, padx=6, pady=6)
        self.btn_gray = self._btn(f, '原图灰度', lambda: self.do_mode(0), w=10)
        self.btn_gray.grid(row=0, column=1, padx=6, pady=6)
        self.lbl_mode = self._lbl(f, '当前：二值图 188×120', fg=C_DIM, font=('Microsoft YaHei', 9))
        self.lbl_mode.grid(row=1, column=0, columnspan=2, sticky='w', padx=6, pady=(0, 6))

        # ---- 仪表 ----
        f = tk.LabelFrame(right, text=' 实时数据 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='ew', pady=(0, 8)); row += 1
        f.columnconfigure(1, weight=1)

        self.v_rpm = tk.StringVar(value='---')
        self.v_acc = tk.StringVar(value='---')
        self.v_gyr = tk.StringVar(value='---')
        self.v_yaw = tk.StringVar(value='---')
        self.v_srv = tk.StringVar(value='---')
        self.v_duty = tk.StringVar(value='---')

        self._lbl(f, '转速').grid(row=0, column=0, sticky='w', padx=6)
        tk.Label(f, textvariable=self.v_rpm, bg=C_PANEL, fg='#66e08a',
                 font=('Consolas', 16, 'bold'), anchor='w').grid(row=0, column=1, sticky='w')
        for i, (k, v) in enumerate((('加速度', self.v_acc), ('角速度', self.v_gyr),
                                     ('偏航角', self.v_yaw), ('舵机', self.v_srv),
                                     ('输出占空比', self.v_duty)), start=1):
            self._lbl(f, k).grid(row=i, column=0, sticky='w', padx=6)
            tk.Label(f, textvariable=v, bg=C_PANEL, fg=C_FG,
                     font=('Consolas', 11), anchor='w').grid(row=i, column=1, sticky='w')

        # ---- 电机1 ----
        f = tk.LabelFrame(right, text=' 电机1 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='ew', pady=(0, 8)); row += 1
        for c in range(4):
            f.columnconfigure(c, weight=1)

        self.lbl_rpm = tk.Label(f, text='电机1：停止', bg=C_PANEL, fg='#ffcc66',
                                font=('Microsoft YaHei', 11, 'bold'))
        self.lbl_rpm.grid(row=0, column=0, columnspan=4, sticky='w', padx=6, pady=(4, 2))

        # ★ 控制方式：占空比（PWM %，默认 30%）或 转速（RPM，闭环）
        self.btn_mode_pwm = self._btn(f, '占空比 %', (lambda: self.set_motor_mode('pwm')),
                                      bg=C_ON, w=7, h=1)
        self.btn_mode_pwm.grid(row=1, column=0, columnspan=2, padx=3, pady=(2, 2), sticky='ew')
        self.btn_mode_rpm = self._btn(f, '转速 RPM', (lambda: self.set_motor_mode('rpm')),
                                      bg=C_BTN, w=7, h=1)
        self.btn_mode_rpm.grid(row=1, column=2, columnspan=2, padx=3, pady=(2, 2), sticky='ew')

        # ★ 启动 / 停止：上电默认停止，必须按"启动"电机才会转
        self.btn_start = self._btn(f, '启 动', self.do_start, bg=C_BTN, w=8, h=2,
                                   font=('Microsoft YaHei', 12, 'bold'))
        self.btn_start.grid(row=2, column=0, columnspan=2, padx=3, pady=3, sticky='ew')
        self.btn_stop = self._btn(f, '停 止', self.do_stop_cmd, bg=C_STOP, fg='white', w=8, h=2,
                                  font=('Microsoft YaHei', 12, 'bold'))
        self.btn_stop.grid(row=2, column=2, columnspan=2, padx=3, pady=3, sticky='ew')

        # ★ 方向（只选方向，本身不启动电机）
        self.btn_fwd = self._btn(f, '前进', lambda: self.do_dir(+1), bg=C_ON, w=7, h=1)
        self.btn_fwd.grid(row=3, column=0, padx=3, pady=(2, 2), sticky='ew')
        self.btn_back = self._btn(f, '后退', lambda: self.do_dir(-1), bg=C_BTN, w=7, h=1)
        self.btn_back.grid(row=3, column=1, padx=3, pady=(2, 2), sticky='ew')
        self.btn_zero = self._btn(f, '归零', self.do_zero, w=5, h=1)
        self.btn_zero.grid(row=3, column=2, padx=3, pady=(2, 2), sticky='ew')
        self.btn_hold = self._btn(f, '急停', self.do_estop, bg=C_ESTOP, fg='white', w=5, h=1,
                                  font=('Microsoft YaHei', 10, 'bold'))
        self.btn_hold.grid(row=3, column=3, padx=3, pady=(2, 2), sticky='ew')

        # 快捷档（文字/数值随模式变）
        self.preset_btns = []
        for i in range(4):
            b = self._btn(f, '--', (lambda idx=i: self.do_preset_idx(idx)), w=6, h=1,
                          font=('Consolas', 10))
            b.grid(row=4, column=i, padx=3, pady=(2, 4), sticky='ew')
            self.preset_btns.append(b)

        self.sld_speed = tk.Scale(f, from_=0, to=100, orient='horizontal',
                                  bg=C_PANEL, fg=C_FG, troughcolor='#3c3c4c',
                                  highlightbackground=C_PANEL, showvalue=False, length=250,
                                  command=lambda v: self.on_speed_slide())
        self.sld_speed.set(30)
        self.sld_speed.grid(row=5, column=0, columnspan=4, sticky='ew', padx=6)
        self.sld_speed.bind('<ButtonRelease-1>', lambda e: self.on_speed_release())

        # ---- 舵机 ----
        f = tk.LabelFrame(right, text=' 舵机角度 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='ew', pady=(0, 8)); row += 1
        for c in range(5):
            f.columnconfigure(c, weight=1)

        self.lbl_srv = tk.Label(f, text='目标  2048', bg=C_PANEL, fg=C_FG,
                                font=('Microsoft YaHei', 11, 'bold'))
        self.lbl_srv.grid(row=0, column=0, columnspan=5, sticky='w', padx=6, pady=(4, 2))

        self.btn_sl = self._btn(f, '左', lambda: self.do_servo(SERVO_LEFT), w=4, h=2)
        self.btn_sl.grid(row=1, column=0, padx=3, pady=3, sticky='ew')
        self.btn_sm = self._btn(f, '中', lambda: self.do_servo(SERVO_MID), bg=C_ON, w=4, h=2)
        self.btn_sm.grid(row=1, column=2, padx=3, pady=3, sticky='ew')
        self.btn_sr = self._btn(f, '右', lambda: self.do_servo(SERVO_RIGHT), w=4, h=2)
        self.btn_sr.grid(row=1, column=4, padx=3, pady=3, sticky='ew')

        self.btn_sd = self._btn(f, '−20', lambda: self.do_servo_nudge(-20), w=4, h=1)
        self.btn_sd.grid(row=2, column=0, padx=3, pady=(0, 4), sticky='ew')
        self.btn_su = self._btn(f, '+20', lambda: self.do_servo_nudge(+20), w=4, h=1)
        self.btn_su.grid(row=2, column=4, padx=3, pady=(0, 4), sticky='ew')

        self.sld_servo = tk.Scale(f, from_=SERVO_MIN, to=SERVO_MAX, orient='horizontal',
                                  bg=C_PANEL, fg=C_FG, troughcolor='#3c3c4c',
                                  highlightbackground=C_PANEL, showvalue=False, length=250)
        self.sld_servo.set(SERVO_MID)
        self.sld_servo.grid(row=3, column=0, columnspan=5, sticky='ew', padx=6)
        self.sld_servo.bind('<ButtonRelease-1>', lambda e: self.do_servo(int(self.sld_servo.get())))

        # ---- 其它 ----
        f = tk.LabelFrame(right, text=' 其它 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='ew', pady=(0, 8)); row += 1
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)

        self.btn_enc = self._btn(f, '编码器清零', self.do_clr_enc, w=12)
        self.btn_enc.grid(row=0, column=0, padx=6, pady=4, sticky='ew')
        self.btn_estop = self._btn(f, '紧急停止', self.do_estop, bg=C_ESTOP, fg='white',
                                   w=12, font=('Microsoft YaHei', 10, 'bold'))
        self.btn_estop.grid(row=0, column=1, padx=6, pady=4, sticky='ew')

        # ★ 电机通道自检：先给"电机1"15%、再给"电机2"15%，各一秒
        #   哪个轮子动就说明固件的"电机1/2"对应板子上的哪一路 —— 不用万用表
        self.btn_mtest = self._btn(f, '电机通道自检 (1→2)', self.do_motor_selftest, w=24, h=1,
                                   font=('Microsoft YaHei', 9))
        self.btn_mtest.grid(row=1, column=0, columnspan=2, padx=6, pady=(0, 6), sticky='ew')

        # ---- 日志 ----
        f = tk.LabelFrame(right, text=' 板子回话 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='nsew'); row += 1
        right.rowconfigure(row - 1, weight=1)

        self.txt_log = tk.Text(f, height=5, bg='#12121a', fg='#8fc8ff', bd=0,
                               font=('Consolas', 8), wrap='none')
        self.txt_log.pack(fill='both', expand=True, padx=4, pady=4)
        self.txt_log.configure(state='disabled')

        right.rowconfigure(row, weight=0)

    # ================================================================ 动作（全部由按钮触发）
    def send(self, cmd, note=None):
        c = self.conn
        if c is None:
            self.log('（未连接，没发出去）%s' % (note or ''))
            return False
        try:
            c.sendall((cmd + '\n').encode('ascii', 'ignore'))
            if note:
                self.log(note)
            return True
        except Exception as e:
            self.log('发送失败: %s' % e)
            return False

    def do_mode(self, binary):
        self.mode_gray = (binary == 0)
        self.shot = None
        self.gray_got = set()
        self.send('s %d' % binary, '图像 → %s' % ('二值图' if binary else '原图灰度'))
        self.refresh_buttons()

    # ---------------------------------------------------------------- 电机1
    def set_motor_mode(self, m):
        """切换控制方式：'pwm' 直接给占空比（默认 30%） / 'rpm' 走速度环闭环"""
        if m == self.ctrl_mode:
            return
        self.running = False
        if self.ctrl_mode == 'pwm':
            self.send('w 0 0', '电机1 → PWM 清零')
        else:
            self.send('q 0', '电机1 → 速度环停止')

        self.ctrl_mode = m
        try:
            if m == 'pwm':
                self.sld_speed.configure(from_=0, to=100)
                self.sld_speed.set(30)
                self.presets = PWM_PRESETS
            else:
                self.sld_speed.configure(from_=0, to=3000)
                self.sld_speed.set(500)
                self.presets = RPM_PRESETS
            for i, b in enumerate(self.preset_btns):
                b.configure(text='%d' % self.presets[i])
        except Exception:
            pass
        self.refresh_buttons()
        self.update_speed_ui()

    def do_start(self):
        """★ 启动：按当前的"方式 × 方向 × 滑条值"驱动电机1"""
        self.running = True
        self.apply_speed()

    def do_stop_cmd(self):
        """★ 停止：速度环目标清零 + 两路占空比清零（两种方式都能停）"""
        self.running = False
        self.req_rpm = 0
        self.send('q 0', '电机1 → 停止')
        self.send('w 0 0')
        self.update_speed_ui()

    def do_dir(self, sign):
        """只选方向，不启动；已经在转的话立刻按新方向重发"""
        self.dir = +1 if sign >= 0 else -1
        if self.running:
            self.apply_speed()
        else:
            self.update_speed_ui()

    def do_preset_idx(self, i):
        try:
            v = self.presets[i]
        except Exception:
            return
        self.sld_speed.set(v)
        if self.running:
            self.apply_speed()
        else:
            self.update_speed_ui()

    def do_zero(self):
        self.sld_speed.set(0)
        if self.running:
            self.apply_speed()
        else:
            self.update_speed_ui()

    def on_speed_slide(self):
        """拖动时只更新显示，**不发命令**（不然滑条一动就刷一串）"""
        if not self.running:
            self.update_speed_ui()

    def on_speed_release(self):
        if self.running:
            self.apply_speed()
        else:
            self.update_speed_ui()

    def apply_speed(self):
        v = abs(int(self.sld_speed.get()))
        if self.ctrl_mode == 'pwm':
            duty = self.dir * v * 100              # 30% → 3000（固件 w 命令的量纲 ±10000）
            self.send('w %d 0' % duty, '电机1 → PWM %+d%%' % (self.dir * v))
        else:
            rpm = self.dir * v
            self.send('q %d' % rpm, '电机1 → %+d RPM' % rpm)
        self.update_speed_ui()

    def update_speed_ui(self):
        v = abs(int(self.sld_speed.get()))
        if self.ctrl_mode == 'pwm':
            tgt = '%+d%%' % (self.dir * v)
        else:
            tgt = '%+d RPM' % (self.dir * v)
        if self.running and v > 0:
            txt = '电机1：运行 %s' % tgt
            state_bg = C_ON
        else:
            txt = '电机1：停止   (目标 %s)' % tgt
            state_bg = C_BTN
        try:
            self.lbl_rpm.configure(text=txt)
            self.btn_start.configure(bg=state_bg)
            self.btn_fwd.configure(bg=C_ON if self.dir > 0 else C_BTN)
            self.btn_back.configure(bg=C_ON if self.dir < 0 else C_BTN)
            self.btn_mode_pwm.configure(bg=C_ON if self.ctrl_mode == 'pwm' else C_BTN)
            self.btn_mode_rpm.configure(bg=C_ON if self.ctrl_mode == 'rpm' else C_BTN)
        except Exception:
            pass

    def do_servo(self, pos):
        pos = max(SERVO_MIN, min(SERVO_MAX, int(pos)))
        self.req_servo = pos
        self.sld_servo.set(pos)
        self.lbl_srv.configure(text='目标  %d' % pos)
        self.send('r %d' % pos, '舵机 → %d' % pos)

    def do_servo_nudge(self, d):
        self.do_servo(self.req_servo + d)

    def do_clr_enc(self):
        self.send('l', '编码器清零')

    def do_motor_selftest(self):
        """通道自检：电机1 15% 一秒 → 停 → 电机2 15% 一秒 → 停
        看哪个轮子动，就知道固件的"电机1/2"对应板子上的哪一路。"""
        self.running = False
        self.send('q 0')
        self.send('w 0 0', '自检开始（注意轮子！）')

        def step(ms, cmd, note):
            self.root.after(ms, lambda: self.send(cmd, note))

        step(300,  'w 1500 0', '→ 只给电机1 15%')
        step(1500, 'w 0 0',    '→ 全停')
        step(2000, 'w 0 1500', '→ 只给电机2 15%')
        step(3200, 'w 0 0',    '→ 全停，自检结束')
        self.update_speed_ui()

    def do_estop(self):
        self.running = False
        self.req_rpm = 0
        self.send('q 0', '★ 紧急停止')
        self.send('w 0 0')
        self.update_speed_ui()

    def refresh_buttons(self):
        try:
            self.btn_bin.configure(bg=C_ON if not self.mode_gray else C_BTN)
            self.btn_gray.configure(bg=C_ON if self.mode_gray else C_BTN)
            self.lbl_mode.configure(
                text='当前：%s' % ('原图灰度 94×60' if self.mode_gray else '二值图 188×120'))
        except Exception:
            pass

    # ================================================================ 日志
    def log(self, s):
        try:
            self.txt_log.configure(state='normal')
            self.txt_log.insert('end', s + '\n')
            n = int(self.txt_log.index('end-1c').split('.')[0])
            if n > 300:
                self.txt_log.delete('1.0', '50.0')
            self.txt_log.see('end')
            self.txt_log.configure(state='disabled')
        except Exception:
            pass

    # ================================================================ 解析
    def feed(self, data):
        self.buf += data
        self.nbytes += len(data)
        buf = self.buf

        while True:
            i = buf.find(MAGIC)
            if i < 0:
                j = 0
                while j < len(buf) and buf[j] < 0x80:
                    j += 1
                if j:
                    self.on_text(buf[:j].decode('ascii', 'replace'))
                    del buf[:j]
                if len(buf) < len(MAGIC):
                    if len(buf) > 65536:
                        del buf[:]
                    return
                k = buf.find(b'\xAA', 1)
                if k > 0:
                    del buf[:k]
                else:
                    del buf[:1]
                continue

            if i > 0:
                self.on_text(buf[:i].decode('ascii', 'replace'))
                del buf[:i]

            if len(buf) < 12:
                return
            typ = buf[4]

            if typ == 0xFF:
                flen = (buf[8] << 8) | buf[9]
                if not (24 <= flen <= 480 * 24 and flen % 24 == 0):
                    del buf[:1]
                    continue
                need = 12 + flen + 2
                if len(buf) < need:
                    return
                pay = bytes(buf[12:12 + flen])
                if (sum(pay) & 0xFFFF) == ((buf[12 + flen] << 8) | buf[12 + flen + 1]):
                    self.on_bin_full(pay)
                del buf[:need]
                continue

            if typ == 0xFC:
                flen = (buf[8] << 8) | buf[9]
                if not (1 <= flen <= 8 * GRAY_W):
                    del buf[:1]
                    continue
                need = 12 + flen + 2
                if len(buf) < need:
                    return
                pay = bytes(buf[12:12 + flen])
                if (sum(pay) & 0xFFFF) == ((buf[12 + flen] << 8) | buf[12 + flen + 1]):
                    self.on_gray_block(buf[5], buf[7], pay)
                del buf[:need]
                continue

            if typ == 0xFD:
                flen = (buf[8] << 8) | buf[9]
                if not (2 <= flen <= 480 * 2 and flen % 2 == 0):
                    del buf[:1]
                    continue
                need = 12 + flen + 2
                if len(buf) < need:
                    return
                pay = bytes(buf[12:12 + flen])
                if (sum(pay) & 0xFFFF) == ((buf[12 + flen] << 8) | buf[12 + flen + 1]):
                    ed = []
                    for r in range(min(buf[5], flen // 2)):
                        l = pay[2 * r]
                        rr = pay[2 * r + 1]
                        ed.append((None if l >= 188 else l, None if rr >= 188 else rr))
                    self.edges = ed
                    self.schedule_draw(force=True)
                del buf[:need]
                continue

            del buf[:1]

    def on_bin_full(self, pay):
        buf = self.bin_img
        for r in range(BIN_H):
            seg = pay[r * 24:(r + 1) * 24]
            base = r * BIN_W
            for i, byte in enumerate(seg):
                for k in range(8):
                    idx = i * 8 + k
                    if idx < BIN_W:
                        buf[base + idx] = 255 if (byte >> (7 - k)) & 1 else 0
        self.frames += 1
        if not self.mode_gray:
            self.shot = (BIN_W, BIN_H, bytes(buf), False)
            self.schedule_draw(force=True)

    def on_gray_block(self, blk, rows, pay):
        r0 = blk * 8
        n = min(rows, GRAY_H - r0)
        for ri in range(n):
            base = (r0 + ri) * GRAY_W
            src = ri * GRAY_W
            self.gray_img[base:base + GRAY_W] = pay[src:src + GRAY_W]
            self.gray_got.add(r0 + ri)
        if len(self.gray_got) >= GRAY_H:
            self.gray_got = set()
            self.frames += 1
            if self.mode_gray:
                self.shot = (GRAY_W, GRAY_H, bytes(self.gray_img), True)
                self.schedule_draw(force=True)

    # ================================================================ 文本
    def on_text(self, txt):
        for ln in txt.split('\n'):
            ln = ln.rstrip('\r')
            if not ln.strip() or not all(32 <= ord(c) < 127 for c in ln):
                continue
            if ln.startswith('T '):
                self.on_tele(ln[2:])
            elif ln.startswith('ACK') or ln.startswith('CFG') or ln.startswith('CAM'):
                self.log(ln)

    def on_tele(self, s):
        d = {}
        for tok in s.split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                try:
                    d[k] = float(v)
                except Exception:
                    pass
        if not d:
            return
        self.tele.update(d)
        self.t_tele = time.time()

        if 'r' in d:
            self.v_rpm.set('%d RPM' % int(d['r']))
        if all(k in d for k in ('ax', 'ay', 'az')):
            self.v_acc.set('%6d %6d %6d' % (int(d['ax']), int(d['ay']), int(d['az'])))
        if all(k in d for k in ('gx', 'gy', 'gz')):
            self.v_gyr.set('%6d %6d %6d' % (int(d['gx']), int(d['gy']), int(d['gz'])))
        if 'y' in d:
            self.v_yaw.set('%+.1f °' % d['y'])
        if 's' in d:
            p = int(d['s'])
            self.v_srv.set('---' if p >= 0xFFFF else '%d' % p)
        if 'd1' in d:
            d1 = d.get('d1', 0.0) / 100.0
            d2 = d.get('d2', 0.0) / 100.0
            self.v_duty.set('电机1 %+.0f%%   电机2 %+.0f%%' % (d1, d2))
            # 界面上"运行"但板子输出是 0 ⇒ 命令没到 / 被清零，提示一次
            if self.running and abs(d.get('d1', 0.0)) < 1.0:
                if not self.warn_zero:
                    self.warn_zero = True
                    self.log('⚠ 界面在"运行"，但板子输出 = 0%：命令没到板子，或已被清零')
            else:
                self.warn_zero = False
        if 'm' in d:
            self.mode_gray = (int(d['m']) == 0)
            self.refresh_buttons()

    # ================================================================ 画图
    def schedule_draw(self, force=False):
        now = time.time()
        if (not force) and (now - self.t_draw) < 0.03:
            return
        self.t_draw = now
        self.draw()

    def draw(self):
        if (self.shot is None) or (not HAVE_PIL):
            return
        w, h, data, gray = self.shot
        im = Image.frombytes('L', (w, h), data)
        im = im.resize((VIEW_W, VIEW_H), Image.NEAREST).convert('RGB')

        if self.edges:
            d = ImageDraw.Draw(im)
            s = SCALE_EDGE
            for side in (0, 1):
                prev = None
                for r, e in enumerate(self.edges):
                    x = e[side]
                    if x is None:
                        prev = None
                        continue
                    px, py = x * s + s // 2, r * s + s // 2
                    if prev is not None and abs(x - prev[0]) <= 12:
                        d.line([(prev[1], prev[2]), (px, py)], fill=(255, 0, 0), width=2)
                    d.rectangle([x * s, r * s, x * s + s - 1, r * s + s - 1], fill=(255, 0, 0))
                    prev = (x, px, py)

        try:
            self.photo = ImageTk.PhotoImage(im)
            self.img_lbl.configure(image=self.photo)
        except Exception:
            pass

    # ================================================================ 状态栏
    def tick(self):
        now = time.time()
        if (now - self.t_fps) >= 1.0:
            self.fps = (self.frames - self.n_fps) / (now - self.t_fps)
            self.t_fps, self.n_fps = now, self.frames
            self.bps = (self.nbytes - self.n_bps) / max(now - self.t_bps, 0.001)
            self.t_bps, self.n_bps = now, self.nbytes

        age = (now - self.t_tele) if self.t_tele else 99.0
        conn = '已连接' if self.conn else '未连接'
        self.lbl_stat.configure(
            text='%s    %s    帧率 %.1f    收 %.0f B/s    遥测 %.1fs 前'
                 % (conn, '原图' if self.mode_gray else '二值', self.fps, self.bps, age))

        if age > 3.0:
            for v in (self.v_rpm, self.v_acc, self.v_gyr, self.v_yaw, self.v_duty):
                if v.get() != '---':
                    v.set('---')

    def close(self):
        try:
            self.root.destroy()
        except Exception:
            pass


# ==================================================================== 主循环
def run():
    app = App()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', PORT))
    srv.listen(1)
    srv.settimeout(0.2)

    print('=' * 62)
    print(' small_car 控制台   （全部用按钮操作）')
    print(' 本机 IP  = %s' % local_ip())
    print(' 监听端口 = %d' % PORT)
    print(' ⇒ wifi_f4.h 里的 WIFI_PC_IP / WIFI_PC_PORT 要和上面一致')
    print(' ⇒ 然后给板子上电（模块会主动连过来）')
    print('=' * 62)

    while True:
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            try:
                app.root.update()
            except tk.TclError:
                return
            continue
        except KeyboardInterrupt:
            break

        print('[已连接] %s:%d' % addr)
        app.conn = conn
        app.buf = bytearray()
        conn.settimeout(0.1)
        app.log('--- 板子已连接 ---')

        try:
            conn.sendall(b's\n')          # 问一下状态（顺便确认下行通）
        except Exception:
            pass

        while True:
            try:
                d = conn.recv(8192)
                if not d:
                    print('[对方断开]')
                    break
                app.feed(d)
            except socket.timeout:
                pass
            except Exception as e:
                print('[接收出错] %s' % e)
                break

            app.tick()
            try:
                app.root.update()
            except tk.TclError:
                app.conn = None
                app.close()
                return

        app.conn = None
        try:
            conn.close()
        except Exception:
            pass
        app.log('--- 连接断开 ---')

    app.close()


if __name__ == '__main__':
    if not HAVE_PIL:
        sys.exit(1)
    run()
