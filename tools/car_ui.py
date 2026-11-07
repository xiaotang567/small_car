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
       舵机：左 / 中 / 右 / −50 / −20 / +20 / +50 / 滑条
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
import os

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

# ★ 位打包解包查表：1 字节 → 8 个像素（255/0）。
#   原来是 120×24×8 = **2.3 万次 Python 循环/帧**，20fps 就把主线程占满
#   ⇒ 按钮迟钝、收包变慢（TCP 反压 ⇒ 板子那边跟着丢数据）。
#   查表后每帧只有 2880 次 join，快一个数量级。
_BIT8 = [bytes(255 if (b >> (7 - k)) & 1 else 0 for k in range(8)) for b in range(256)]

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
C_WAIT    = '#b8860b'      # ★ 命令"已发出、等板子回执"时按钮变这个琥珀色

# 快捷档 / 预设
# 电机控制：两种方式
#   'pwm' —— 直接给占空比（默认 30%，开环，不依赖编码器，用来先确认电机装反没/转不转）
#   'rpm' —— 走速度环闭环（需要编码器读数正确）
PWM_PRESETS = (10, 20, 30, 50)          # 占空比 %
RPM_PRESETS = (500, 800, 1200, 1500)

# ★ 2026-11-07：二值化阈值（手动，**加减 1** 调节；0 在固件里仍是自动 Otsu）
THRESH_MIN, THRESH_MAX, THRESH_INIT = 1, 255, 100
# ★ 2026-11-07：画面亮度目标（SCCB R0xA5，**加减 2** 调节；越小越暗，AEC 会自动跟着调）
EXPO_MIN, EXPO_MAX, EXPO_INIT = 1, 63, 40
SERVO_MID = 2748                       # ★ 2026-11-07 用户实测：默认中位
SERVO_MIN, SERVO_MAX = 1700, 3024      # ★ 安全行程（左/右快捷档就取这两个端点）
SERVO_LEFT, SERVO_RIGHT = SERVO_MIN, SERVO_MAX


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
        # 遥测质量（用板子发的序号 n= 算"收到几条 / 丢几条"）
        self.tel_seq = None
        self.tel_rx = 0
        self.tel_lost = 0
        self.tel_rate = 0.0
        self.tel_loss = 0.0

        # ---- 目标值（界面状态）----
        self.req_rpm = 0            # 想跑多少（带方向）
        self.req_servo = SERVO_MID
        self.thresh_val = 0            # ★ 0 = 自动 Otsu（和固件默认一致）；第一次按 ± 从 100 起步
        self.expo_val = EXPO_INIT       # ★ 100（和固件默认的 cam_shutter 一致）
        self.running = False        # ★ 电机1 是否"启动"状态：上电默认停
        self.dir = +1               # 方向：+1 前进 / -1 后退
        self.ctrl_mode = 'rpm'      # ★ 默认"转速闭环"（固件巡航目标 500 RPM）；可切回占空比
        self.presets = RPM_PRESETS
        self.pending = []           # 等回执的命令（send_ack 用）
        self.txt_pend = ''          # 文本行缓冲（TCP 分包时半行先攒着）
        self.txt_ok = 0             # 板子报的文本发送成功次数（包头第10字节）
        self.txt_fail = 0           # 板子报的文本抢不到窗口的次数（包头第11字节）
        self.warn_zero = False      # 只提示一次"运行但输出 0"
        # ★ 2026-11-07：S 心跳里带着板子的"复位来源 / 上次 HardFault 现场"，
        #   以前完全没解析（等于把最重要的证据丢了）。现在变化时打到日志里。
        self.fw_seen = None
        self.fault_seen = None
        self.t_keep = 0.0           # 上次发保活的时刻
        self.t_rx = time.time()     # 上次收到任何字节的时刻
        # ★ 2026-11-07：**把收到的每一行都写进文件**（遥测/S/ACK…），
        #   这样出问题时直接看文件就行，不用去抢连接。
        self.logf = None
        self.log_path = None
        self.log_n = 0

        # ---- 窗口 ----
        self.root = tk.Tk()
        self.root.title('small_car 控制台')
        self.root.configure(bg=C_BG)
        self.root.geometry('1220x880')
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

        # ---------------- 右：控制台（★ 可滚动）----------------
        # 右侧面板从"图像源"到"板子回话"一共 6 块，560 高的窗口根本放不下
        # ⇒ 下面几块（舵机角度 / 其它 / 板子回话）会被挤出屏幕，用户看不到按钮。
        # 现在套一个 Canvas + Scrollbar，怎么缩放窗口都能滚到。
        right_outer = tk.Frame(root, bg=C_PANEL)
        right_outer.grid(row=0, column=1, sticky='nsew')
        right_outer.rowconfigure(0, weight=1)
        right_outer.columnconfigure(0, weight=1)

        canvas = tk.Canvas(right_outer, bg=C_PANEL, highlightthickness=0, bd=0)
        canvas.grid(row=0, column=0, sticky='nsew')
        sb = tk.Scrollbar(right_outer, orient='vertical', command=canvas.yview)
        sb.grid(row=0, column=1, sticky='ns')
        canvas.configure(yscrollcommand=sb.set)

        right = tk.Frame(canvas, bg=C_PANEL, padx=10, pady=8)
        win = canvas.create_window((0, 0), window=right, anchor='nw')

        def _on_right_conf(e):
            canvas.configure(scrollregion=canvas.bbox('all'))

        def _on_canvas_conf(e):
            canvas.itemconfigure(win, width=e.width)

        right.bind('<Configure>', _on_right_conf)
        canvas.bind('<Configure>', _on_canvas_conf)

        def _on_wheel(e):
            canvas.yview_scroll(int(-e.delta / 120), 'units')

        canvas.bind_all('<MouseWheel>', _on_wheel)

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
        # ★ 停/恢复图传：把链路完全让给文本，专门用来验证遥测/命令
        self.btn_img_off = self._btn(f, '停图传', lambda: self.do_img(0), w=10)
        self.btn_img_off.grid(row=1, column=0, columnspan=2, padx=6, pady=(0, 6), sticky='ew')
        self.btn_img_on = self._btn(f, '恢复图传', lambda: self.do_img(1), w=10)
        self.btn_img_on.grid(row=1, column=2, columnspan=2, padx=6, pady=(0, 6), sticky='ew')
        # ★★ 2026-11-07：**二值化阈值（加减 1）** ★★
        #   固件用手动阈值；这里 -1 / +1 两个按钮，中间显示当前值。
        #   每点一下发 `th <n>`，板子回 ACK th；同前缀命令会自动顶掉上一条，连点不会堆积。
        self.btn_th_dn = self._btn(f, '阈值 −1', lambda: self.do_thresh(-1), w=7, h=1)
        self.btn_th_dn.grid(row=2, column=0, padx=6, pady=(0, 6), sticky='w')
        self.lbl_th = self._lbl(f, '阈值 自动', fg=C_ACCENT,
                                font=('Consolas', 12, 'bold'))
        self.lbl_th.grid(row=2, column=1, columnspan=2, sticky='ew')
        self.btn_th_up = self._btn(f, '阈值 +1', lambda: self.do_thresh(+1), w=7, h=1)
        self.btn_th_up.grid(row=2, column=3, padx=6, pady=(0, 6), sticky='e')

        # ★★ 2026-11-07：**曝光（加减 5）** —— 固件 `e <n>` 是**运行时生效**的
        #   （直接写 SCCB R0x0B，不用重配摄像头）。范围 10..110 行，越小越暗。
        self.btn_ex_dn = self._btn(f, '亮度 −2', lambda: self.do_expo(-2), w=7, h=1)
        self.btn_ex_dn.grid(row=3, column=0, padx=6, pady=(0, 6), sticky='w')
        self.lbl_ex = self._lbl(f, '亮度 %d' % EXPO_INIT, fg=C_ACCENT,
                                font=('Consolas', 12, 'bold'))
        self.lbl_ex.grid(row=3, column=1, columnspan=2, sticky='ew')
        self.btn_ex_up = self._btn(f, '亮度 +2', lambda: self.do_expo(+2), w=7, h=1)
        self.btn_ex_up.grid(row=3, column=3, padx=6, pady=(0, 6), sticky='e')

        self.lbl_mode = self._lbl(f, '当前：二值图 188×120', fg=C_DIM, font=('Microsoft YaHei', 9))
        self.lbl_mode.grid(row=4, column=0, columnspan=4, sticky='w', padx=6, pady=(0, 6))

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
            b = self._btn(f, '%d' % self.presets[i], (lambda idx=i: self.do_preset_idx(idx)),
                          w=6, h=1, font=('Consolas', 10))
            b.grid(row=4, column=i, padx=3, pady=(2, 4), sticky='ew')
            self.preset_btns.append(b)

        self.sld_speed = tk.Scale(f, from_=0, to=3000, orient='horizontal',
                                  bg=C_PANEL, fg=C_FG, troughcolor='#3c3c4c',
                                  highlightbackground=C_PANEL, showvalue=False, length=250,
                                  command=lambda v: self.on_speed_slide())
        self.sld_speed.set(500)          # ★ 默认闭环 500 RPM（和固件巡航目标一致）
        self.sld_speed.grid(row=5, column=0, columnspan=4, sticky='ew', padx=6)
        self.sld_speed.bind('<ButtonRelease-1>', lambda e: self.on_speed_release())

        # ---- 舵机 ----
        f = tk.LabelFrame(right, text=' 舵机角度 ', bg=C_PANEL, fg=C_ACCENT,
                          font=('Microsoft YaHei', 10, 'bold'), bd=1, relief='groove')
        f.grid(row=row, column=0, sticky='ew', pady=(0, 8)); row += 1
        for c in range(5):
            f.columnconfigure(c, weight=1)

        self.lbl_srv = tk.Label(f, text='目标  %d' % SERVO_MID, bg=C_PANEL, fg=C_FG,
                                font=('Microsoft YaHei', 11, 'bold'))
        self.lbl_srv.grid(row=0, column=0, columnspan=5, sticky='w', padx=6, pady=(4, 2))

        self.btn_sl = self._btn(f, '左', lambda: self.do_servo(SERVO_LEFT, self.btn_sl), w=4, h=2)
        self.btn_sl.grid(row=1, column=0, padx=3, pady=3, sticky='ew')
        self.btn_sm = self._btn(f, '中', lambda: self.do_servo(SERVO_MID, self.btn_sm), bg=C_ON, w=4, h=2)
        self.btn_sm.grid(row=1, column=2, padx=3, pady=3, sticky='ew')
        self.btn_sr = self._btn(f, '右', lambda: self.do_servo(SERVO_RIGHT, self.btn_sr), w=4, h=2)
        self.btn_sr.grid(row=1, column=4, padx=3, pady=3, sticky='ew')

        # ★★ 2026-11-07：**步进加减**：−50 / −20 … +20 / +50（一次点一下 = 目标位置 ±50 步）★★
        #   舵机 4096 步 = 360° ⇒ 50 步 ≈ 4.4°；按住不放/连点就连续加。
        #   每一步都会夹在 SERVO_MIN..SERVO_MAX 里，不会顶到机械限位。
        self.btn_sd50 = self._btn(f, '−50', lambda: self.do_servo_nudge(-50, self.btn_sd50), w=4, h=1)
        self.btn_sd50.grid(row=2, column=0, padx=3, pady=(0, 4), sticky='ew')
        self.btn_sd = self._btn(f, '−20', lambda: self.do_servo_nudge(-20, self.btn_sd), w=4, h=1)
        self.btn_sd.grid(row=2, column=1, padx=3, pady=(0, 4), sticky='ew')
        self.btn_su = self._btn(f, '+20', lambda: self.do_servo_nudge(+20, self.btn_su), w=4, h=1)
        self.btn_su.grid(row=2, column=3, padx=3, pady=(0, 4), sticky='ew')
        self.btn_su50 = self._btn(f, '+50', lambda: self.do_servo_nudge(+50, self.btn_su50), w=4, h=1)
        self.btn_su50.grid(row=2, column=4, padx=3, pady=(0, 4), sticky='ew')

        self.sld_servo = tk.Scale(f, from_=SERVO_MIN, to=SERVO_MAX, orient='horizontal',
                                  bg=C_PANEL, fg=C_FG, troughcolor='#3c3c4c',
                                  highlightbackground=C_PANEL, showvalue=False, length=250)
        self.sld_servo.set(SERVO_MID)
        self.sld_servo.grid(row=3, column=0, columnspan=5, sticky='ew', padx=6)
        self.sld_servo.bind('<ButtonRelease-1>', lambda e: self.do_servo(int(self.sld_servo.get())))

        # ★ 物理层切换：全双工（PC10=TX / PC11=RX） ⇄ 单线半双工（只接 PC10）
        #   调试板/BusLinker 到底该用哪种，点一下就能试，不用重烧固件
        self.btn_full = self._btn(f, '全双工(TX+RX)', lambda: self.do_duplex(0), w=13, h=1,
                                  font=('Microsoft YaHei', 9))
        self.btn_full.grid(row=4, column=0, columnspan=3, padx=3, pady=(2, 4), sticky='ew')
        self.btn_half = self._btn(f, '单线半双工(只接PC10)', lambda: self.do_duplex(1), w=18, h=1,
                                  font=('Microsoft YaHei', 9))
        self.btn_half.grid(row=4, column=3, columnspan=2, padx=3, pady=(2, 4), sticky='ew')

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

    # ---------------------------------------------------------------- 命令确认（按钮变黄 → 收到 ACK 才变回）
    WAIT_MS = 250          # 没回执多久重发一次
    WAIT_MAX = 80          # 最多重发多少次（80 × 250ms ≈ 20 秒，够板子慢半拍）

    def send_ack(self, cmd, pfx, note=None, btn=None):
        """发一条**板子会回 ACK** 的命令。
        ★ 交互（用户要求）：点下去按钮立刻变**黄色**，然后每 250ms 一直重发，
          直到板子回 `ACK ...` 才把按钮颜色**恢复**（恢复后按当前状态重新上色）。
        ⚠️ 同前缀的新命令会顶掉旧的（否则"停止"的重发会晚到，把刚启动的电机又停掉）。"""
        if not self.send(cmd, note):
            return False

        # 顶掉同类旧命令，并把它的按钮颜色恢复
        keep = []
        for p in self.pending:
            if p['pfx'] == pfx:
                self._btn_restore(p)
            else:
                keep.append(p)
        self.pending = keep

        p = {'cmd': cmd, 'pfx': pfx, 't': time.time(), 'n': 0, 'btn': btn,
             'base': None, 'wait': False}
        if btn is not None:
            try:
                p['base'] = btn.cget('bg')
                btn.configure(bg=C_WAIT)
                p['wait'] = True
            except Exception:
                pass
        self.pending.append(p)
        return True

    def _btn_restore(self, p):
        if p.get('wait') and (p.get('btn') is not None) and (p.get('base') is not None):
            try:
                p['btn'].configure(bg=p['base'])
            except Exception:
                pass
            p['wait'] = False

    def _ack_seen(self, ln):
        if not self.pending:
            return
        keep = []
        for p in self.pending:
            if ln.startswith(p['pfx']):
                self._btn_restore(p)          # 板子收到了 ⇒ 按钮颜色恢复
            else:
                keep.append(p)
        self.pending = keep
        # 恢复后再按"当前真实状态"重新上色（启动/前进/后退/模式按钮颜色跟着状态走）
        self.update_speed_ui()
        self.refresh_buttons()

    def _check_pending(self):
        if not self.pending:
            return
        now = time.time()
        for p in list(self.pending):
            if (now - p['t']) * 1000.0 < self.WAIT_MS:
                continue
            if p['n'] < self.WAIT_MAX:
                p['n'] += 1
                p['t'] = now
                try:
                    self.conn.sendall((p['cmd'] + '\n').encode('ascii', 'ignore'))
                    if p['n'] in (1, 5, 20):
                        self.log('… 板子还没回执，重发 %d 次：%s' % (p['n'], p['cmd']))
                except Exception:
                    pass
            else:
                self.pending.remove(p)
                self._btn_restore(p)
                self.log('✗ 命令一直没回执（板子没收到 / 没连上）：%s' % p['cmd'])
                self.update_speed_ui()
                self.refresh_buttons()

    def _clear_pending(self):
        """断线时把所有待确认的命令扔掉，按钮颜色恢复"""
        for p in self.pending:
            self._btn_restore(p)
        self.pending = []

    def do_mode(self, binary):
        self.mode_gray = (binary == 0)
        self.shot = None
        self.gray_got = set()
        self.send_ack('s %d' % binary, 'ACK s', '图像 → %s' % ('二值图' if binary else '原图灰度'),
                        self.btn_bin if binary else self.btn_gray)
        self.refresh_buttons()

    # ---------------------------------------------------------------- 电机1
    def set_motor_mode(self, m):
        """切换控制方式：'pwm' 直接给占空比（默认 30%） / 'rpm' 走速度环闭环"""
        if m == self.ctrl_mode:
            return
        self.running = False
        if self.ctrl_mode == 'pwm':
            self.send_ack('w 0 0', 'ACK w', '电机1 → PWM 清零', self.btn_mode_rpm)
        else:
            self.send_ack('q 0', 'ACK q', '电机1 → 速度环停止', self.btn_mode_pwm)

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
        self.apply_speed(self.btn_start)

    def do_stop_cmd(self):
        """★ 停止：速度环目标清零 + 两路占空比清零（两种方式都能停）"""
        self.running = False
        self.req_rpm = 0
        self.send_ack('q 0', 'ACK q', '电机1 → 停止')
        self.send_ack('w 0 0', 'ACK w', None, self.btn_stop)
        self.update_speed_ui()

    def do_dir(self, sign):
        """只选方向，不启动；已经在转的话立刻按新方向重发"""
        self.dir = +1 if sign >= 0 else -1
        if self.running:
            self.apply_speed(self.btn_fwd if sign >= 0 else self.btn_back)
        else:
            self.update_speed_ui()

    def do_preset_idx(self, i):
        try:
            v = self.presets[i]
        except Exception:
            return
        self.sld_speed.set(v)
        if self.running:
            self.apply_speed(self.preset_btns[i])
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

    def apply_speed(self, btn=None):
        v = abs(int(self.sld_speed.get()))
        if self.ctrl_mode == 'pwm':
            duty = self.dir * v * 100              # 30% → 3000（固件 w 命令的量纲 ±10000）
            self.send_ack('w %d 0' % duty, 'ACK w', '电机1 → PWM %+d%%' % (self.dir * v), btn)
        else:
            rpm = self.dir * v
            self.send_ack('q %d' % rpm, 'ACK q', '电机1 → %+d RPM' % rpm, btn)
        self.update_speed_ui()

    def _is_waiting(self, btn):
        """这个按钮是不是还在"等板子回执"（等的时候别被状态刷新覆盖掉琥珀色）"""
        for p in self.pending:
            if (p.get('btn') is btn) and p.get('wait'):
                return True
        return False

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
            if not self._is_waiting(self.btn_start):
                self.btn_start.configure(bg=state_bg)
            if not self._is_waiting(self.btn_fwd):
                self.btn_fwd.configure(bg=C_ON if self.dir > 0 else C_BTN)
            if not self._is_waiting(self.btn_back):
                self.btn_back.configure(bg=C_ON if self.dir < 0 else C_BTN)
            if not self._is_waiting(self.btn_mode_pwm):
                self.btn_mode_pwm.configure(bg=C_ON if self.ctrl_mode == 'pwm' else C_BTN)
            if not self._is_waiting(self.btn_mode_rpm):
                self.btn_mode_rpm.configure(bg=C_ON if self.ctrl_mode == 'rpm' else C_BTN)
        except Exception:
            pass

    def do_thresh(self, d):
        """二值化阈值 **加减 1**（范围 1..255）；0 在固件里仍是自动 Otsu，按钮不会走到 0。"""
        v = self.thresh_val + d
        if (v < THRESH_MIN): v = THRESH_MIN
        if (v > THRESH_MAX): v = THRESH_MAX
        self.thresh_val = v
        self.lbl_th.configure(text='阈值 %d' % v)
        self.send_ack('th %d' % v, 'ACK th', '二值化阈值 → %d' % v,
                      self.btn_th_up if d > 0 else self.btn_th_dn)

    def do_expo(self, d):
        """曝光 **加减 5**（SCCB R0x0B 行数，10..110，越小越暗）。
           固件收到 `e <n>` 会立刻写寄存器 ⇒ **运行时动态生效**，不用重配摄像头。"""
        v = self.expo_val + d
        if (v < EXPO_MIN): v = EXPO_MIN
        if (v > EXPO_MAX): v = EXPO_MAX
        self.expo_val = v
        self.lbl_ex.configure(text='亮度 %d' % v)
        self.send_ack('e %d' % v, 'ACK e', '画面亮度 → %d' % v,
                      self.btn_ex_up if d > 0 else self.btn_ex_dn)

    def do_servo(self, pos, btn=None):
        pos = max(SERVO_MIN, min(SERVO_MAX, int(pos)))
        self.req_servo = pos
        self.sld_servo.set(pos)
        self.lbl_srv.configure(text='目标  %d' % pos)
        self.send_ack('r %d' % pos, 'ACK r', '舵机 → %d' % pos, btn)

    def do_servo_nudge(self, d, btn=None):
        """在当前目标上加减 d 步（−50 / −20 / +20 / +50），夹在安全限位内再下发"""
        self.do_servo(self.req_servo + d, btn)

    def do_duplex(self, half):
        """切换舵机物理层：0=全双工(PC10=TX/PC11=RX)，1=单线半双工(只接PC10)"""
        if half:
            self.send_ack('r -1', 'ACK r', '舵机 → 单线半双工（只接 PC10 一根线）', self.btn_half)
        else:
            self.send_ack('r -2', 'ACK r', '舵机 → 全双工（PC10=TX, PC11=RX）', self.btn_full)

    def do_img(self, on):
        """图传开关：0 = 停（链路全给文本，调遥测用），1 = 恢复"""
        if on:
            self.send_ack('s 3', 'ACK s', '图传 → 恢复', self.btn_img_on)
        else:
            self.send_ack('s 2', 'ACK s', '图传 → 已停（只发文本）', self.btn_img_off)

    def do_clr_enc(self):
        self.send_ack('l', 'ACK l', '编码器清零', self.btn_enc)

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
        self.send_ack('q 0', 'ACK q', '★ 紧急停止')
        self.send_ack('w 0 0', 'ACK w', None, self.btn_estop)
        self.update_speed_ui()

    def refresh_buttons(self):
        try:
            if not self._is_waiting(self.btn_bin):
                self.btn_bin.configure(bg=C_ON if not self.mode_gray else C_BTN)
            if not self._is_waiting(self.btn_gray):
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
            # ★ 板子把"文本发包 成功/失败次数"塞在包头第 10/11 字节（FW=2087+）
            #   ⇒ 就算遥测一条都收不到，这里也能看出板子到底有没有把文本发出去
            if typ in (0xFF, 0xFC, 0xFD):
                self.txt_ok = buf[10]
                self.txt_fail = buf[11]

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
        out = bytearray()
        for r in range(BIN_H):
            seg = pay[r * 24:(r + 1) * 24]
            out += b''.join(_BIT8[b] for b in seg)[:BIN_W]
        self.frames += 1
        if not self.mode_gray:
            self.shot = (BIN_W, BIN_H, bytes(out), False)
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
    # ★ 已知的记录开头。板子发来的流里会混进大段 0x00（模块的 SPI 读事务会漏字节
    #   到发送流里，实测 25KB/s），还会把一行从中间切断 ⇒ 解析前先洗一遍：
    #   ① 去掉 NUL 和其它控制字符；② 从**最后一个**记录开头切进去（前面可能是半行/垃圾）。
    PFX = ('T n=', 'ACK', 'CFG', 'CAM FOUND', 'S f=', 'BPF ', 'MD ', 'PIN ', 'CAM1', 'CAM2', 'CAM3')

    def on_text(self, txt):
        # ★ 文本可能被 TCP 分包切在两半（遥测行现在有 150 字节，很容易被切开）：
        #   先攒起来，只处理**完整的行**，否则半行会被当成整行丢掉或解析出乱七八糟的值。
        self.txt_pend += txt
        while True:
            k = self.txt_pend.find('\n')
            if k < 0:
                if len(self.txt_pend) > 4096:      # 防跑飞
                    self.txt_pend = ''
                return
            ln = self.txt_pend[:k].rstrip('\r')
            self.txt_pend = self.txt_pend[k + 1:]

            # ① 洗掉 NUL / 控制字符（不能再用 "整行必须全是可打印字符" 一刀切，
            #    否则只要夹了一个 0x00，这条遥测就没了 —— 之前"遥测一直是 ---"就是这么来的）
            ln = ''.join(c for c in ln if 32 <= ord(c) < 127)
            if not ln.strip():
                continue

            # ② 从最后一个已知记录开头切进去（半行文本会和下一条粘在一起）
            cut = -1
            for pfx in self.PFX:
                i = ln.rfind(pfx)
                if i > cut:
                    cut = i
            if cut > 0:
                ln = ln[cut:]

            if ln.startswith('T '):
                self.log_line(ln)
                self.on_tele(ln[2:])
            elif ln.startswith('S '):
                self.log_line(ln)
                self.on_stat(ln[2:])
            elif ln.startswith('ACK') or ln.startswith('CFG') or ln.startswith('CAM'):
                self.log_line(ln)
                self.log(ln)
                self._ack_seen(ln)          # ★ 收到回执 ⇒ 这条命令不用再重发

    # ---- S 心跳：只关心"固件版本 / 复位来源 / 上次 HardFault 现场"，变化时打一行 ----
    def on_stat(self, s):
        d = {}
        for tok in s.split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                d[k] = v
        fw = d.get('FW')
        if fw and fw != self.fw_seen:
            self.fw_seen = fw
            self.log('板子固件 FW=%s（心跳在线）' % fw)

        rst = d.get('RST', '??')
        pc  = d.get('PC', '00000000')
        cf  = d.get('CF', '0000')
        ba  = d.get('BA', '00000000')
        lm  = d.get('LM', '0')
        # LM = 主循环看门狗复位前"卡在第几步"（FW=2100 起）
        LM_NAME = {'0': '开机/未记录', '1': 'LED', '2': 'cam_poll', '3': 'cam_harvest',
                   '4': 'cmd_poll(SPI读)', '5': 'board_ctrl_tick(舵机/IMU/OLED)',
                   '6': 'cam_pol_search', '7': '统计', '8': '图像发送(SPI)',
                   '9': '摄像头探测', '10': '遥测', '11': '心跳'}
        if lm not in ('0', '') or pc not in ('00000000', '') or cf not in ('0000', ''):
            key = (rst, pc, cf, lm)
            if key != self.fault_seen:
                self.fault_seen = key
                # RST 位：bit0=欠压 BOR / bit1=引脚复位 / bit2=上电 / bit3=软件 / bit4=IWDG
                try:
                    r = int(rst, 16)
                except Exception:
                    r = -1
                why = []
                for bit, name in ((0, '欠压BOR'), (1, '引脚复位'), (2, '上电'), (3, '软件复位'),
                                  (4, '看门狗'), (5, '窗口看门狗'), (6, '低功耗')):
                    if r >= 0 and (r >> bit) & 1:
                        why.append(name)
                self.log('⚠ 板子上次运行异常：RST=%s(%s) PC=%s CF=%s BA=%s  卡在 LM=%s(%s)'
                         % (rst, '/'.join(why) or '?', pc, cf, ba,
                            lm, LM_NAME.get(lm, '?')))

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

        # ★ 遥测序号：算丢包率，区分"板子没发"和"链路丢了"
        if 'n' in d:
            nn = int(d['n']) & 0xFFFF
            if self.tel_seq is not None:
                gap = (nn - self.tel_seq) & 0xFFFF
                if 0 < gap < 100:
                    self.tel_lost += gap - 1
            self.tel_seq = nn
            self.tel_rx += 1

        if 'r' in d:
            self.v_rpm.set('%d RPM' % int(d['r']))
        if all(k in d for k in ('ax', 'ay', 'az')):
            self.v_acc.set('%6d %6d %6d' % (int(d['ax']), int(d['ay']), int(d['az'])))
        if all(k in d for k in ('gx', 'gy', 'gz')):
            self.v_gyr.set('%6d %6d %6d' % (int(d['gx']), int(d['gy']), int(d['gz'])))
        if 'y' in d:
            self.v_yaw.set('%+.1f °' % d['y'])
        if 's' in d:
            p  = int(d['s'])
            sid = int(d.get('si', 0))
            rx  = int(d.get('sr', 0))
            sb  = int(d.get('sb', 0))
            st = int(d.get('st', 0))
            ba = int(d.get('sba', 0))
            bas = ('%gM' % (ba / 1000000.0)) if ba >= 1000000 else ('%gk' % (ba / 1000.0))
            lv = int(d.get('sl', 1))
            if p < 0xFFFF:
                self.v_srv.set('%d   (ID %d, %s, 发%dB 收%dB)' % (p, sid, bas, st, rx))
            else:
                # 读不到时把诊断全带上：发了没 / 收了没 / 线是不是被拉低 / 校验错多少
                tip = '' if lv else '  ← RX 被拉低!'
                self.v_srv.set('--- (ID %d, %s, 发%dB 收%dB 校验错%d%s)'
                               % (sid, bas, st, rx, sb, tip))
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

    def log_open(self):
        """★ 记录文件：每次连上都新建一个 tools/log_年月日_时分秒.txt
           里面的每行 = `时间戳 原始文本行`（T 遥测 / S 心跳 / ACK 回执都在），
           出问题时直接把文件给助手看，不用抢连接。"""
        try:
            d = os.path.join(os.path.dirname(os.path.abspath(__file__)))
            os.makedirs(d, exist_ok=True)
            self.log_path = os.path.join(d, 'log_%s.txt' % time.strftime('%Y%m%d_%H%M%S'))
            self.logf = open(self.log_path, 'w', encoding='utf-8', buffering=1)
            self.log_n = 0
            self.log('记录中 → %s' % os.path.basename(self.log_path))
            print('[记录] %s' % self.log_path)
        except Exception as e:
            self.logf = None
            print('[记录] 打开失败: %s' % e)

    def log_close(self):
        try:
            if self.logf is not None:
                self.logf.close()
        except Exception:
            pass
        self.logf = None

    def log_line(self, ln):
        """把一行原始文本追加进记录文件（>8MB 就换新文件，别把盘写爆）"""
        if self.logf is None:
            return
        try:
            self.logf.write('%.3f %s\n' % (time.time(), ln))
            self.log_n += 1
            if self.log_n >= 200:              # 每 200 行 flush 一次，尽量不拖慢 UI
                self.log_n = 0
                self.logf.flush()
                if self.logf.tell() > 8 * 1024 * 1024:
                    self.log_close()
                    self.log_open()
        except Exception:
            self.logf = None

    # ================================================================ 画图
    def schedule_draw(self, force=False):
        now = time.time()
        if (not force) and (now - self.t_draw) < 0.03:
            return
        self.t_draw = now
        self.draw()

    def view_size(self):
        """★ 2026-11-07：显示尺寸**跟着左侧面板走**（原来是写死 752×480）
           写死的后果：窗口一大，画面就缩在中间一小块 —— 用户说的"图片站不满"。
           这里按面板大小 + 188:120 的宽高比算一个尽量大的尺寸（不拉伸变形）。"""
        try:
            fw = self.img_lbl.master.winfo_width() - 18
            fh = self.img_lbl.master.winfo_height() - 30
        except Exception:
            fw, fh = VIEW_W, VIEW_H
        if fw < 120 or fh < 80:
            return VIEW_W, VIEW_H
        sc = min(fw / float(BIN_W), fh / float(BIN_H))
        return max(120, int(BIN_W * sc)), max(80, int(BIN_H * sc))

    def draw(self):
        if (self.shot is None) or (not HAVE_PIL):
            return
        w, h, data, gray = self.shot
        im = Image.frombytes('L', (w, h), data)
        vw, vh = self.view_size()
        im = im.resize((vw, vh), Image.NEAREST).convert('RGB')
        sx = vw / float(w if w else BIN_W)
        sy = vh / float(h if h else BIN_H)

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
                    px, py = int(x * sx), int(r * sy + sy / 2)
                    if prev is not None and abs(x - prev[0]) <= 12:
                        d.line([(prev[1], prev[2]), (px, py)], fill=(255, 0, 0), width=2)
                    d.rectangle([int(x * sx), int(r * sy),
                                 int(x * sx + max(sx, 1)) - 1, int(r * sy + max(sy, 1)) - 1],
                                fill=(255, 0, 0))
                    prev = (x, px, py)

        try:
            self.photo = ImageTk.PhotoImage(im)
            self.img_lbl.configure(image=self.photo)
        except Exception:
            pass

    # ================================================================ 状态栏
    def tick(self):
        now = time.time()
        self._check_pending()          # ★ 命令没回执就重发

        if (now - self.t_fps) >= 1.0:
            dt = max(now - self.t_fps, 0.001)
            self.fps = (self.frames - self.n_fps) / dt
            self.t_fps, self.n_fps = now, self.frames
            self.bps = (self.nbytes - self.n_bps) / max(now - self.t_bps, 0.001)
            self.t_bps, self.n_bps = now, self.nbytes
            # 遥测质量：本秒收到几条 / 丢几条
            tot = self.tel_rx + self.tel_lost
            self.tel_rate = self.tel_rx / dt
            self.tel_loss = (100.0 * self.tel_lost / tot) if tot > 0 else 0.0
            self.tel_rx = 0
            self.tel_lost = 0

        age = (now - self.t_tele) if self.t_tele else 99.0
        conn = '已连接' if self.conn else '未连接'
        self.lbl_stat.configure(
            text='%s    %s    帧率 %.1f    收 %.0f B/s    遥测 %.1f/s 丢%.0f%% (%.1fs 前)    板子文本 ok=%d fail=%d'
                 % (conn, '原图' if self.mode_gray else '二值', self.fps, self.bps,
                    self.tel_rate, self.tel_loss, age, self.txt_ok, self.txt_fail))

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
        app.log_open()                    # ★ 开始记录到 tools/log_*.txt

        try:
            conn.sendall(b's\n')          # 问一下状态（顺便确认下行通）
        except Exception:
            pass
        app.t_keep = 0.0
        app.t_rx = time.time()

        while True:
            # ★ 2026-11-07：**每 2 秒发一个保活 `!`**（板子只计数、不回包）。
            #   板子靠它判断"上行还通不通"：模块被复位/掉线后 SPI 写照样成功，
            #   板子自己发现不了链路已死 ⇒ 会一直"已连接 + 0 B/s"。
            #   有了保活，板子 6 秒收不到就会自己重连/整模块复位。
            now = time.time()
            if (now - app.t_keep) >= 2.0:
                app.t_keep = now
                try:
                    conn.sendall(b'!\n')
                except Exception:
                    pass

            try:
                d = conn.recv(8192)
                if not d:
                    print('[对方断开]')
                    break
                app.t_rx = time.time()
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
        app._clear_pending()
        try:
            conn.close()
        except Exception:
            pass
        app.log('--- 连接断开 ---')
        app.log_close()                   # ★ 收尾，保证记录文件落盘

    app.close()


if __name__ == '__main__':
    if not HAVE_PIL:
        sys.exit(1)
    run()
