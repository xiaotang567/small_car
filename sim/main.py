# -*- coding: utf-8 -*-
"""仿真上位机：3D 追车视角 + 车载相机 + 遥测 + 启停按钮

    python sim\\main.py [端口]
端口默认 8086（和实车一样）⇒ tools/car_ui.py 也能连上来调 sp/sc/q
"""
import math
import os
import sys
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from PIL import Image, ImageDraw, ImageTk

import control as C
import view3d
from sim import Sim

BG = '#1b1f27'
FG = '#dfe6f2'
C_BTN = '#2b3340'
C_ON = '#2f7d4f'
C_OFF = '#7d2f2f'


class App(object):
    def __init__(self, root, port):
        self.root = root
        self.sim = Sim()
        self.trail = []
        self.port = port
        self.link = None

        root.title('small_car 仿真（188x120 车载相机 + 3D 追车视角）')
        root.configure(bg=BG)

        left = tk.Frame(root, bg=BG)
        left.grid(row=0, column=0, padx=6, pady=6, sticky='n')
        right = tk.Frame(root, bg=BG)
        right.grid(row=0, column=1, padx=6, pady=6, sticky='n')

        self.lbl3d = tk.Label(left, bg=BG)
        self.lbl3d.pack()
        self.lblcam = tk.Label(right, bg=BG)
        self.lblcam.pack()

        self.txt = tk.Label(right, text='', justify='left', anchor='w', width=34,
                            font=('Consolas', 10), fg=FG, bg=BG)
        self.txt.pack(fill='x', pady=6)

        btns = tk.Frame(right, bg=BG)
        btns.pack(fill='x')
        self.b_run = self._btn(btns, '启动 100RPM', self.do_run, C_ON, 0, 0)
        self.b_stop = self._btn(btns, '停止', self.do_stop, C_OFF, 0, 1)
        self.b_reset = self._btn(btns, '复位', self.do_reset, C_BTN, 1, 0)
        self.b_trail = self._btn(btns, '清轨迹', self.do_clear, C_BTN, 1, 1)
        for b in (self.b_run, self.b_stop, self.b_reset, self.b_trail):
            b.configure(width=16)

        tk.Label(right, text='调参可走 TCP（端口 %d），用 tools/car_ui.py 连' % port,
                 fg='#7d8797', bg=BG).pack(pady=4)

        self.tick_n = 0
        self.t_wall0 = time.time()
        self.tick()

    def _btn(self, parent, text, cmd, bg, r, c):
        b = tk.Button(parent, text=text, command=cmd, bg=bg, fg='white',
                      activebackground=bg, relief='flat', padx=6, pady=4)
        b.grid(row=r, column=c, padx=3, pady=3, sticky='ew')
        return b

    # ---------------- 按钮 ----------------
    def do_run(self):
        self.sim.start()

    def do_stop(self):
        self.sim.stop()

    def do_reset(self):
        self.sim = Sim()
        self.trail = []
        if self.link:
            self.link.sim = self.sim

    def do_clear(self):
        self.trail = []

    # ---------------- 主循环 ----------------
    def tick(self):
        sim = self.sim
        # ★ 用**墙上时间**推进：渲染再慢也不会把仿真拖成慢动作（一次最多补 100ms）
        now = time.time()
        if not hasattr(self, 't_last'):
            self.t_last = now
            self.acc = 0.0
        self.acc += min(now - self.t_last, 0.1)
        self.t_last = now
        while self.acc >= 0.02:
            self.acc -= 0.02
            for _ in range(4):           # 4×5ms = 20ms 一拍（控制链内部对齐 20ms）
                sim.step(0.005)

        if sim.t_ms % 100 < 20:
            self.trail.append((sim.x, sim.y))
            if len(self.trail) > 900:
                self.trail.pop(0)

        self.tick_n += 1
        if self.tick_n % 4 == 0:         # 3D 每 4 拍刷新（低分辨率渲染再放大，省时间）
            im = view3d.render(sim.track, sim.x, sim.y, sim.yaw, W=240, H=150,
                               trail=self.trail).resize((560, 350), Image.NEAREST)
            self._img3d = ImageTk.PhotoImage(im)
            self.lbl3d.configure(image=self._img3d)
        if self.tick_n % 3 == 1:         # 相机每 3 拍刷新
            im = Image.fromarray(sim.img).resize((C.CAM_W * 3, C.CAM_H * 3), Image.NEAREST)
            self._imgcam = ImageTk.PhotoImage(im)
            self.lblcam.configure(image=self._imgcam)

        t = sim.telemetry()
        wall = time.time() - self.t_wall0
        spd = (sim.t_ms / 1000.0) / max(wall, 1e-6)
        self.txt.configure(text=(
            '仿真时间 %6.1fs  实际 %5.1fs  x%.2f\n'
            '位置 (%5.2f, %5.2f)  航向 %6.1f°\n'
            '转速 %3d RPM  占空比 %4d  舵机 %4d\n'
            '状态 %-4s  偏差 e=%+4d  残差 %3d  行数 %3d\n'
            '近 %+4d  远 %+4d  斜率 kn=%+.2f\n'
            '出赛道累计 %.0f ms   %s'
        ) % (sim.t_ms / 1000.0, wall, spd, sim.x, sim.y, math.degrees(sim.yaw),
             t['r'], t['duty'], t['servo'], sim.state.name(), t['e'], t['rs'], t['nc'],
             t['en'], t['ef'], t['kn'], t['off_ms'],
             '在线 ✓' if sim.on_track else '!! 压线/出界'))

        self.root.after(20, self.tick)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    port = int(args[0]) if args else 8086
    auto = '--auto' in sys.argv          # 直接开跑（不用点按钮，方便看/截图）
    root = tk.Tk()
    app = App(root, port)
    try:
        from tcp_link import TcpLink
        app.link = TcpLink(app.sim, port)
        app.link.start()
        print('[sim] TCP 客户端启动（上位机 car_ui.py 在 %d 上监听时自动连上）' % port)
    except Exception as e:
        print('[sim] TCP 未启动：%s' % e)
    if auto:
        app.do_run()                     # 直接开跑
    root.mainloop()


if __name__ == '__main__':
    main()
