# -*- coding: utf-8 -*-
"""地图绘制器：像搭积木一样画赛道（直线 / 45°圆弧 / 90°圆弧 / 自定义圆弧 / 十字 / 圆环）

    python sim\\map_editor.py

画完点【保存】，赛道存到 sim/track_design.json；
再跑 `python sim\\main.py --auto` 就是你自己画的地图（仿真每次启动都会读这个文件）。
"""
import json
import math
import os
import sys
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from track import (DESIGN_FILE, DEFAULT_DESIGN, Track, load_design,
                   path_from_elements, save_design)

BG = '#1b1f27'
FG = '#dfe6f2'
C_BTN = '#2b3340'
C_ON = '#2f7d4f'
C_CURVE = '#3a5a8c'
C_WARN = '#8c3a3a'
ROAD = '#c9d1dc'
LANE = '#5a6678'


class Editor(object):
    def __init__(self, root):
        self.root = root
        root.title('地图绘制器 —— 直线 / 圆弧 / 十字 / 圆环')
        root.configure(bg=BG)

        self.actions = []          # [('path', elem) | ('cross', dict) | ('ring', dict)]
        self.half_w = 0.25
        self.start = [0.15, 0.0, 0.0]

        # ---------- 画布 ----------
        left = tk.Frame(root, bg=BG)
        left.grid(row=0, column=0, padx=8, pady=8, sticky='n')
        self.cv = tk.Canvas(left, width=760, height=620, bg='#12151b',
                            highlightthickness=0)
        self.cv.pack()

        right = tk.Frame(root, bg=BG)
        right.grid(row=0, column=1, padx=8, pady=8, sticky='n')
        self._build_panel(right)

        # 启动时把当前地图（track_design.json）读进来，直接在上面改
        self._load(load_design())

    # ---------------------------------------------------------------- 面板
    def _row(self, parent, r, text, default, width=7):
        tk.Label(parent, text=text, fg=FG, bg=BG, anchor='w', width=12).grid(
            row=r, column=0, sticky='w', pady=2)
        e = tk.Entry(parent, width=width, bg='#20262f', fg='#ffd479',
                     insertbackground='#ffd479', relief='flat')
        e.insert(0, str(default))
        e.grid(row=r, column=1, pady=2)
        return e

    def _btn(self, parent, r, c, text, cmd, bg=C_BTN, span=1):
        b = tk.Button(parent, text=text, command=cmd, bg=bg, fg='white',
                      activebackground=bg, relief='flat', pady=4)
        b.grid(row=r, column=c, columnspan=span, sticky='ew', padx=2, pady=2)
        return b

    def _build_panel(self, p):
        r = 0
        tk.Label(p, text='① 直线', fg='#8fb3ff', bg=BG).grid(row=r, column=0, columnspan=2, sticky='w')
        r += 1
        self.e_len = self._row(p, r, '长度 (m)', '1.00')
        r += 1
        self._btn(p, r, 0, '前进 →', self.add_line, C_ON, span=2)
        r += 1

        tk.Label(p, text='② 圆弧（45° / 90° / 自定义）', fg='#8fb3ff', bg=BG).grid(
            row=r, column=0, columnspan=2, sticky='w', pady=(8, 0))
        r += 1
        self.e_r = self._row(p, r, '半径 (m)', '1.30')
        r += 1
        self.e_deg = self._row(p, r, '角度 (°)', '45')
        r += 1
        self._btn(p, r, 0, '左转 45°', lambda: self.add_arc(45), C_CURVE)
        self._btn(p, r, 1, '右转 45°', lambda: self.add_arc(-45), C_CURVE)
        r += 1
        self._btn(p, r, 0, '左转 90°', lambda: self.add_arc(90), C_CURVE)
        self._btn(p, r, 1, '右转 90°', lambda: self.add_arc(-90), C_CURVE)
        r += 1
        self._btn(p, r, 0, '左转 自定义', lambda: self.add_arc(None, +1), C_CURVE)
        self._btn(p, r, 1, '右转 自定义', lambda: self.add_arc(None, -1), C_CURVE)
        r += 1

        tk.Label(p, text='③ 特殊元素（放在**当前车头位置**）', fg='#8fb3ff', bg=BG).grid(
            row=r, column=0, columnspan=2, sticky='w', pady=(8, 0))
        r += 1
        self.e_cross = self._row(p, r, '十字半长', '1.30')
        r += 1
        self._btn(p, r, 0, '＋十字', self.add_cross, C_BTN, span=2)
        r += 1
        self.e_rin = self._row(p, r, '圆环内径', '0.75')
        self.e_rout = self._row(p, r + 1, '圆环外径', '1.35')
        r += 2
        self._btn(p, r, 0, '＋圆环', self.add_ring, C_BTN, span=2)
        r += 1

        tk.Label(p, text='④ 赛道宽度', fg='#8fb3ff', bg=BG).grid(
            row=r, column=0, columnspan=2, sticky='w', pady=(8, 0))
        r += 1
        self.e_hw = self._row(p, r, '半宽 (m)', '0.25')
        r += 1
        self._btn(p, r, 0, '应用宽度', self.apply_width, C_BTN, span=2)
        r += 1

        self._btn(p, r, 0, '撤销一步', self.undo, C_WARN, span=2)
        r += 1
        self._btn(p, r, 0, '清空', self.clear, C_WARN, span=2)
        r += 1
        self._btn(p, r, 0, '保存地图', self.save, C_ON, span=2)
        self._btn(p, r + 1, 0, '载入已保存', self.reload, C_BTN, span=2)
        self._btn(p, r + 2, 0, '载入默认跑道', self.load_default, C_BTN, span=2)
        r += 3

        self.lbl = tk.Label(p, text='', fg=FG, bg=BG, justify='left', anchor='w',
                            font=('Consolas', 9))
        self.lbl.grid(row=r, column=0, columnspan=2, sticky='w', pady=6)

    # ---------------------------------------------------------------- 数据
    def _num(self, entry, default):
        try:
            return float(entry.get())
        except Exception:
            return default

    def design(self):
        path = [a[1] for a in self.actions if a[0] == 'path']
        return {
            'half_w': self.half_w,
            'start': self.start,
            'path': path,
            'crosses': [a[1] for a in self.actions if a[0] == 'cross'],
            'rings': [a[1] for a in self.actions if a[0] == 'ring'],
        }

    def path_points(self):
        d = self.design()
        el = d['path']
        if not el:
            return np.array([self.start[:2]], dtype=np.float64)
        return path_from_elements(el, start_xy=self.start[:2], start_yaw=self.start[2])

    def pose_at_end(self):
        d = self.design()
        el = d['path']
        if not el:
            return self.start[0], self.start[1], self.start[2]
        p = path_from_elements(el, start_xy=self.start[:2], start_yaw=self.start[2])
        yaw = math.atan2(p[-1][1] - p[-2][1], p[-1][0] - p[-2][0])
        return float(p[-1][0]), float(p[-1][1]), yaw

    # ---------------------------------------------------------------- 动作
    def add_line(self):
        L = self._num(self.e_len, 1.0)
        if L <= 0:
            return
        self.actions.append(('path', {'t': 'line', 'len': L}))
        self.redraw()

    def add_arc(self, deg, sign=1):
        r = self._num(self.e_r, 1.3)
        d = abs(self._num(self.e_deg, 45.0)) if deg is None else abs(deg)
        if r <= 0 or d <= 0:
            return
        self.actions.append(('path', {'t': 'arc', 'r': r, 'deg': sign * d}))
        self.redraw()

    def add_cross(self):
        x, y, yaw = self.pose_at_end()
        half = self._num(self.e_cross, 1.3)
        self.actions.append(('cross', {'x': x, 'y': y,
                                       'yaw': yaw + math.pi / 2, 'half': half}))
        self.redraw()

    def add_ring(self):
        x, y, yaw = self.pose_at_end()
        ri = self._num(self.e_rin, 0.75)
        ro = self._num(self.e_rout, 1.35)
        self.actions.append(('ring', {'x': x, 'y': y, 'r_in': ri, 'r_out': ro}))
        self.redraw()

    def apply_width(self):
        self.half_w = max(0.05, self._num(self.e_hw, 0.25))
        self.redraw()

    def undo(self):
        if self.actions:
            self.actions.pop()
            self.redraw()

    def clear(self):
        self.actions = []
        self.redraw()

    def save(self):
        d = self.design()
        p = save_design(d)
        self.lbl.configure(text='已保存到 %s\n跑仿真：python sim\\main.py --auto'
                                % os.path.basename(p))

    def reload(self):
        d = load_design()
        self._load(d)

    def load_default(self):
        self._load(json.loads(json.dumps(DEFAULT_DESIGN)))

    def _load(self, d):
        self.actions = []
        for e in d.get('path', []):
            if e.get('t') == 'pts':
                continue
            self.actions.append(('path', e))
        for c in d.get('crosses', []):
            self.actions.append(('cross', c))
        for g in d.get('rings', []):
            self.actions.append(('ring', g))
        self.half_w = float(d.get('half_w', 0.25))
        self.start = d.get('start', [0.15, 0.0, 0.0])
        self.e_hw.delete(0, 'end'); self.e_hw.insert(0, '%.2f' % self.half_w)
        self.redraw()

    # ---------------------------------------------------------------- 绘制
    def redraw(self):
        cv = self.cv
        cv.delete('all')
        pts = self.path_points()
        hw = self.half_w

        xs = [p[0] for p in pts] + [self.start[0]]
        ys = [p[1] for p in pts] + [self.start[1]]
        for a in self.actions:
            if a[0] == 'ring':
                g = a[1]
                xs += [g['x'] - g['r_out'], g['x'] + g['r_out']]
                ys += [g['y'] - g['r_out'], g['y'] + g['r_out']]
            elif a[0] == 'cross':
                c = a[1]
                xs += [c['x'] - c['half'], c['x'] + c['half']]
                ys += [c['y'] - c['half'], c['y'] + c['half']]
        x0, x1 = min(xs) - 0.6, max(xs) + 0.6
        y0, y1 = min(ys) - 0.6, max(ys) + 0.6

        W, H = 760, 620
        s = min((W - 20) / max(x1 - x0, 1e-6), (H - 20) / max(y1 - y0, 1e-6))
        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5

        def T(p):
            return (W * 0.5 + (p[0] - cx) * s, H * 0.5 - (p[1] - cy) * s)

        wpx = max(2.0, 2.0 * hw * s)

        # 圆环
        for a in self.actions:
            if a[0] != 'ring':
                continue
            g = a[1]
            c = T((g['x'], g['y']))
            for rr, col in ((g['r_out'], ROAD), (g['r_in'], '#12151b')):
                cv.create_oval(c[0] - rr * s, c[1] - rr * s, c[0] + rr * s, c[1] + rr * s,
                               outline=col, width=wpx, fill='')
                if col != ROAD:
                    cv.create_oval(c[0] - rr * s, c[1] - rr * s, c[0] + rr * s,
                                   c[1] + rr * s, outline='', fill='#12151b')
        # 十字
        for a in self.actions:
            if a[0] != 'cross':
                continue
            c = a[1]
            dx, dy = math.cos(c['yaw']) * c['half'], math.sin(c['yaw']) * c['half']
            p1 = T((c['x'] - dx, c['y'] - dy))
            p2 = T((c['x'] + dx, c['y'] + dy))
            cv.create_line(p1[0], p1[1], p2[0], p2[1], fill=ROAD, width=wpx,
                           capstyle='round')
        # 赛道（用粗线画，宽度 = 真实赛道宽）
        for k in range(len(pts) - 1):
            p1 = T(pts[k])
            p2 = T(pts[k + 1])
            cv.create_line(p1[0], p1[1], p2[0], p2[1], fill=ROAD, width=wpx,
                           capstyle='round')
        # 中心虚线
        if len(pts) > 1:
            flat = []
            for p in pts:
                q = T(p)
                flat += [q[0], q[1]]
            cv.create_line(flat, fill=LANE, width=1, dash=(4, 6))

        # 起点 + 车头方向
        sp = T(self.start[:2])
        cv.create_oval(sp[0] - 5, sp[1] - 5, sp[0] + 5, sp[1] + 5,
                       fill=C_ON, outline='')
        cv.create_line(sp[0], sp[1],
                       sp[0] + math.cos(self.start[2]) * 26,
                       sp[1] - math.sin(self.start[2]) * 26,
                       fill=C_ON, width=2, arrow='last')

        # 当前笔 = 最后一个元素的终点
        x, y, yaw = self.pose_at_end()
        ep = T((x, y))
        cv.create_oval(ep[0] - 4, ep[1] - 4, ep[0] + 4, ep[1] + 4, fill='#ffd479', outline='')
        cv.create_line(ep[0], ep[1], ep[0] + math.cos(yaw) * 20, ep[1] - math.sin(yaw) * 20,
                       fill='#ffd479', width=2, arrow='last')
        # 闭合提示
        gap = math.hypot(x - self.start[0], y - self.start[1])
        if gap > 0.05:
            sp2 = T((self.start[0], self.start[1]))
            cv.create_line(ep[0], ep[1], sp2[0], sp2[1], fill='#c05a5a', width=1, dash=(3, 5))

        n_path = sum(1 for a in self.actions if a[0] == 'path')
        total = 0.0
        for a in self.actions:
            if a[0] == 'path':
                e = a[1]
                total += e.get('len', 0.0) if e['t'] == 'line' else abs(
                    math.radians(e['deg'])) * e['r']
        info = ('元素 %d 段 · 总长 %.2f m · 赛道宽 %.2f m\n'
                '笔位 (%.2f, %.2f) 朝向 %.0f° · 闭合差 %.3f m %s'
                % (n_path, total, 2 * hw, x, y, math.degrees(yaw), gap,
                   '✓ 闭合' if gap < 0.3 else '⚠ 没闭合'))
        self.lbl.configure(text=info)

    def run(self):
        self.root.mainloop()


def main():
    root = tk.Tk()
    ed = Editor(root)
    ed.run()


if __name__ == '__main__':
    main()
