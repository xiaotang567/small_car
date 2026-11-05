# -*- coding: utf-8 -*-
"""
small_car 综合上位机  ——  图像 + 仪表 + 控制
================================================================
和 cam_view.py 的区别：这个不只看图，还是"驾驶台"。

  ① 图像：二值图（188×120，位打包）或 **原图灰度**（94×60，8bit）——
          板子上的 `s 0` / `s 1` 命令切换，界面上的单选按钮直接发；
          两种模式下都会把板子发来的**左右边界数组**画成红线。
  ② 仪表：转速 / 三轴加速度 / 三轴角速度 / 偏航角 / 舵机位置 / 编码器
          数据来自板子 10Hz 发的**遥测行** `T r=.. ax=.. ay=.. az=.. gx=.. gy=.. gz=.. y=.. s=.. m=..`
  ③ 控制：速度环目标（q <rpm>）、舵机位置（r <pos>）、图像模式（s 0/1）、
          开环占空比（w <d1> <d2>）、编码器清零（l）

协议（和固件 app.c / cam_f4.c 一致，同一个 TCP 流里混着三种二进制包 + 文本行）：
    AA 55 A5 5A | 0xFF | txh | pol|0x80 | CAM_H | lenH lenL | 2B 预留 | 数据 | ckH ckL   ← 二值整帧
    AA 55 A5 5A | 0xFC | blk | 0     | rows  | lenH lenL | 2B 预留 | 数据 | ckH ckL   ← 灰度缩略图（8 块）
    AA 55 A5 5A | 0xFD | rows| 0     | CAM_H | lenH lenL | 2B 预留 | 数据 | ckH ckL   ← 左右边界算法
    文本行：`T ...` 遥测 / `ACK ...` / `CFG ...`

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

# 板子侧的两套分辨率（和固件宏一一对应）
BIN_W, BIN_H = 188, 120        # 二值图
GRAY_W, GRAY_H = 94, 60        # 灰度缩略图（188×120 的 2×2 抽取）
EDGE_ROWS = 120

VIEW_W, VIEW_H = 752, 480      # 屏幕显示尺寸（两种模式都放大到这么大）
SCALE_EDGE = 4                 # 边界坐标 → 显示坐标的倍率（188→752 / 120→480）


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

        # ---- 图像数据 ----
        self.bin_img = bytearray(BIN_W * BIN_H)          # 二值图缓冲
        self.gray_img = bytearray(GRAY_W * GRAY_H)       # 灰度缓冲
        self.gray_got = set()                            # 已收到的灰度块
        self.edges = None                                # [(左x, 右x)]，None = 该行没找到
        self.mode_gray = False                           # 当前显示模式（跟随板子上报的 m=）
        self.shot = None                                 # 最近一张完整图 (w, h, bytes, gray?)
        self.photo = None
        self.ev_img = None

        # ---- 遥测 ----
        self.tele = {}
        self.t_tele = 0.0
        self.frames = 0
        self.t_fps = time.time()
        self.n_fps = 0
        self.fps = 0.0
        self.nbytes = 0
        self.bps = 0.0
        self.t_bps = time.time()
        self.n_bps = 0
        self.t_draw = 0.0
        self.log_lines = []

        # ---- 界面 ----
        self.root = tk.Tk()
        self.root.title('small_car 驾驶台')
        self.root.configure(bg='#202028')
        self.build_ui()
        self.root.update()

    # ================================================================ 界面
    def build_ui(self):
        st = ttk.Style()
        try:
            st.theme_use('clam')
        except Exception:
            pass
        st.configure('TLabel', background='#202028', foreground='#e0e0e0')
        st.configure('TLabelframe', background='#202028', foreground='#9fd0ff')
        st.configure('TLabelframe.Label', background='#202028', foreground='#9fd0ff')
        st.configure('TCheckbutton', background='#202028', foreground='#e0e0e0')
        st.configure('TRadiobutton', background='#202028', foreground='#e0e0e0')
        st.configure('TButton', padding=4)

        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # ---------------- 左边：图像 ----------------
        left = ttk.Frame(root, padding=6)
        left.grid(row=0, column=0, sticky='nsew')
        self.img_lbl = tk.Label(left, bg='black', bd=1, relief='sunken')
        self.img_lbl.pack(fill='both', expand=True)
        self.lbl_stat = ttk.Label(left, text='未连接…', font=('Consolas', 9))
        self.lbl_stat.pack(fill='x', pady=(4, 0))

        # ---------------- 右边：仪表 + 控制 ----------------
        right = ttk.Frame(root, padding=6)
        right.grid(row=0, column=1, sticky='nsew')

        # ---- 图像模式 ----
        f1 = ttk.LabelFrame(right, text='图像', padding=6)
        f1.pack(fill='x', pady=(0, 6))
        self.var_mode = tk.IntVar(value=1)          # 1 = 二值图
        ttk.Radiobutton(f1, text='二值图 188×120', value=1, variable=self.var_mode,
                        command=lambda: self.set_mode(1)).grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(f1, text='原图灰度 94×60', value=0, variable=self.var_mode,
                        command=lambda: self.set_mode(0)).grid(row=0, column=1, sticky='w')

        # ---- 仪表 ----
        f2 = ttk.LabelFrame(right, text='仪表', padding=6)
        f2.pack(fill='x', pady=(0, 6))
        self.v_rpm = tk.StringVar(value='---')
        self.v_acc = tk.StringVar(value='---')
        self.v_gyr = tk.StringVar(value='---')
        self.v_yaw = tk.StringVar(value='---')
        self.v_srv = tk.StringVar(value='---')
        rows = [('转速', self.v_rpm), ('加速度 ax,ay,az', self.v_acc),
                ('角速度 gx,gy,gz', self.v_gyr), ('偏航角 yaw', self.v_yaw),
                ('舵机位置', self.v_srv)]
        for i, (k, v) in enumerate(rows):
            ttk.Label(f2, text=k, width=15).grid(row=i, column=0, sticky='w')
            ttk.Label(f2, textvariable=v, font=('Consolas', 10)).grid(row=i, column=1, sticky='w')

        # ---- 速度环 ----
        f3 = ttk.LabelFrame(right, text='速度环  q <rpm>', padding=6)
        f3.pack(fill='x', pady=(0, 6))
        self.var_rpm = tk.IntVar(value=0)
        self.lbl_rpm = ttk.Label(f3, text='目标: 0', width=12)
        sc = tk.Scale(f3, from_=-3000, to=3000, orient='horizontal', variable=self.var_rpm,
                      command=lambda v: self.lbl_rpm.configure(text='目标: %d' % int(float(v))),
                      bg='#202028', fg='#e0e0e0', troughcolor='#404050',
                      highlightbackground='#202028', length=210, showvalue=False)
        sc.grid(row=0, column=0, columnspan=3, sticky='we')
        self.lbl_rpm.grid(row=1, column=0, sticky='w')
        ttk.Button(f3, text='下发', command=lambda: self.send('q %d' % self.var_rpm.get())).grid(row=1, column=1)
        ttk.Button(f3, text='停', command=self.cmd_stop).grid(row=1, column=2)

        # ---- 舵机 ----
        f4 = ttk.LabelFrame(right, text='舵机  r <pos>  (4096 = 360°, 中位 2048)', padding=6)
        f4.pack(fill='x', pady=(0, 6))
        self.var_srv = tk.IntVar(value=2048)
        self.lbl_srv = ttk.Label(f4, text='目标: 2048', width=12)
        sc2 = tk.Scale(f4, from_=1048, to=3048, orient='horizontal', variable=self.var_srv,
                       command=lambda v: self.lbl_srv.configure(text='目标: %d' % int(float(v))),
                       bg='#202028', fg='#e0e0e0', troughcolor='#404050',
                       highlightbackground='#202028', length=210, showvalue=False)
        sc2.grid(row=0, column=0, columnspan=3, sticky='we')
        self.lbl_srv.grid(row=1, column=0, sticky='w')
        ttk.Button(f4, text='下发', command=lambda: self.send('r %d' % self.var_srv.get())).grid(row=1, column=1)
        ttk.Button(f4, text='中位', command=lambda: self.set_servo(2048)).grid(row=1, column=2)

        # ---- 其它 ----
        f5 = ttk.LabelFrame(right, text='其它', padding=6)
        f5.pack(fill='x', pady=(0, 6))
        ttk.Button(f5, text='编码器清零 (l)', command=lambda: self.send('l')).grid(row=0, column=0, sticky='w')
        ttk.Button(f5, text='板子状态 (s)', command=lambda: self.send('s')).grid(row=0, column=1, sticky='w')
        ttk.Button(f5, text='开环停 (w 0 0)', command=lambda: self.send('w 0 0')).grid(row=0, column=2, sticky='w')

        # ---- 日志 ----
        f6 = ttk.LabelFrame(right, text='板子回话', padding=4)
        f6.pack(fill='both', expand=True)
        self.txt_log = tk.Text(f6, height=7, width=44, bg='#101014', fg='#9fd0ff',
                               font=('Consolas', 8), bd=0)
        self.txt_log.pack(fill='both', expand=True)
        self.txt_log.configure(state='disabled')

    # ================================================================ 发送
    def send(self, cmd):
        c = self.conn
        if c is None:
            self.log('> %s   （还没连上板子）' % cmd)
            return
        try:
            c.sendall((cmd + '\n').encode('ascii', 'ignore'))
            self.log('> ' + cmd)
        except Exception as e:
            self.log('发送失败: %s' % e)

    def set_mode(self, binary):
        """用户在界面上切"二值图/原图"：先立刻切显示，再通知板子。
        不等遥测回来，否则切换后要等 100ms 才会变。"""
        self.mode_gray = (binary == 0)
        self.var_mode.set(binary)
        self.send('s %d' % binary)
        self.shot = None            # 丢掉上一模式的图，等新模式的帧
        self.gray_got = set()

    def cmd_stop(self):
        self.var_rpm.set(0)
        self.lbl_rpm.configure(text='目标: 0')
        self.send('q 0')
        self.send('w 0 0')

    def set_servo(self, pos):
        self.var_srv.set(pos)
        self.lbl_srv.configure(text='目标: %d' % pos)
        self.send('r %d' % pos)

    def log(self, s):
        self.log_lines.append(s)
        if len(self.log_lines) > 200:
            self.log_lines = self.log_lines[-200:]
        try:
            self.txt_log.configure(state='normal')
            self.txt_log.insert('end', s + '\n')
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

            # ---------- 二值整帧 ----------
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

            # ---------- 灰度缩略图（8 块）----------
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

            # ---------- 左右边界数组 ----------
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

            del buf[:1]        # 不认识的类型 ⇒ 挪 1 字节重找

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
                continue
            if ln.startswith('ACK') or ln.startswith('CFG') or ln.startswith('CAM'):
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
        if 'm' in d:
            self.mode_gray = (int(d['m']) == 0)
            want = 0 if self.mode_gray else 1
            if self.var_mode.get() != want:
                self.var_mode.set(want)

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

        # ---- 左右边界：板子算好的点，这里只画红线 ----
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
        mode = '原图' if self.mode_gray else '二值'
        self.lbl_stat.configure(
            text='%s | %s | 帧率 %.1f | 收 %.0f B/s | 遥测 %.1fs 前'
                 % (conn, mode, self.fps, self.bps, age))

        # 遥测断了就把仪表标成 ---
        if age > 2.0:
            for v in (self.v_rpm, self.v_acc, self.v_gyr, self.v_yaw):
                if v.get() not in ('---',):
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
    print(' small_car 驾驶台')
    print(' 本机 IP   = %s' % local_ip())
    print(' 监听端口  = %d' % PORT)
    print(' ⇒ wifi_f4.h 里的 WIFI_PC_IP / WIFI_PC_PORT 要和上面一致')
    print(' ⇒ 然后给板子上电（模块会主动连过来）')
    print('=' * 62)

    while True:
        # ---- 等连接（期间保持界面响应）----
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

        # 连上自动问一句状态（顺便确认下行通）
        try:
            conn.sendall(b's\n')
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
                app.root.update()       # ★ 在收数线程里驱动 Tk（按钮/滑条才响应）
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
