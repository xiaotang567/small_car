# -*- coding: utf-8 -*-
"""逆透视标定工具（一条龙）

    连板子 → 抓一张**相机原图** → 鼠标点正方形 4 个角 → 生成查表 → 编译 → 烧录

用法：
    1) **先关掉 small_car 控制台**（8086 端口只能有一个监听）
    2) 双击 run_cal.bat（或 python tools\\ipm_cal.py）
    3) 点「连板子并抓图」→ 画面出来后在白赛道/黑地板上点正方形 4 个角
       （远左 → 远右 → 近右 → 近左，点错就右键撤销；顺序点乱也没关系，程序会自己排）
    4) 填正方形实际边长（米）→ 点「生成+编译+烧录」
    5) 烧完关掉本工具，双击 run_ui.bat 恢复正常使用

点 4 个角之后，右边会显示反解出来的相机姿态（f / 低头角 / 相机高）和重投影误差；
误差 < 3 像素算标得准；> 6 像素说明正方形没摆正（四条边不等长/没对正车头）⇒ 重摆重抓。

★ 固件对应：`cal 1` = 临时发相机原图（只给本工具用），`cal 0` = 恢复发逆透视图。
"""
import io
import os
import socket
import subprocess
import sys
import time

import tkinter as tk
from tkinter import filedialog, messagebox

import numpy as np
from PIL import Image, ImageDraw, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAL = os.path.join(HERE, 'cal')
sys.path.insert(0, HERE)

import gen_ipm_lut_cal as G

PORT = 8086
MAGIC = b'\xAA\x55\xA5\x5A'
W, H = 188, 120
K = 4                                   # 显示放大倍数
_BIT8 = [bytes(255 if (b >> (7 - k)) & 1 else 0 for k in range(8)) for b in range(256)]

UVPROJ = os.path.join(ROOT, 'MDK-ARM', 'small_car.uvprojx')
BUILD_LOG = os.path.join(ROOT, 'MDK-ARM', 'build_out.txt')
HEX = os.path.join(ROOT, 'MDK-ARM', 'small_car', 'small_car.hex')
UV4 = r'C:\Keil_v5\UV4\UV4.exe'
STLINK = (r'C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility'
          r'\ST-LINK Utility\ST-LINK_CLI.exe')

C_BG = '#1e1e26'
C_PANEL = '#26262f'
C_FG = '#e6e6ea'
C_DIM = '#9aa0aa'
C_BTN = '#3a3a48'
C_ON = '#2e7d4f'
C_ACC = '#4aa3ff'


# ====================================================================== 板子连接
class Link(object):
    def __init__(self):
        self.srv = None
        self.conn = None
        self.buf = bytearray()
        self.frame = None

    def listen(self, port=PORT, wait=45.0):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', port))
        s.listen(1)
        s.settimeout(wait)
        self.srv = s

    def accept(self):
        self.conn, addr = self.srv.accept()
        self.conn.setblocking(False)
        return addr

    def send(self, cmd):
        if not self.conn:
            return False
        try:
            self.conn.sendall((cmd + '\n').encode('ascii'))
            return True
        except Exception:
            return False

    def close(self):
        for s in (self.conn, self.srv):
            try:
                if s:
                    s.close()
            except Exception:
                pass
        self.conn = self.srv = None

    def pump(self, secs=0.06):
        """收一段数据、解出整帧；返回 True = 这一轮收到了新帧"""
        got = False
        if not self.conn:
            return False
        end = time.time() + secs
        while time.time() < end:
            try:
                d = self.conn.recv(65536)
                if not d:
                    break
                self.buf += d
            except BlockingIOError:
                time.sleep(0.004)
            except Exception:
                break

        buf = self.buf
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if len(buf) > 3:
                    del buf[:len(buf) - 3]
                break
            if i:
                del buf[:i]
                continue
            if len(buf) < 12:
                break
            if buf[4] != 0xFF:                    # 不是整帧包
                del buf[:1]
                continue
            flen = (buf[8] << 8) | buf[9]
            if not (24 <= flen <= 480 * 24 and flen % 24 == 0):
                del buf[:1]
                continue
            need = 12 + flen + 2
            if len(buf) < need:
                break
            pay = bytes(buf[12:12 + flen])
            if (sum(pay) & 0xFFFF) == ((buf[12 + flen] << 8) | buf[12 + flen + 1]):
                out = bytearray()
                for r in range(H):
                    out += b''.join(_BIT8[b] for b in pay[r * 24:(r + 1) * 24])[:W]
                self.frame = np.frombuffer(bytes(out), dtype=np.uint8).reshape(H, W).copy()
                got = True
            del buf[:need]
        return got


# ====================================================================== 界面
class App(object):
    def __init__(self, root):
        self.root = root
        self.link = Link()
        self.img = None            # 188x120 uint8
        self.photo = None
        self.pts = []              # [[u,v], ...] 鼠标点的 4 个角
        self.pose = None
        self.err = None
        self.bev = None

        root.title('逆透视标定工具  (smal_car IPM calibration)')
        root.configure(bg=C_BG)
        root.geometry('1230x760')

        left = tk.Frame(root, bg=C_BG, padx=8, pady=8)
        left.grid(row=0, column=0, sticky='nsew')
        self.cv = tk.Canvas(left, width=W * K, height=H * K, bg='black',
                            highlightthickness=0, bd=0)
        self.cv.pack()
        self.cv.bind('<Button-1>', self.on_click)
        self.cv.bind('<Button-3>', self.on_rclick)
        tk.Label(left, text='左键点角（最多 4 个）· 右键撤销 · 点完自动拟合',
                 bg=C_BG, fg=C_DIM).pack(anchor='w', pady=(6, 0))

        right = tk.Frame(root, bg=C_PANEL, padx=10, pady=8)
        right.grid(row=0, column=1, sticky='nsew')

        def btn(txt, cmd, bg=C_BTN, w=18):
            b = tk.Button(right, text=txt, command=cmd, bg=bg, fg=C_FG, width=w, height=1,
                          relief='raised', bd=2, cursor='hand2',
                          activebackground='#4c4c60', font=('Microsoft YaHei', 10))
            b.pack(fill='x', pady=2)
            return b

        tk.Label(right, text='① 取图', bg=C_PANEL, fg=C_ACC,
                 font=('Microsoft YaHei', 10, 'bold')).pack(anchor='w')
        btn('连板子并抓图', self.do_grab, C_ON)
        btn('打开 PNG 图片…', self.do_open)

        tk.Label(right, text='② 正方形边长（米）', bg=C_PANEL, fg=C_ACC,
                 font=('Microsoft YaHei', 10, 'bold')).pack(anchor='w', pady=(10, 0))
        self.ent_L = tk.Entry(right, width=10, bg='#303038', fg=C_FG, insertbackground=C_FG)
        self.ent_L.insert(0, '0.45')
        self.ent_L.pack(anchor='w')
        self.ent_L.bind('<KeyRelease>', lambda e: self.refit())

        tk.Label(right, text='③ 生成 / 烧录', bg=C_PANEL, fg=C_ACC,
                 font=('Microsoft YaHei', 10, 'bold')).pack(anchor='w', pady=(10, 0))
        btn('生成查表（不烧录）', lambda: self.do_gen(False))
        btn('生成 + 编译 + 烧录', lambda: self.do_gen(True), C_ON)

        self.lbl = tk.Label(right, text='', bg=C_PANEL, fg=C_FG, justify='left',
                            anchor='w', font=('Consolas', 9))
        self.lbl.pack(fill='x', pady=(10, 0))

        self.log_txt = tk.Text(right, height=16, width=44, bg='#16161c', fg='#c8d0d8',
                               font=('Consolas', 9), relief='sunken', bd=2)
        self.log_txt.pack(fill='both', expand=True, pady=(8, 0))

        self.draw()
        self.show_info()

    # ---------------------------------------------------------------- 画布
    def log(self, s):
        self.log_txt.insert('end', s + '\n')
        self.log_txt.see('end')
        self.root.update_idletasks()

    def render(self):
        """把 self.img + 网格 + 已点的角 + 拟合结果画成一张 RGB 图"""
        if self.img is None:
            im = Image.new('RGB', (W, H), (24, 24, 30))
        else:
            im = Image.fromarray(self.img).convert('RGB')
        im = im.resize((W * K, H * K), Image.NEAREST)
        d = ImageDraw.Draw(im)
        for x in range(0, W, 20):
            d.line([(x * K, 0), (x * K, H * K)], fill=(40, 70, 110))
            d.text((x * K + 2, 1), str(x), fill=(140, 190, 240))
        for y in range(0, H, 20):
            d.line([(0, y * K), (W * K, y * K)], fill=(40, 70, 110))
            d.text((2, y * K + 1), str(y), fill=(140, 190, 240))

        if len(self.pts) == 4 and (self.pose is not None):
            q = G.sort_quad(np.array(self.pts, dtype=float))
            uv = [(float(p[0]) * K, float(p[1]) * K) for p in q]
            d.line(uv + [uv[0]], fill=(0, 255, 0), width=2)
        for i, p in enumerate(self.pts):
            x, y = p[0] * K, p[1] * K
            d.line([(x - 9, y), (x + 9, y)], fill=(255, 60, 60), width=2)
            d.line([(x, y - 9), (x, y + 9)], fill=(255, 60, 60), width=2)
            d.text((x + 6, y - 14), str(i + 1), fill=(255, 220, 0))
        return im

    def draw(self):
        im = self.render()
        self.photo = ImageTk.PhotoImage(im)
        self.cv.delete('all')
        self.cv.create_image(0, 0, image=self.photo, anchor='nw')

    def show_bev(self):
        if self.bev is None:
            return
        big = Image.fromarray(self.bev).resize((W * 3, H * 3), Image.NEAREST)
        big.show(title='俯视图预览（近处在下）')

    # ---------------------------------------------------------------- 鼠标
    def on_click(self, ev):
        if self.img is None:
            self.log('先抓一张图')
            return
        if len(self.pts) >= 4:
            self.log('已经 4 个角了（右键撤销 / 双击画布清空重标）')
            return
        self.pts.append([ev.x / float(K), ev.y / float(K)])
        self.refit()
        self.draw()

    def on_rclick(self, ev):
        if self.pts:
            self.pts.pop()
        self.pose = None
        self.draw()
        self.show_info()

    # ---------------------------------------------------------------- 拟合
    def cur_L(self):
        try:
            return float(self.ent_L.get())
        except Exception:
            return 0.45

    def refit(self):
        if len(self.pts) != 4:
            self.pose = None
            self.show_info()
            return
        try:
            self.pose, self.err = G.fit_from_uv(np.array(self.pts, dtype=float), self.cur_L())
        except Exception as e:
            self.pose = None
            self.log('拟合失败: %s' % e)
            return
        self.show_info()
        self.draw()

    def show_info(self):
        if len(self.pts) != 4 or self.pose is None:
            self.lbl.configure(text='已点 %d/4 个角' % len(self.pts))
            return
        f, tau, h, Xc, Yc, psi = self.pose
        hfov = np.rad2deg(2 * np.arctan(W * 0.5 / f))
        self.lbl.configure(text=(
            'f = %.1f px   HFOV = %.1f°\n'
            '低头 = %.2f°   相机高 = %.3f m\n'
            '正方形中心 前 %.3f 横 %+.3f m  转角 %.2f°\n'
            '重投影误差 最大 %.2f px'
            % (f, hfov, np.rad2deg(tau), h, Xc, Yc, np.rad2deg(psi), self.err.max())))

    # ---------------------------------------------------------------- 动作
    def do_open(self):
        p = filedialog.askopenfilename(initialdir=CAL,
                                       filetypes=[('PNG', '*.png'), ('All', '*.*')])
        if not p:
            return
        im = Image.open(p).convert('L')
        if im.size != (W, H):
            im = im.resize((W, H), Image.NEAREST)
        self.img = np.asarray(im).copy()
        self.pts = []
        self.pose = None
        self.log('已打开：%s' % os.path.basename(p))
        self.draw()

    def do_grab(self):
        try:
            self.link.close()
            self.link.listen(PORT, 45.0)
        except OSError as e:
            self.log('✗ 8086 端口被占用（先关掉 small_car 控制台！）%s' % e)
            return
        self.log('等板子连上来（最多 45 秒）…')
        self.root.update()
        try:
            addr = self.link.accept()
        except socket.timeout:
            self.log('✗ 等超时：板子没连过来。检查车是否上电/同一 WiFi，再点一次')
            self.link.close()
            return
        self.log('板子连上了：%s' % (addr,))
        time.sleep(0.5)
        self.link.send('cal 1')             # ★ 让板子临时发相机原图
        self.log('已发 cal 1（相机原图），收 8 帧…')
        n = 0
        t0 = time.time()
        while n < 8 and (time.time() - t0) < 8.0:
            if self.link.pump(0.1):
                n += 1
        self.link.send('cal 0')             # ★ 恢复发逆透视图
        if self.link.frame is None:
            self.log('✗ 没收到整帧（板子没在发图？）')
            return
        self.img = self.link.frame.copy()
        self.pts = []
        self.pose = None
        tag = time.strftime('cam_%Y%m%d_%H%M%S')
        try:
            os.makedirs(CAL, exist_ok=True)
            Image.fromarray(self.img).save(os.path.join(CAL, tag + '.png'))
            self.log('已存原图：tools\\cal\\%s.png' % tag)
        except Exception as e:
            self.log('存图失败: %s' % e)
        self.log('收到 %d 帧 —— 现在点正方形 4 个角' % n)
        self.draw()
        self.show_info()

    def do_gen(self, then_flash):
        if len(self.pts) != 4 or self.pose is None:
            messagebox.showwarning('还没标', '先在图上点正方形 4 个角')
            return
        L = self.cur_L()
        try:
            info = G.write_lut(L, self.pose, self.err, self.img,
                               os.path.join(CAL, 'bev_new.png'))
        except Exception as e:
            self.log('✗ 生成查表失败: %s' % e)
            return
        self.log('✔ 已生成 ipm_lut.c：HFOV %.1f° 低头 %.2f° 高 %.3f m，有效 %.0f%%，行 %d~%d'
                 % (2 * np.rad2deg(np.arctan(W * 0.5 / info['f'])), info['tau_deg'],
                    info['h'], info['valid_pct'], info['rows_ok'][0], info['rows_ok'][1]))
        self.log('  俯视预览：tools\\cal\\bev_new.png')

        # 顺便在界面里显示俯视预览
        try:
            idx = G.build_index(self.pose)
            bev = np.zeros((H, W), dtype=np.uint8)
            ok = idx['ok']
            bev[ok] = self.img[idx['vi'][ok], idx['ui'][ok]]
            self.bev = np.flipud(bev)
        except Exception:
            self.bev = None

        if not then_flash:
            self.show_bev()
            return
        if not self.build():
            return
        if not self.flash():
            return
        self.log('')
        self.log('✔ 全部完成：关掉本工具 → 双击 run_ui.bat 恢复控制台')

    def build(self):
        if not os.path.exists(UV4):
            self.log('✗ 找不到 Keil：%s' % UV4)
            return False
        self.log('编译中（Keil）…')
        self.root.update()
        try:
            subprocess.run([UV4, '-b', UVPROJ, '-j0', '-o', BUILD_LOG],
                           cwd=os.path.join(ROOT, 'MDK-ARM'),
                           capture_output=True, text=True, errors='replace', timeout=300)
        except Exception as e:
            self.log('✗ 编译调用失败: %s' % e)
            return False
        txt = ''
        try:
            with io.open(BUILD_LOG, encoding='utf-8', errors='replace') as fp:
                txt = fp.read()
        except Exception:
            pass
        tail = [ln for ln in txt.splitlines() if ln.strip()][-4:]
        for ln in tail:
            self.log('  ' + ln)
        if '- 0 Error(s)' not in txt:
            self.log('✗ 编译有错误，先修代码')
            return False
        self.log('✔ 编译通过')
        return True

    def flash(self):
        if not os.path.exists(STLINK):
            self.log('✗ 找不到 ST-LINK_CLI.exe')
            return False
        self.log('烧录中…')
        self.root.update()
        try:
            r = subprocess.run([STLINK, '-c', 'SWD', 'UR', 'FREQ=4000', '-P', HEX, '-V', '-Rst'],
                               capture_output=True, text=True, errors='replace', timeout=180)
            out = (r.stdout or '') + (r.stderr or '')
        except Exception as e:
            self.log('✗ 烧录调用失败: %s' % e)
            return False
        ok = ('Verification...OK' in out) or ('Programming Complete' in out)
        self.log('✔ 烧录完成（板子已复位）' if ok else '✗ 烧录失败，看 ST-LINK 是否插好')
        return ok


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
