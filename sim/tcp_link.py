# -*- coding: utf-8 -*-
"""和实车一样的 TCP 协议（端口 8086），让现有的 tools/car_ui.py 直接连上来调参。

协议对齐 Core/Src/cmd_f4.c 与 app.c：
  板子 → 上位机： CFG / T（遥测 10Hz）/ S（心跳 2Hz）
  上位机 → 板子： s | sp <kp ki kd ×10> | sc <kp kd kff [ki] ×10> | q <rpm> | w <d1> <d2> | s 2 | s 3 | ! | 其它
"""
import socket
import threading
import time


class TcpLink(threading.Thread):
    daemon = True

    def __init__(self, sim, port=8086):
        super().__init__(name='tcp-link')
        self.sim = sim
        self.port = port
        self.srv = None
        self.conn = None
        self.lock = threading.Lock()
        self.alive = True
        self.img_on = True
        self.n_tel = 0

    # ---------- 发送 ----------
    def _send(self, s):
        with self.lock:
            if self.conn is None:
                return
            try:
                self.conn.sendall(s.encode('ascii', 'ignore'))
            except Exception:
                self.conn = None

    def cfg_line(self):
        s = self.sim.scan
        return ('CFG f=20 g=0 x=1 y=1 (188x120) d=1 CST=0 RV=000000 LINE=3 FW=9001 '
                'EDG=%d EL=%d ER=%d DRP=0 DST=0 SELF=0 TH=255\r\n' % (
                    self.sim.state.state, int((s.bound_l != 0xFF).sum()),
                    int((s.bound_r != 0xFF).sum())))

    def tel_line(self):
        t = self.sim.telemetry()
        self.n_tel += 1
        return ('T n=%d r=%d ax=0 ay=0 az=0 gx=0 gy=0 gz=0 y=0.0 s=%d m=1 e=%d cv=%d ds=%d '
                'rs=%d iv=0 tg=%d en=%d ef=%d nc=%d d1=%d d2=0 si=1 sr=0 sb=0 st=0 '
                'sba=1000000 sl=1 k1=0 k2=0 ol=%d\r\n' % (
                    self.n_tel, t['r'], t['servo'], t['e'], t['cv'], t['ds'], t['rs'],
                    t['servo'], t['en'], t['ef'], t['nc'], t['duty'],
                    1 if self.sim.speed.target_x10 == 0 else 0))

    def st_line(self):
        s = self.sim.scan
        return ('S f=%d V=0 VS=0 LG=0 TO=0 F=%d FL=%d P=1 RX=0 CAM=1 ID=1313 N=1 MD=1 BIN=1 '
                'R0D=030A W=120 E=0 K=1134 R=0000 IE=001F MI=0000 CR=00000000 ND=0 '
                'DC=00000000 HE=0 VE=0 LW=0 GP=0 FW=9001 LM=0 RST=00 PC=00000000 CF=0000 '
                'BA=00000000 MG=140 SAT=30 A5=003A SH=0064\r\n' % (
                    int(self.sim.t_ms / 20), self.sim.img.shape[0], s.rows_n))

    # ---------- 收命令 ----------
    def _cmd(self, line):
        line = line.strip('\r\n\x00 ')
        if not line:
            return
        c = line[0]
        rest = line[1:]
        sim = self.sim
        p = sim.p
        try:
            if c == '!':
                return
            if c == 's':
                sub = rest.strip()
                if sub == '2':
                    self.img_on = False
                    self._send('ACK s: image off\r\n')
                elif sub == '3':
                    self.img_on = True
                    self._send('ACK s: image on\r\n')
                else:
                    self._send(self.cfg_line())
                return
            if c == 's' and rest.startswith('p'):
                pass
            if line.startswith('sp'):
                a = [int(float(v) * 10) / 10.0 for v in line[2:].split()]
                if len(a) >= 3:
                    p['kp'], p['ki'], p['kd'] = a[0], a[1], a[2]
                self._send('ACK sp: kp=%.1f ki=%.2f kd=%.1f\r\n' % (p['kp'], p['ki'], p['kd']))
                return
            if line.startswith('sc'):
                a = [int(float(v) * 10) / 10.0 for v in line[2:].split()]
                if len(a) >= 2:
                    p['kp_cur'], p['kd_cur'] = a[0], a[1]
                if len(a) >= 3:
                    p['kff'] = a[2]
                if len(a) >= 4:
                    p['ki_cur'] = a[3]
                self._send('ACK sc: kp=%.1f kd=%.1f kff=%.1f ki=%.2f\r\n' % (
                    p['kp_cur'], p['kd_cur'], p['kff'], p['ki_cur']))
                return
            if c == 'q':
                v = int(float(rest.strip() or '0'))
                if v > 0:
                    sim.start(v)
                else:
                    sim.stop()
                self._send('ACK q: target=%d state=%s\r\n' % (v, sim.state.name()))
                return
            if c == 'w':
                a = rest.split()
                d1 = int(float(a[0])) if a else 0
                sim.speed.set_target(0)
                sim.duty = d1
                self._send('ACK w: d1=%d d2=0\r\n' % d1)
                return
            if c == 'r':
                sim.__init__(sim.p)
                self._send('ACK r: sim reset\r\n')
                return
            self._send('ACK %s: sim\r\n' % c)
        except Exception as e:
            self._send('ACK err: %s\r\n' % e)

    # ---------- 线程 ----------
    def run(self):
        # ★ 和实车一样：**仿真车当客户端**，去连上位机（car_ui.py 在 8086 上监听）。
        #   这样 tools/car_ui.py 完全不用改就能连上仿真车调参。
        self.srv = None
        last_tel = 0.0
        last_st = 0.0
        buf = b''
        while self.alive:
            if self.conn is None:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0)
                    s.connect(('127.0.0.1', self.port))
                    s.settimeout(0.05)
                    self.conn = s
                    buf = b''
                    self._send(self.cfg_line())
                    print('[sim] 已连上上位机 127.0.0.1:%d' % self.port)
                except Exception:
                    try:
                        s.close()
                    except Exception:
                        pass
                    time.sleep(1.0)
                continue

            now = time.time()
            if now - last_tel > 0.1:
                last_tel = now
                self._send(self.tel_line())
            if now - last_st > 0.5:
                last_st = now
                self._send(self.st_line())
            try:
                d = self.conn.recv(4096)
                if not d:
                    print('[sim] 上位机断开')
                    self.conn = None
                    continue
                buf += d
                while b'\n' in buf:
                    ln, buf = buf.split(b'\n', 1)
                    self._cmd(ln.decode('ascii', 'ignore'))
            except socket.timeout:
                pass
            except Exception:
                self.conn = None
