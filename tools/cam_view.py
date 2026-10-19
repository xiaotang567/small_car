# -*- coding: utf-8 -*-
"""
small_car 图传上位机  ——  在电脑上运行，充当 **TCP 服务端**，等 F407 通过 WiFi 连上来。

数据流里混着两种东西：
  1) 纯文本心跳行   CAM1 ... / CAM2 ...           —— 直接打印
  2) 二进制**行包**（一次一行，很小）               —— 拼成图显示

行包格式（v3）：
    0xAA 0x55 0xA5 0x5A | 行号 | W | 极性组号 | H | 像素[W] | 校验和高8 低8

⚠️ 为什么从"整帧一大包"改成"一行一小包"：
   实测 WiFi 这条链路上**一坨 11KB 的载荷会丢字节**，而"整帧一个校验和"只要丢 1 字节
   就整帧作废 ⇒ 图全是花条、而且解析器还容易卡住。
   拆成 198 字节的小包后，丢字节最多毁掉**那一行**，整幅图仍然可用。

用法：
    python cam_view.py [端口]        默认 8086
"""

import socket
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8086
SCALE = 4                       # 显示放大倍数
DRAW_MIN_MS = 60                # 显示节流：最快约 16fps 刷新一次窗口

MAGIC = b'\xAA\x55\xA5\x5A'

try:
    import tkinter as tk
    from PIL import Image, ImageTk, ImageDraw
    HAVE_GUI = True
except Exception as _e:
    HAVE_GUI = False
    print('[提示] 没有 tkinter 或 Pillow（%s）⇒ 改成"存文件"模式' % _e)

try:
    from PIL import Image
    HAVE_PIL = True
except Exception:
    HAVE_PIL = False


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        return '??'
    finally:
        s.close()


def score_vert(buf, w, h):
    """竖向连续性：相邻两行同列的差值绝对和（越小说明越像一张正常的图）"""
    tot = 0
    for y in range(1, h):
        a = buf[(y - 1) * w:y * w]
        b = buf[y * w:(y + 1) * w]
        tot += sum(abs(b[i] - a[i]) for i in range(w))
    return tot // max(h - 1, 1)


class Frame(object):
    """一个"极性组"对应一张正在拼的图"""
    def __init__(self, w, h, pol):
        self.w = w
        self.h = h
        self.pol = pol
        self.buf = bytearray(w * h)
        self.got = set()
        self.bad = 0

    def put(self, row, px, ck_ok):
        # ★ 不管校验过不过，都写进去 —— 目标优先是"能流畅看到图"。
        #   校验不过的行在图上表现为一道花条，比"整块黑、什么都看不见"好得多；
        #   坏行数照样统计，用来衡量链路质量。
        self.buf[row * self.w:(row + 1) * self.w] = px
        self.got.add(row)
        if not ck_ok:
            self.bad += 1


class Viewer(object):
    def __init__(self):
        self.frames = {}          # (pol,w,h) -> Frame
        self.nframe = 0           # 完整帧数
        self.okrows = 0
        self.badrows = 0
        self.texts = 0
        self.shot = None
        self.shot = None                    # ★ 最近一张**完整帧**的快照 (w, h, pol, bytes) —— 只画它
        self.fps = 0.0                      # ★ 实时帧率（用"完整帧"计数在 1s 窗口内的增量算）
        self.t_fps = 0.0
        self.n_fps = 0
        self.bounds = None                  # ★ 每行的亮/暗交界列（板子算好发来，这里只画）
        self.t_last = 0.0
        self.t_draw = 0.0
        self.root = None
        self.lbl = None
        self.tkimg = None
        if HAVE_GUI:
            self.root = tk.Tk()
            self.root.title('small_car 图传')
            self.lbl = tk.Label(self.root)
            self.lbl.pack()
            self.root.update()

    # ---------------------------------------------------------------- 文本
    def on_text(self, txt):
        for ln in txt.split('\n'):
            ln = ln.rstrip('\r')
            if not ln.strip():
                continue
            if not (len(ln) <= 130 and all(32 <= ord(c) < 127 for c in ln)):
                continue

            self.texts += 1

            # 一律照旧写进 cam_log.txt —— **文件不嫌吵**，事后随时可翻
            try:
                with open('cam_log.txt', 'a', encoding='utf-8') as f:
                    f.write(ln + '\n')
            except Exception:
                pass

            # ---------------- ★★ 控制台降噪策略 ★★ ----------------
            #   ① 命令回执（ACK ... / CFG ...）与**摄像头事件（CAM ...）** ⇒ **永远打印**
            #      ⚠️ 2026-10-15 踩坑：原来只放行 ACK/CFG ⇒ 板子发的 `CAM FOUND!` / `CAM CFG: bin4 ...`
            #         **被这里静默掉了**，让人误以为"板子没发"（其实日志文件里有）✗
            #      ⇒ 凡是"板子主动报告的重要状态"都要在这里放行 ✓
            #   ② 板子那行状态（S ...）⇒ **最快 HB_MIN_S 秒一次**，其余静默并计数
            #   ③ 其余（全量诊断 CAM1/CAM2/CAM3/BPF/...）⇒ 只有 VERB >= 1 才打印
            if ln.startswith('ACK') or ln.startswith('CFG') or ln.startswith('CAM'):
                print('  ' + ln)
                continue

            if ln.startswith('S '):
                if (time.time() - _hb_last[0]) >= HB_MIN_S:
                    extra = ''
                    if _hb_mute[0]:
                        extra = '   (期间 %d 条状态已静默)' % _hb_mute[0]
                        _hb_mute[0] = 0
                    _hb_last[0] = time.time()
                    print('  ' + ln + extra)
                else:
                    _hb_mute[0] += 1
                continue

            if VERB >= 1:
                print('  ' + ln)

    # ---------------------------------------------------------------- 行包
    def on_row(self, row, w, pol, h, px, ck_ok):
        key = (pol, w, h)
        f = self.frames.get(key)
        if f is None:
            f = self.frames[key] = Frame(w, h, pol)

        f.put(row, px, ck_ok)
        if ck_ok:
            self.okrows += 1
        else:
            self.badrows += 1

        # ★★ 只有**收满一整圈**才更新显示（关键）★★
        #   🔥 为什么：120 行是在 720ms 里**逐行**到达的，而这里原来**每收到一行就重画**
        #      ⇒ 屏幕上必然一半是新帧、一半是旧帧 ⇒ 一道横着爬的"分割线"
        #      （"上一张图片没有被完全覆盖"就是这个）。
        #   ✅ 收满才拍快照并显示 ⇒ 画面永远是一整帧，接缝消失。
        #      代价：刷新率 = 板子的帧率（现在 1 帧/秒）—— 这正是我们要的。
        if len(f.got) >= h:
            self.nframe += 1
            f.got = set()
            self.bounds = None              # 新帧的边界随后由 0xFE 包更新 ✓
            self.shot = (w, h, pol, bytes(f.buf))   # 快照：板子之后再怎么写都不影响这张
            self.draw()

    # ---------------------------------------------------------------- 显示
    def draw(self):
        if self.shot is None:               # ★ 还没有"完整一帧" ⇒ 什么都不画（旧图继续挂着）
            return
        now = time.time()
        if (now - self.t_draw) * 1000.0 < DRAW_MIN_MS:
            return
        self.t_draw = now

        w, h, pol, buf = self.shot         # ★ 画的是**完整一帧的快照**，不是正在填的缓冲

        # ★ 实时 FPS：1 秒窗口内"完整帧"的增量
        if (now - self.t_fps) >= 1.0:
            self.fps   = (self.nframe - self.n_fps) / (now - self.t_fps)
            self.t_fps = now
            self.n_fps = self.nframe
        if HAVE_GUI:
            im = Image.frombytes('L', (w, h), buf)

            im = im.resize((w * SCALE, h * SCALE), Image.NEAREST)

            # ★ 帧率已由**单片机自己画进图像**（左上角 FPS xx.x）⇒ 上位机不再覆盖，**只负责接收显示** ✓
            #   （标题栏那个 FPS 仅作参考，不改图像✓）

            self.tkimg = ImageTk.PhotoImage(im)
            self.lbl.configure(image=self.tkimg)
            # ★ 标题带上两个"跑不掉"的量（包头 byte8 / byte9）：
            #     ER  = DCMI 看门狗**重启过几次**（低 8 位）
            #     PIN = 活引脚电平： bit3=PA6(PCLK) bit2=PB7(VSYNC) bit1=**PA4(HREF)** bit0=PC6(D0)
            _v = getattr(self, 'bpl', 0)          # 解析器把 byte8/byte9 拼成了它
            self.root.title('small_car 图传  极性=%d  ER=%d  PIN=%X  完整帧 %d  FPS=%.1f  行 好%d/坏%d  %dx%d'
                            % (pol, (_v >> 8) & 0xFF, _v & 0xFF,
                               self.nframe, self.fps, self.okrows, self.badrows, w, h))
            self.root.update()

            # ★★ 每 1.2 秒把"PC 拼出来的整帧"按**极性组号**存一张 PNG ★★
            #   🔑 目的：**让 AI 直接看图**挑极性 —— 不再靠"哪里花、线在哪"这种文字描述。
            #      文件名 = `cam_pol<组号>.png`（存在运行目录，和 cam_log.txt 同一个目录）
            #      ⇒ 板上开"普查模式"轮流换极性时，**8 个组各自留下自己那张图** ✓
            if now - getattr(self, 't_save', 0) > 1.2:
                self.t_save = now
                try:
                    im.save('cam_pol%d.png' % pol)
                except Exception:
                    pass
        else:
            try:
                Image.frombytes('L', (w, h), buf).save('cam_frame.png')
            except Exception:
                with open('cam_frame.pgm', 'wb') as fp:
                    fp.write(b'P5\n%d %d\n255\n' % (w, h))
                    fp.write(buf)

    # ---------------------------------------------------------------- 统计
    def report(self, buf_len=-1):
        now = time.time()
        if now - self.t_last < 10.0:        # ★ 3 秒 → 10 秒（这行本来就是"心跳"，不是实时量）
            return
        self.t_last = now

        rows = self.okrows + self.badrows
        # ★ 这 10 秒内**一行新数据都没有** ⇒ 直接静默（板子掉线时不再每 10 秒刷一句废话）
        if rows == getattr(self, 'r_rows', -1):
            return
        self.r_rows = rows

        print('[统计] 帧 %d  行 %d (坏 %d, %.1f%%)  文本 %d 行%s'
              % (self.nframe, rows, self.badrows,
                 100.0 * self.badrows / max(rows, 1), self.texts,
                 ('  | 缓冲 %d B' % buf_len) if buf_len >= 0 else ''))

    def close(self):
        if self.root is not None:
            try:
                self.root.destroy()
            except Exception:
                pass


# ---------------------------------------------------------------- ★ 控制台 → 板子
#   🔑 板子是 **TCP 客户端**（它连我们），所以电脑没法另开一条连接去发命令
#      ⇒ 命令必须复用**当前这条**已经建立好的连接。
#   用法：在运行本程序的**控制台里直接打字 + 回车**，就会把这一行原样发给板子。
#         板子会回一行 ACK/状态（形如 CFG f=.. / ACK v: ...），直接打印在下面。
import threading
import os

CUR = {'conn': None}

# ★★ 控制台噪声档位（**本地**，不发给板子）★★
#   0 = 默认。只打印：① 你打的命令的回执（ACK/CFG）② 板子那行状态（**最快 5 秒一次**，其余静默计数）
#   1 = 再加上板子的全量诊断行（CAM1/CAM2/CAM3/BPF/MD/PIN/P —— 只有板子开 d=2 时才会有）
#   🔑 目的：调参数/看图时，控制台应该只剩"我发了什么 / 板子回了什么"。
VERB = 0

HB_MIN_S    = 5.0          # 板子那行 "S ..." 最快多久打印一次
_hb_last    = [0.0]        # 上次打印时刻
_hb_mute    = [0]          # 期间被静默掉多少条


def stdin_loop():
    global VERB
    while True:
        try:
            line = sys.stdin.readline()
        except Exception:
            return
        if not line:
            return
        line = line.strip()
        if not line:
            continue

        # ---------------- ★ 本地命令（# 或 / 开头，**不发给板子**）----------------
        if line[0] in '#/':
            cmd = line[1:].strip().lower()
            if cmd in ('h', 'help', '?'):
                print(HELP)
            elif cmd in ('c', 'cls', 'clear'):
                os.system('cls' if os.name == 'nt' else 'clear')
            elif cmd.startswith('v'):
                try:
                    VERB = int(cmd[1:].strip())
                except Exception:
                    print('[本地] 用法：#v 0  或  #v 1')
                    continue
                print('[本地] 控制台档位 = %d' % VERB)
            else:
                print('[本地] 未知：#%s（#h 看帮助）' % cmd)
            continue

        # ---------------- 其余一律原样发给板子（记得结尾补 \n）----------------
        c = CUR['conn']
        if c is None:
            print('[提示] 板子还没连上，先给板子上电')
            continue
        try:
            c.sendall((line + '\n').encode('ascii', 'ignore'))
            print('> %s' % line)
        except Exception as e:
            print('[发送失败] %s' % e)


HELP = (
    '  ── 默认已经就是"能用"的状态，不用敲任何命令 ─────────────\n'
    '     开机 = 1 帧/秒 + 188x120（放得下）+ 控制台静默\n'
    '  ── 想改再打字 + 回车（发给板子）─────────────────────────\n'
    '    f <n>    帧率 帧/秒(0=不限)        g <ms>  每行间隔(默认6)\n'
    '    x <n>    横向抽点(1=188列)         y <n>   纵向抽点(1=120行)\n'
    '    d <n>    心跳档位：0静默(默认) / 1一行 / 2全量诊断\n'
    '    a 恢复默认    s 回状态    ? 帮助\n'
    '    b <n> 摄像头串口波特率 0..4   v 探活\n'
    '    c <cmd> <val> 发一个配置包(例: c 3 10)   i 发INIT   p 读两根线电平\n'
    '  ── 本地命令（# 开头，不发给板子）─────────────────────────\n'
    '    #v 0|1|2  控制台档位     #c 清屏     #h 本帮助\n'
)


def run():
    v = Viewer()

    threading.Thread(target=stdin_loop, daemon=True).start()   # ★ 控制台打字就下发

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', PORT))
    srv.listen(1)

    print('=' * 64)
    print(' 本机 IP  = %s' % local_ip())
    print(' 监听端口 = %d' % PORT)
    print(' ⇒ 确认 wifi_f4.h 里的 WIFI_PC_IP / WIFI_PC_PORT 和上面一致')
    print(' ⇒ 然后**给板子上电**（模块会主动连过来）')
    print(HELP)
    print('=' * 64)

    try:
        while True:
            conn, addr = srv.accept()
            print('\n[已连接] %s:%d' % addr)
            print(HELP)
            CUR['conn'] = conn                    # ★ 把这条连接交给"控制台打字"那条线程
            conn.settimeout(1.0)
            buf = bytearray()

            while True:
                try:
                    d = conn.recv(8192)
                    if not d:
                        print('[对方断开]')
                        CUR['conn'] = None
                        break
                    buf += d
                except socket.timeout:
                    v.report(len(buf))
                except Exception as e:
                    print('[接收出错] %s' % e)
                    break

                # ---------------- 解析：★ 逐字节精确，绝不用 decode/encode 去截断缓冲 ----------------
                #   旧版有两个致命问题：
                #     ① 用 `decode('ascii','ignore')` 处理"混了二进制"的缓冲 ⇒ **静默丢字节** ⇒ 错位被放大；
                #     ② 一旦错位 1 字节，`need` 就算错 ⇒ **把后面的文本当成像素吃掉**
                #        ⇒ 现象正是"CAM3 / CAM4 整行消失、只剩 CAM1/CAM2 的尾巴"。
                while True:
                    i = buf.find(MAGIC)

                    if i < 0:
                        # 没有完整魔数：把开头**连续的可打印 ASCII**当文本，遇到非 ASCII 就停
                        j = 0
                        while j < len(buf) and buf[j] < 0x80:
                            j += 1
                        if j > 0:
                            v.on_text(buf[:j].decode('ascii', 'replace'))
                            del buf[:j]

                        # ★★★ 关键修正 ★★★
                        #   剩下的如果**不足一个魔数的长度**，很可能是"被 recv 切开的魔数前半截"
                        #   ⇒ **必须留着等下一批数据，绝不能丢！**
                        #   丢了就永久失去同步：后面整包数据会被当成文本吃掉
                        #   —— "CAM3 / CAM4 整行消失、只剩 CAM1/CAM2 尾巴"就是这么来的。
                        if len(buf) < len(MAGIC):
                            if len(buf) > 65536:        # 兜底：真错位了别无限攒
                                print('[警告] 缓冲积压过多，丢弃重来')
                                buf = bytearray()
                            break

                        # 否则跳到下一个"可能是魔数开头"的位置（0xAA），避免逐字节 O(n²)
                        k = buf.find(b'\xAA', 1)
                        if k > 0:
                            del buf[:k]
                        else:
                            del buf[:1]
                        continue                    # 继续在本批数据里找

                    if i > 0:
                        v.on_text(buf[:i].decode('ascii', 'replace'))
                        del buf[:i]

                    # 此时 buf 一定以魔数开头
                    # 包头 **10 字节**： magic(4) | 行号 | W | 极性 | H | BPL高 | BPL低
                    #   BPL = 板子实测的"每行字节数" ⇒ 显示在窗口标题上（不依赖会被截断的文本心跳）
                    if len(buf) < 12:
                        break                       # 包头还没收全
                    row, w, pol, h = buf[4], buf[5], buf[6], buf[7]

                    # ★★ 边界行包（row=0xFE）：载荷 = 每行的亮/暗交界列
                    #    **单片机已经算好**，这里只存下来画蓝线 ✓（pkt[9] = Otsu 阈值）
                    if row == 0xFE:
                        # 边界包：蓝线功能已取消（用户要求）⇒ **只把它吐掉**，
                        # 保留这个分支是为了兼容"老固件还在发边界包"的情况（不会错位）✓
                        need = 12 + w
                        if len(buf) < need:
                            break
                        del buf[:need]
                        continue

                    if row == 0xFF:
                        # ★★ 整帧包（新）：载荷 = 行数 × 每行 24 字节（位打包）
                        #   长度在 pkt[8..9]（2 字节）；一帧只有 1 个包 ⇒ 模块包开销几乎消失 ✓✓
                        if len(buf) < 14:
                            break
                        flen = (buf[8] << 8) | buf[9]
                        need2 = 12 + flen + 2
                        if len(buf) < need2:
                            break
                        pl   = bytes(buf[12:12 + flen])
                        polf = pol & 0x07
                        nrow = h
                        bw   = 24
                        for rr in range(min(nrow, flen // bw)):
                            seg = pl[rr * bw:(rr + 1) * bw]
                            pxp = bytearray(188)
                            for i2, byte in enumerate(seg):
                                for k2 in range(8):
                                    idx = i2 * 8 + k2
                                    if idx < 188 and ((byte >> (7 - k2)) & 1):
                                        pxp[idx] = 255
                            v.on_row(rr, 188, polf, nrow, bytes(pxp), True)
                        del buf[:need2]
                        continue

                    packed = bool(pol & 0x80)          # ★ 包头最高位 = 位打包
                    pol   &= 0x07                       # 低 3 位才是极性组号
                    bpl = (buf[8] << 8) | buf[9]
                    if not (1 <= w <= 752 and 1 <= h <= 480 and row < h):
                        del buf[:1]                 # 不合理 ⇒ 假包头，挪 1 字节重找
                        continue
                    need = 12 + w
                    if len(buf) < need:
                        break                       # 这一行还没收全，等下一批
                    px = bytes(buf[10:10 + w])
                    ck = (buf[10 + w] << 8) | buf[11 + w]
                    ok = ((sum(px) & 0xFFFF) == ck)
                    del buf[:need]                  # 按长度推进
                    v.bpl = bpl
                    if packed:
                        # ★ 解包：每字节 8 像素（高位在前）⇒ 24 字节 → 188 像素
                        pxp = bytearray(188)
                        for i2, byte in enumerate(px):
                            for k2 in range(8):
                                idx = i2 * 8 + k2
                                if idx < 188 and ((byte >> (7 - k2)) & 1):
                                    pxp[idx] = 255
                        v.on_row(row, 188, pol, h, bytes(pxp), ok)
                    else:
                        v.on_row(row, w, pol, h, px, ok)
    except KeyboardInterrupt:
        pass
    finally:
        v.close()
        print('\n[退出]')


if __name__ == '__main__':
    run()
