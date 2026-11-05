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
DRAW_MIN_MS = 16                # 显示节流：最快约 60fps 刷新一次窗口
                                # ★ 2026-11-02 从 60 改下来：60ms ⇒ 窗口**最多 16.7fps**，
                                #   数据就算跑 30fps，眼睛看到的也只有 16 帧 —— 这正是
                                #   "明明调快了、看着还是 16 帧"的原因（标题里的 FPS 是真实
                                #   完整帧率，不受这里影响；这里只影响"看起来顺不顺"）。
                                #   16ms 对应 60fps 上限，真实帧率低于 60 时它就不会再成为瓶颈。

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


def edge_overlay(im, h, edges, scale):
    """把板子发来的「左右边界数组」画成**红线**（纯显示，不做任何转向判断）

    edges[r] = (左x, 右x)；None = 这一行板子没找到边界
      · 相邻两行都有效、且横向差 ≤12 ⇒ 连一条红线（差太大就不连，免得一道线横穿画面）
      · 每行再点一个 scale×scale 的红方块 ⇒ 孤立行（两侧都断）也看得见
    """
    d = ImageDraw.Draw(im)
    S = scale
    wd = max(2, S // 2)

    for side in (0, 1):                    # 0 = 左边界   1 = 右边界
        prev = None
        for r in range(min(h, len(edges))):
            x = edges[r][side]
            if x is None:
                prev = None                # 这一行没找到 ⇒ 红线在这里断开
                continue
            px = x * S + S // 2
            py = r * S + S // 2
            if (prev is not None) and (abs(x - prev[0]) <= 12):
                d.line([(prev[1], prev[2]), (px, py)], fill=(255, 0, 0), width=wd)
            d.rectangle([x * S, r * S, x * S + S - 1, r * S + S - 1], fill=(255, 0, 0))
            prev = (x, px, py)
    return im


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
        # ★★ 2026-11-05：板子八邻域巡线算出来的左右边界，整包发来，PC **只画红线** ★★
        #    self.edges      = 最近一包：[(左x, 右x), ...]，None = 这一行没找到（画线时断开）
        #    self.shot_edges = 正在显示的那张图对应的一包（图像和线就不会错位）
        self.edges = None
        self.shot_edges = None
        self.edge_pkts = 0                  # 收到几包（校验通过）
        self.edge_nl = 0                    # 最近一包左边界有效行数
        self.edge_nr = 0                    # 最近一包右边界有效行数
        # ★★ 2026-11-05 晚：卡死自愈 + 可观测 ★★
        #   pend_draw : 有新帧但还没画（等紧随其后的边界包，**图线一起画** ⇒ 每帧只画一次）
        #   resync    : 丢同步/重同步次数（>0 说明链路上掉过字节，但已经自己救回来了）
        #   nbytes    : 一共收到多少字节（判断"链路还活着"还是"板子不发了"）
        self.pend_draw = False
        self.t_draw_due = 0.0
        self.resync = 0
        self.t_frame = 0.0                  # 最近一张完整帧的时刻
        self.nbytes = 0
        self.t_rate = 0.0
        self.n_rate = 0
        self.bps = 0.0                      # 最近 1 秒收到的字节数
        self.need_resync = False
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
            self.bounds = None              # 新帧的边界随后由 0xFD 包更新 ✓
            self.shot = (w, h, pol, bytes(f.buf))   # 快照：板子之后再怎么写都不影响这张
            self.t_frame = time.time()
            # ★★ 先不画：板子的整帧包后面**紧跟**一包边界数组，等它到了**图线一起画** ★★
            #   ⇒ 每帧只画一次（少一半 GUI 开销），也不会出现"先看到旧红线"的闪现。
            #   万一边包丢了：最多等 80ms 就自己画（见 tick()）。
            if not self.pend_draw:
                self.t_draw_due = time.time() + 0.08
            self.pend_draw = True

    # ---------------------------------------------------------------- 边界数组
    def on_edges(self, edges):
        """★ 板子发来的「左右边界数组」（八邻域巡线的结果）
           这里**只画**，不做任何转向判断 —— 用户要求先不做左右转 ✓"""
        self.edge_pkts += 1
        self.edge_nl = sum(1 for e in edges if e[0] is not None)
        self.edge_nr = sum(1 for e in edges if e[1] is not None)
        self.edges = edges
        self.shot_edges = edges             # 挂到当前这张图上
        self.pend_draw = False              # ★ 图+线一起画，一帧只画一次
        self.draw(force=True)               # ★ force：跳过节流，保证"图就是这一帧的线"

    # ---------------------------------------------------------------- 收尾/看门狗
    def tick(self):
        """每收完一批字节调一次。返回 True = 请求解析器"丢到下一个魔数重来"。

        两个作用：
          ① 新帧拍完快照后最多等 80ms 边界包；等不到就先画（线是上一帧的，但图不会停）
          ② 链路还在进字节、却 **3 秒没拼出完整帧** ⇒ 判丢同步 ⇒ 请求重同步
             （以前这种情况就是**永久卡住**，只能重启上位机）"""
        now = time.time()

        # 链路速率（1 秒窗口）：卡住时用它区分"板子不发了"还是"上位机没在解析"
        if self.t_rate == 0.0:
            self.t_rate = now
            self.n_rate = self.nbytes
        elif (now - self.t_rate) >= 1.0:
            self.bps   = (self.nbytes - self.n_rate) / (now - self.t_rate)
            self.t_rate = now
            self.n_rate = self.nbytes

        if self.pend_draw and (now >= self.t_draw_due):
            self.pend_draw = False
            self.draw(force=True)

        if self.need_resync:
            self.need_resync = False
            return True

        if (self.nframe > 0) and ((now - self.t_frame) > 3.0) and (self.bps > 500.0):
            self.t_frame = now              # 别每轮都请求
            self.need_resync = True
            print('[重同步] 3 秒没有完整帧（链路还在进 %.0f B/s）⇒ 丢掉缓冲，重新找魔数'
                  % self.bps)
        return False

    # ---------------------------------------------------------------- 显示
    def draw(self, force=False):
        if self.shot is None:               # ★ 还没有"完整一帧" ⇒ 什么都不画（旧图继续挂着）
            return
        now = time.time()
        if (not force) and ((now - self.t_draw) * 1000.0 < DRAW_MIN_MS):
            return
        self.t_draw = now

        w, h, pol, buf = self.shot         # ★ 画的是**完整一帧的快照**，不是正在填的缓冲

        # ★ 实时 FPS：1 秒窗口内"完整帧"的增量
        if (now - self.t_fps) >= 1.0:
            self.fps   = (self.nframe - self.n_fps) / (now - self.t_fps)
            self.t_fps = now
            self.n_fps = self.nframe
        if HAVE_GUI:
            # ★★ 整块 GUI 操作套 try ★★
            #   以前一个 TclError / Tk 异常会从 on_row 一路抛到 run() ⇒ **解析线程直接结束**
            #   ⇒ 窗口还挂着、图永远不动（"运行一段时间卡住"的可能原因之一）。
            #   现在只计数 + 报一次，收数/解析继续跑 ✓
            try:
                im = Image.frombytes('L', (w, h), buf)

                im = im.resize((w * SCALE, h * SCALE), Image.NEAREST)

                # ★★★ 左右边界线：**板子八邻域巡线算好的点**，这里只负责画成红线 ★★★
                #   🔑 2026-11-05 改：以前这里是**PC 自己**按"从中间往左扫、上行±20"重算一遍边界
                #      ⇒ 屏幕上那条红线和板子真正在用的边界其实是两套东西。
                #      现在板子把 120 对 (左x, 右x) 整包发来（0xFD 包），PC 只画
                #      ⇒ **看到的线 = 单片机实际用的线** ✓
                #   0xFF（这里转成 None）= 这一行没找到边界 ⇒ 线在那一行断开，不瞎连 ✓
                if self.shot_edges:
                    im = im.convert('RGB')
                    edge_overlay(im, h, self.shot_edges, SCALE)

                self.tkimg = ImageTk.PhotoImage(im)
                self.lbl.configure(image=self.tkimg)
                # ★ 标题带上两个"跑不掉"的量（包头 byte8 / byte9）：
                #     ER  = DCMI 看门狗**重启过几次**（低 8 位）
                #     PIN = 活引脚电平
                _v = getattr(self, 'bpl', 0)      # 解析器把 byte8/byte9 拼成了它
                _age = (time.time() - self.t_frame) if self.t_frame else -1.0
                self.root.title('small_car 图传  极性=%d  ER=%d  PIN=%X  完整帧 %d  FPS=%.1f  '
                                '行 好%d/坏%d  边包 %d (L%d/R%d)  RS=%d  无新帧 %.1fs  %.0fB/s  %dx%d'
                                % (pol, (_v >> 8) & 0xFF, _v & 0xFF,
                                   self.nframe, self.fps, self.okrows, self.badrows,
                                   self.edge_pkts, self.edge_nl, self.edge_nr,
                                   self.resync, max(_age, 0.0), self.bps, w, h))
                self.root.update()
            except Exception as _e:
                self.gui_err = getattr(self, 'gui_err', 0) + 1
                if self.gui_err in (1, 10, 100, 1000):
                    print('[GUI 异常 %d] %s（不影响收数，继续跑）' % (self.gui_err, _e))

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
        # ★★ 2026-11-05：**每 10 秒无条件打一行**（哪怕一行数据都没有）★★
        #   🔑 卡住的时候，这一行就是判据：
        #        B/s = 0                ⇒ 板子/链路没在发（板子侧问题）
        #        B/s > 0 但"无新帧"很大 ⇒ 上位机丢同步了（会自己重同步，RS 会涨）
        age = (now - self.t_frame) if self.t_frame else -1.0
        print('[统计] 帧 %d (FPS %.1f)  无新帧 %.1fs  收 %.0f B/s  重同步 %d  行 %d (坏 %d)%s'
              % (self.nframe, self.fps, max(age, 0.0), self.bps, self.resync,
                 rows, self.badrows,
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


def parse_stream(v, buf):
    """把缓冲里**所有完整的包**解析掉；不完整的那截留在 buf 里等下一批。

    ★ 2026-11-05 从 run() 里提出来（逻辑一字没改），只为两件事：
      ① 外面能套 try/except —— 解析里任何异常都不许弄死收数线程；
      ② 好测（假 TCP 流直接喂进来就行）。
    """
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
                    del buf[:]
                return

            # 否则跳到下一个"可能是魔数开头"的位置（0xAA），避免逐字节 O(n²)
            k = buf.find(b'\xAA', 1)
            if k > 0:
                del buf[:k]
            else:
                del buf[:1]
            continue                        # 继续在本批数据里找

        if i > 0:
            v.on_text(buf[:i].decode('ascii', 'replace'))
            del buf[:i]

        # 此时 buf 一定以魔数开头
        # 包头 **12 字节**： magic(4) | 行号 | W | 极性 | H | 长度2 | 预留2
        #   （逐行包只用到前 10 字节；整帧/边界数组包用 [8..9] 存长度、[10..11] 不用）
        if len(buf) < 12:
            return                          # 包头还没收全
        row, w, pol, h = buf[4], buf[5], buf[6], buf[7]

        # ★★ 边界数组包（row=0xFD）★★
        #    载荷 = 每行 [左边界x][右边界x]；0xFF = 这一行没找到
        #    🔑 这是**板子八邻域巡线的结果**，PC 只负责画红线（不再自己扫）
        if row == 0xFD:
            elen  = (buf[8] << 8) | buf[9]
            erows = w
            # ★ 先验长度再等数据：假魔数（缓冲区刚好错位时）可能报出 6 万多字节，
            #   若不验，就会**一直等一个永远来不齐的包** ⇒ 看起来就是"卡死"。
            if not (2 <= elen <= 480 * 2 and (elen % 2) == 0 and 1 <= erows <= 480):
                v.resync += 1
                del buf[:1]                 # 假包头 ⇒ 挪 1 字节重找
                continue
            need = 12 + elen + 2
            if len(buf) < need:
                return
            pay = bytes(buf[12:12 + elen])
            ck2 = sum(pay) & 0xFFFF
            if ((buf[12 + elen] << 8) | buf[12 + elen + 1]) == ck2:
                ed = []
                for rr in range(min(erows, elen // 2)):
                    l  = pay[2 * rr]
                    r2 = pay[2 * rr + 1]
                    ed.append((None if l  >= 188 else l,
                               None if r2 >= 188 else r2))
                v.on_edges(ed)
            del buf[:need]
            continue

        # ★★ 边界行包（row=0xFE）：老固件的包，直接吐掉（画线功能已取消）★★
        if row == 0xFE:
            need = 12 + w
            if len(buf) < need:
                return
            del buf[:need]
            continue

        if row == 0xFF:
            # ★★ 整帧包：载荷 = 行数 × 每行 24 字节（位打包）
            #   长度在 pkt[8..9]（2 字节）；一帧只有 1 个包 ⇒ 模块包开销几乎消失 ✓✓
            flen = (buf[8] << 8) | buf[9]
            # ★ 同上：先验长度（必须是 24 的整数倍、不超过 480 行），假包直接跳过
            if not (24 <= flen <= 480 * 24 and (flen % 24) == 0):
                v.resync += 1
                del buf[:1]
                continue
            need2 = 12 + flen + 2
            if len(buf) < need2:
                return
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

        # ---------- 逐行包（老格式；`z 0` 或用逐飞助手时才有）----------
        packed = bool(pol & 0x80)          # ★ 包头最高位 = 位打包
        pol   &= 0x07                       # 低 3 位才是极性组号
        bpl = (buf[8] << 8) | buf[9]
        if not (1 <= w <= 752 and 1 <= h <= 480 and row < h):
            del buf[:1]                     # 不合理 ⇒ 假包头，挪 1 字节重找
            continue
        need = 12 + w
        if len(buf) < need:
            return                          # 这一行还没收全，等下一批
        px = bytes(buf[10:10 + w])
        ck = (buf[10 + w] << 8) | buf[11 + w]
        ok = ((sum(px) & 0xFFFF) == ck)
        del buf[:need]                      # 按长度推进
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

            def settle():
                """收尾：画"等边界包"的帧；必要时丢到下一个魔数重来。

                ★ 不重绑 buf（只 del 内容）⇒ 闭包直接改外层的 bytearray ✓"""
                if v.tick():
                    v.resync += 1
                    k = buf.find(MAGIC, 1)
                    print('[重同步] 缓冲 %d B ⇒ 丢到下一个魔数' % len(buf))
                    if k > 0:
                        del buf[:k]
                    else:
                        del buf[:]

            while True:
                try:
                    d = conn.recv(8192)
                    if not d:
                        print('[对方断开]')
                        CUR['conn'] = None
                        break
                    buf += d
                    v.nbytes += len(d)
                except socket.timeout:
                    settle()                       # ★ 没数据也要让"延迟画帧"有机会画出来
                    v.report(len(buf))
                except Exception as e:
                    print('[接收出错] %s' % e)
                    break

                # ---------------- 解析（实现见 parse_stream）----------------
                try:
                    parse_stream(v, buf)
                except Exception as e:
                    # ★ 解析里任何异常都不许弄死这条线程（否则窗口还在、图永远不动）
                    v.resync += 1
                    print('[解析异常] %s ⇒ 丢 1 字节重找魔数' % e)
                    del buf[:1]

                # ★ 收尾：① 把"等边界包"的帧画出来 ② 3 秒没成帧就请求重同步
                settle()
    except KeyboardInterrupt:
        pass
    finally:
        v.close()
        print('\n[退出]')


if __name__ == '__main__':
    run()
