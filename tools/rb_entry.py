# -*- coding: utf-8 -*-
"""环岛"开头"检测的候选判据，在存下来的帧上跑一遍，看命中率随时间怎么分布。

用户口径（2026-11-07 21:56）："环岛中间就是直线，所以有开头就可以了"
⇒ 只要**认出入口那一下**，之后保持直走；不需要精确跟踪/退出逻辑。

候选：
  R1 岛块  ：黑连通块 高>=8 宽>=30 面积>=350，块下面（r1+1..r1+4）还是白
  R2 宽白  ：贴近车的行(90..114)里最大白宽 >= 1.30 x 正常宽(120)
  R3 = R1 且 R2

用法：
    python rb_entry.py frames_round2 1 3154 50
    python rb_entry.py frames_round  1 1521 50
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
TXW, CAMH = 188, 120
BM_CLOSE = 2          # 黑游程间 <=2px 白缝先并掉
MBR = 3               # 窄于 3px 的黑游程丢掉（细黑桥）
BLACK_MERGE = 4       # 白游程并缝（和固件一致，只为取"白宽"）

# R1
I_MIN_H, I_MIN_W, I_MIN_AREA = 8, 30, 350
I_BELOW_ROWS = 4
I_BELOW_WHITE = 0.5
# R2
W_K = 1.30


def load(path):
    im = Image.open(path).convert('L')
    px = im.load()
    return [[1 if px[x, y] > 127 else 0 for x in range(TXW)] for y in range(CAMH)]


def black_runs(row):
    rr = []
    s = None
    e = -1
    for c in range(TXW):
        if not row[c]:
            if s is None:
                s = c
            e = c
        elif s is not None:
            rr.append([s, e])
            s = None
    if s is not None:
        rr.append([s, e])
    out = []
    for a, b in rr:
        if out and a - out[-1][1] - 1 <= BM_CLOSE:
            out[-1][1] = b
        else:
            out.append([a, b])
    return [t for t in out if t[1] - t[0] + 1 >= MBR]


def white_width(img, r):
    """该行最左白到最右白的跨度（和固件 trace 的 union 口径一致）"""
    xs = [c for c in range(TXW) if img[r][c]]
    if not xs:
        return 0
    return xs[-1] - xs[0]


def r1_island(img):
    comps = []
    prev = []
    for r in range(CAMH):
        cur = []
        used = [False] * len(prev)
        for (a, b) in black_runs(img[r]):
            ci = None
            for i, (pa, pb, pci) in enumerate(prev):
                if used[i]:
                    continue
                if min(b, pb) - max(a, pa) >= 0:
                    used[i] = True
                    ci = pci
                    break
            if ci is None:
                comps.append(dict(r0=r, r1=r, x0=a, x1=b, maxw=b - a + 1,
                                  area=b - a + 1))
                ci = len(comps) - 1
            else:
                c = comps[ci]
                c['r1'] = r
                c['x0'] = min(c['x0'], a)
                c['x1'] = max(c['x1'], b)
                c['maxw'] = max(c['maxw'], b - a + 1)
                c['area'] += b - a + 1
            cur.append((a, b, ci))
        prev = cur

    best = None
    for c in comps:
        h = c['r1'] - c['r0'] + 1
        if h < I_MIN_H or c['maxw'] < I_MIN_W or c['area'] < I_MIN_AREA:
            continue
        if c['r1'] > CAMH - 12:                  # 底下没有白 ⇒ 不是"岛"
            continue
        # 块下面还要是白的（车前方是赛道）
        white = 0
        for rr in range(c['r1'] + 1, min(CAMH, c['r1'] + 1 + I_BELOW_ROWS)):
            seg = img[rr][c['x0']:c['x1'] + 1]
            white += sum(seg)
        cov = white / float(I_BELOW_ROWS * (c['x1'] - c['x0'] + 1))
        if cov < I_BELOW_WHITE:
            continue
        if best is None or c['area'] > best['area']:
            best = dict(c, cov=cov)
    return best


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else 'frames_round2'
    a = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    b = int(sys.argv[3]) if len(sys.argv) > 3 else 10 ** 9
    step = int(sys.argv[4]) if len(sys.argv) > 4 else 50
    fs = sorted(glob.glob(os.path.join(HERE, d, '*.png')))
    n = 0
    h1 = h2 = h3 = 0
    for i in range(a - 1, min(b, len(fs)), step):
        img = load(fs[i])
        isl = r1_island(img)
        wide = max(white_width(img, r) for r in range(90, 115)) >= 120 * W_K
        n += 1
        h1 += 1 if isl else 0
        h2 += 1 if wide else 0
        h3 += 1 if (isl and wide) else 0
        print('%4d  R1=%s%s R2=%s  R3=%s' % (
            i + 1,
            'Y' if isl else '.',
            (' %dx%d@%d-%d cov=%.2f' % (isl['maxw'], isl['r1'] - isl['r0'] + 1,
                                        isl['r0'], isl['r1'], isl['cov'])) if isl else ' ' * 22,
            'Y' if wide else '.',
            'Y' if (isl and wide) else '.'))
    print('---- %s: n=%d R1=%d R2=%d R3=%d' % (d, n, h1, h2, h3))


if __name__ == '__main__':
    main()
