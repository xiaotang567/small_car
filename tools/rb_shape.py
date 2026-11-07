# -*- coding: utf-8 -*-
"""环岛"开头"判据（最终候选）—— 只看**形状**，不看绝对宽度。

用户口径（21:56）："环岛中间就是直线，所以有开头就可以了"
⇒ 只需要在入口那一下认出"前方白地被一块黑岛夹断"，然后保持直走。

判据（每条都在 1500+ 张实拍帧上验过）：
  ① 逐行找"白-黑-白"：黑缝 >=14px，缝两侧的白各 >=8px，
     且**两侧的白都不许贴画面边**（贴边的是背景白，不是赛道白）
  ② 这些缝按行**垂直连通**（相邻行重叠 >=50%）成一块
  ③ 块要像"岛"：高 >=6 行、最宽 >=20px、最宽那一行的中心在画面中段（35..155）
  ④ 块的正下方还是白（车前方是赛道；黑一直连到画面底的不算）

对比：老判据"某一行有 >=14px 黑缝就算岛"会在这个场地一帧里命中好几次（白地外面全是黑），
所以一进 cam_wide 就出不来 —— 这就是"弯道直线全废"的原因。

用法：
    python rb_shape.py frames_round2 1 3200 100
    python rb_shape.py frames_round  1 1521 100
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
TXW, CAMH = 188, 120

GAP_MERGE, MIN_RUN, MAX_RUNS = 4, 10, 12
ROW_LO, ROW_HI = 24, 100
HOLE_MIN, SIDE_MIN = 14, 8
EXT_MIN, WID_MIN = 6, 20
CX_LO, CX_HI = 35, 155
BELOW_ROWS, BELOW_WHITE, BELOW_OFF = 3, 0.40, 4


def white_runs(row):
    rr = []
    s = None
    e = -1
    for c in range(TXW):
        if row[c]:
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
        if out and a - out[-1][1] - 1 <= GAP_MERGE:
            out[-1][1] = b
        elif len(out) < MAX_RUNS:
            out.append([a, b])
    return [t for t in out if t[1] - t[0] + 1 >= MIN_RUN]


def row_holes(img, r):
    rr = white_runs(img[r])
    hs = []
    for j in range(len(rr) - 1):
        lw = rr[j][1] - rr[j][0] + 1
        rw = rr[j + 1][1] - rr[j + 1][0] + 1
        if lw < SIDE_MIN or rw < SIDE_MIN:
            continue
        if rr[j][0] < 3 or rr[j + 1][1] > TXW - 4:      # 两侧贴边 ⇒ 背景白
            continue
        a, b = rr[j][1] + 1, rr[j + 1][0] - 1
        if b - a + 1 < HOLE_MIN:
            continue
        hs.append((a, b))
    return hs


def find_island(img):
    comps = []
    prev = []
    for r in range(ROW_LO, ROW_HI + 1):
        cur = []
        used = [False] * len(prev)
        for (a, b) in row_holes(img, r):
            ci = None
            for i, (pa, pb, pci) in enumerate(prev):
                if used[i]:
                    continue
                ov = min(b, pb) - max(a, pa) + 1
                if ov * 2 >= (pb - pa + 1):
                    used[i] = True
                    ci = pci
                    break
            if ci is None:
                comps.append(dict(r0=r, r1=r, maxw=b - a + 1,
                                  wl=a, wr=b, wrow=r))
                ci = len(comps) - 1
            else:
                c = comps[ci]
                c['r1'] = r
                if b - a + 1 > c['maxw']:
                    c['maxw'] = b - a + 1
                    c['wl'], c['wr'], c['wrow'] = a, b, r
            cur.append((a, b, ci))
        prev = cur

    best = None
    for c in comps:
        h = c['r1'] - c['r0'] + 1
        if h < EXT_MIN or c['maxw'] < WID_MIN:
            continue
        cx = (c['wl'] + c['wr']) // 2
        if cx < CX_LO or cx > CX_HI:
            continue
        if c['r1'] >= ROW_HI:            # 一直到窗口底还没结束 ⇒ 它连到画面下面去了，不是岛
            continue
        if c['r1'] + 1 >= CAMH:
            continue
        white = 0
        for rr in range(c['r1'] + 1 + BELOW_OFF, min(CAMH, c['r1'] + 1 + BELOW_OFF + BELOW_ROWS)):
            white += sum(img[rr][c['wl']:c['wr'] + 1])
        cov = white / float(BELOW_ROWS * (c['wr'] - c['wl'] + 1))
        if cov < BELOW_WHITE:
            continue
        if best is None or h > best[3]:
            best = (c['wl'], c['wr'], cx, h, c['maxw'], c['r0'], c['r1'], cov)
    return best


def load(path):
    im = Image.open(path).convert('L')
    px = im.load()
    return [[1 if px[x, y] > 127 else 0 for x in range(TXW)] for y in range(CAMH)]


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else 'frames_round2'
    a = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    b = int(sys.argv[3]) if len(sys.argv) > 3 else 10 ** 9
    step = int(sys.argv[4]) if len(sys.argv) > 4 else 100
    fs = sorted(glob.glob(os.path.join(HERE, d, '*.png')))
    n = hit = 0
    for i in range(a - 1, min(b, len(fs)), step):
        n += 1
        isl = find_island(load(fs[i]))
        if isl:
            hit += 1
            wl, wr, cx, h, w, r0, r1, cov = isl
            print('%4d  ISLAND x%d-%d cx%d  h%d w%d rows%d-%d cov%.2f'
                  % (i + 1, wl, wr, cx, h, w, r0, r1, cov))
        else:
            print('%4d  -' % (i + 1))
    print('---- %s: n=%d hit=%d (%.0f%%)' % (d, n, hit, 100.0 * hit / max(n, 1)))


if __name__ == '__main__':
    main()
