# -*- coding: utf-8 -*-
"""复刻固件 rb_detect() 的判据，在存下来的帧上跑一遍，看"岛/开阔区"是怎么判的。

和 Core/Src/cam_f4.c 里的 rb_detect() 一一对应（改一边要改另一边）：
    岛   ：行 26..102 里"白-黑-白"，黑缝 >=14px，两侧各 >=8px 白
    开阔 ：行 74..114 里白宽 >=1.45*wn，或白铺到画面两边，或贴边且到另一边 >=1.35*wn
    输出 ：每帧一行  hole=宽@行  open=行数  maxw=近处最大宽  wn=学习到的正常宽

用法：
    python rb_diag.py 200 1500 100
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, 'frames_round')

TXW, CAMH = 188, 120
GAP_MERGE, MIN_RUN, MAX_RUNS = 4, 10, 12
HOLE_MIN, SIDE_MIN = 14, 8
ROW_LO, ROW_HI = 26, 102
OPEN_K, OPEN_EDGE_K, OPEN_ROWS = 1.45, 1.35, 10


def runs_of(row):
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


def load(path):
    im = Image.open(path).convert('L')
    px = im.load()
    return [[1 if px[x, y] > 127 else 0 for x in range(TXW)] for y in range(CAMH)]


def rb_detect(img, wn):
    best, b_row = 0, 0
    for r in range(ROW_LO, ROW_HI + 1):
        rr = runs_of(img[r])
        gp = 0
        for j in range(len(rr) - 1):
            wl = rr[j][1] - rr[j][0] + 1
            wr = rr[j + 1][1] - rr[j + 1][0] + 1
            a, b = rr[j][1] + 1, rr[j + 1][0] - 1
            gw = b - a + 1
            if wl < SIDE_MIN or wr < SIDE_MIN or gw < HOLE_MIN:
                continue
            if gw > gp:
                gp = gw
        if gp > best:
            best, b_row = gp, r
    open_rows = 0
    maxw = 0
    for r in range(74, 115):
        rr = runs_of(img[r])
        if not rr:
            continue
        l, rt = rr[0][0], rr[-1][1]
        if rt <= l:
            continue
        w = rt - l
        maxw = max(maxw, w)
        clip_l = 1 if l <= 2 else 0
        clip_r = 1 if rt >= TXW - 2 else 0
        if clip_l and clip_r:
            open_rows += 1
        elif w >= wn * OPEN_K:
            open_rows += 1
        elif clip_r and (TXW - l) >= wn * OPEN_EDGE_K:
            open_rows += 1
        elif clip_l and rt >= wn * OPEN_EDGE_K:
            open_rows += 1
    return best, b_row, open_rows, maxw


def main():
    fs = sorted(glob.glob(os.path.join(FRAMES, '*.png')))
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    b = int(sys.argv[2]) if len(sys.argv) > 2 else len(fs)
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 100
    wn = 120.0
    print('%-6s %-5s %-6s %-5s %-5s' % ('frame', 'hole', 'row', 'open', 'maxw'))
    for i in range(a - 1, min(b, len(fs)), step):
        img = load(fs[i])
        hole, hrow, orows, maxw = rb_detect(img, wn)
        flag = ''
        if hole >= HOLE_MIN:
            flag += ' ISLAND'
        if orows >= OPEN_ROWS:
            flag += ' OPEN'
        print('%-6d %-5d %-6d %-5d %-5d%s' % (i + 1, hole, hrow, orows, maxw, flag))


if __name__ == '__main__':
    main()
