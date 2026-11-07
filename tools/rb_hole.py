# -*- coding: utf-8 -*-
"""更严格的"岛"判据：把每行的黑洞按行**连通成块**，再看整块的高度/宽度。
（单行一条黑缝不算岛 —— 一条横杠、一个反光点都能骗过老判据，这正是环岛逻辑
 误触发、把正常赛道也当成环岛的原因。）

判据（和固件里准备改成的一致）：
  · 每行的洞：白-黑-白，两侧白各 >=8px，且**两侧都没贴画面边**（贴边的是背景白）
  · 垂直连通：相邻两行的洞在 x 方向重叠 >=50% ⇒ 同一个块
  · 是岛：块高 >=6 行、最大宽 >=20px，且洞**上下都被白封住**（不是一直通到画面顶/底）

用法：
    python rb_hole.py 1 1521 60
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, 'frames_round')

TXW, CAMH = 188, 120
GAP_MERGE, MIN_RUN, MAX_RUNS = 4, 10, 12
ROW_LO, ROW_HI = 26, 102
HOLE_MIN, SIDE_MIN, EXT_MIN, WID_MIN = 14, 8, 6, 20


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


def row_holes(img, r):
    rr = runs_of(img[r])
    hs = []
    for j in range(len(rr) - 1):
        lw = rr[j][1] - rr[j][0] + 1
        rw = rr[j + 1][1] - rr[j + 1][0] + 1
        if lw < SIDE_MIN or rw < SIDE_MIN:
            continue
        if rr[j][0] <= 2 or rr[j + 1][1] >= TXW - 2:      # 两侧贴画面边 ⇒ 背景白，不算洞
            continue
        a, b = rr[j][1] + 1, rr[j + 1][0] - 1
        if b - a + 1 < HOLE_MIN:
            continue
        hs.append((a, b))
    return hs


def islands(img):
    """返回 (块高最大, 最大宽, 起始行, 结束行)"""
    comps = []            # [r0, r1, maxw]
    prev = []             # 上一行的洞：[(a, b, comp_idx)]
    for r in range(ROW_LO, ROW_HI + 1):
        cur = row_holes(img, r)
        used = [False] * len(cur)
        new_prev = []
        for (a, b, ci) in prev:
            best, bi = 0, -1
            for i, (c, d) in enumerate(cur):
                if used[i]:
                    continue
                ov = min(b, d) - max(a, c) + 1
                if ov > best:
                    best, bi = ov, i
            if bi >= 0 and best * 2 >= (b - a + 1):
                used[bi] = True
                comps[ci][1] = r
                comps[ci][2] = max(comps[ci][2], cur[bi][1] - cur[bi][0] + 1)
                new_prev.append((cur[bi][0], cur[bi][1], ci))
        for i, (c, d) in enumerate(cur):
            if not used[i]:
                comps.append([r, r, d - c + 1])
                new_prev.append((c, d, len(comps) - 1))
        prev = new_prev
    if not comps:
        return 0, 0, 0, 0
    best = max(comps, key=lambda t: (t[1] - t[0] + 1, t[2]))
    return best[1] - best[0] + 1, best[2], best[0], best[1]


def load(path):
    im = Image.open(path).convert('L')
    px = im.load()
    return [[1 if px[x, y] > 127 else 0 for x in range(TXW)] for y in range(CAMH)]


def main():
    fs = sorted(glob.glob(os.path.join(FRAMES, '*.png')))
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    b = int(sys.argv[2]) if len(sys.argv) > 2 else len(fs)
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 60
    print('%-6s %-5s %-5s %-6s %-6s' % ('frame', 'hgt', 'wid', 'r0', 'r1'))
    hits = 0
    for i in range(a - 1, min(b, len(fs)), step):
        img = load(fs[i])
        h, w, r0, r1 = islands(img)
        ok = (h >= EXT_MIN and w >= WID_MIN)
        if ok:
            hits += 1
        print('%-6d %-5d %-5d %-6d %-6d%s' % (i + 1, h, w, r0, r1,
                                              '  ISLAND' if ok else ''))
    print('sampled=%d island=%d' % (len(range(a - 1, min(b, len(fs)), step)), hits))


if __name__ == '__main__':
    main()
