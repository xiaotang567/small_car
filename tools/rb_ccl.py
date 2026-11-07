# -*- coding: utf-8 -*-
"""按"轨道形状"判环岛：把**黑色块**连通成一块，看它是不是被白整个包住。

思路（用户 2026-11-07 21:27 提示："按照轨道形状判断环岛"）：
    环岛的岛 = 一块**被白色赛道包住的黑块**；
    赛道外面的黑（背景、墙根、场地边）都会**连到画面边上**。
所以判据不是"有多大一条黑缝"，而是：
    ① 黑块不碰画面四边（被白整圈包住）
    ② 块够大：高 >=8 行、最宽 >=20px、面积 >=250 px
    ③ 形状圆一点的（宽/高 在 0.35~3 之间）

实现用的是"按行游程 + 上下行重叠合并"的连通块（固件里也能这么写，不用 22KB 的标签图）。

用法：
    python rb_ccl.py 1 1521 30
    python rb_ccl.py 1 1 1 -v      # 打印所有黑块的详情
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, 'frames_round')

TXW, CAMH = 188, 120
BLACK_MERGE = int(os.environ.get('BM', '2'))   # 黑游程间 <=N px 的白缝先并掉（把黑块的白色噪点填掉）
MIN_BLACK_RUN = int(os.environ.get('MBR', '3'))  # 黑游程窄于 N px 直接丢掉（细黑桥/噪点）

MIN_H, MIN_W, MIN_AREA = 8, 20, 250
ASPECT_LO, ASPECT_HI = 0.35, 3.0


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
        if out and a - out[-1][1] - 1 <= BLACK_MERGE:
            out[-1][1] = b
        else:
            out.append([a, b])
    # 窄于 MIN_BLACK_RUN 的黑游程 = 细黑桥/噪点，直接丢（这一步会把"岛"和外面的黑断开）
    return [t for t in out if t[1] - t[0] + 1 >= MIN_BLACK_RUN]


def load(path):
    im = Image.open(path).convert('L')
    px = im.load()
    return [[1 if px[x, y] > 127 else 0 for x in range(TXW)] for y in range(CAMH)]


def black_blobs(img):
    """返回 [(r0, r1, x0, x1, maxw, area, border), ...]"""
    comps = []
    prev = []            # [(a, b, comp_idx)]
    for r in range(CAMH):
        cur = []
        used = [False] * len(prev)
        for (a, b) in black_runs(img[r]):
            ci = None
            for i, (pa, pb, pci) in enumerate(prev):
                if used[i]:
                    continue
                if min(b, pb) - max(a, pa) >= 0:      # 有任何重叠 ⇒ 同一块
                    used[i] = True
                    ci = pci
                    break
            if ci is None:
                comps.append(dict(r0=r, r1=r, x0=a, x1=b, maxw=b - a + 1,
                                  area=b - a + 1, border=0))
                ci = len(comps) - 1
            else:
                c = comps[ci]
                c['r1'] = r
                c['x0'] = min(c['x0'], a)
                c['x1'] = max(c['x1'], b)
                c['maxw'] = max(c['maxw'], b - a + 1)
                c['area'] += b - a + 1
            if a <= 0 or b >= TXW - 1:
                comps[ci]['border'] = 1
            cur.append((a, b, ci))
        prev = cur
    return [(c['r0'], c['r1'], c['x0'], c['x1'], c['maxw'], c['area'], c['border'])
            for c in comps]


def find_island(img, verbose=False):
    best = None
    for (r0, r1, x0, x1, maxw, area, border) in black_blobs(img):
        h = r1 - r0 + 1
        w = maxw
        if border:
            if verbose:
                print('   border blob r%d-%d x%d-%d w%d a%d' % (r0, r1, x0, x1, w, area))
            continue
        if h < MIN_H or w < MIN_W or area < MIN_AREA:
            continue
        asp = float(w) / float(h)
        if asp < ASPECT_LO or asp > ASPECT_HI:
            continue
        if best is None or area > best[5]:
            best = (r0, r1, x0, x1, w, area)
    return best


def main():
    fs = sorted(glob.glob(os.path.join(FRAMES, '*.png')))
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    b = int(sys.argv[2]) if len(sys.argv) > 2 else len(fs)
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    verbose = '-v' in sys.argv
    hits = 0
    n = 0
    for i in range(a - 1, min(b, len(fs)), step):
        n += 1
        img = load(fs[i])
        if verbose:
            print('frame %d:' % (i + 1))
        isl = find_island(img, verbose)
        if isl:
            hits += 1
            r0, r1, x0, x1, w, area = isl
            print('%4d  ISLAND r%d-%d x%d-%d w%d area%d' % (i + 1, r0, r1, x0, x1, w, area))
        elif not verbose:
            print('%4d  -' % (i + 1))
    print('sampled=%d island=%d' % (n, hits))


if __name__ == '__main__':
    main()
