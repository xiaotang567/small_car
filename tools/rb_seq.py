# -*- coding: utf-8 -*-
"""环岛序列分析：对一串帧打印"紧凑几何摘要"，看环岛从远处到跟前怎么变化。

每帧一行：
  y=行号 : L-R(w)  左边界-右边界(宽度)，'-' = 该行没有白
  G  = 该行内部最大黑缝宽度（>=6 才算"有洞"）
最后汇总：判"洞"的行数、最大洞宽、整幅宽度

用法：
    python rb_seq.py                # 从 1 到最后一帧，每 40 帧取一张
    python rb_seq.py 1200 1600 20   # start end step
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, 'frames_round')
ROWS = (112, 100, 88, 76, 64, 52, 40, 28)
GAP_MERGE = 4      # 与固件 LF_GAP_MERGE 一致
MIN_RUN = 10       # 与固件 LF_MIN_RUN 一致


def row_runs(row, gap_merge=GAP_MERGE):
    n = []
    s = None
    e = -1
    for c, v in enumerate(row):
        if v:
            if s is None:
                s = c
            e = c
        elif s is not None:
            n.append([s, e])
            s = None
    if s is not None:
        n.append([s, e])
    # 合并小缝
    out = []
    for a, b in n:
        if out and a - out[-1][1] - 1 <= gap_merge:
            out[-1][1] = b
        else:
            out.append([a, b])
    return [t for t in out if t[1] - t[0] + 1 >= MIN_RUN]


def holes(row):
    """白色整体范围内的黑缝：返回 [(a,b),...]，只算"被白夹住"的洞"""
    r = row_runs(row)
    out = []
    for i in range(len(r) - 1):
        a = r[i][1] + 1
        b = r[i + 1][0] - 1
        if b - a + 1 >= 6:
            out.append((a, b))
    return out, r


def summarize(path):
    im = Image.open(path).convert('L')
    px = im.load()
    w, h = im.size
    cols = ['%3d' % y for y in ROWS]
    parts = []
    nhole = 0
    maxhole = 0
    for y in ROWS:
        row = [1 if px[x, y] > 127 else 0 for x in range(w)]
        hs, rr = holes(row)
        if not rr:
            parts.append('   -   ')
            continue
        L = rr[0][0]
        R = rr[-1][1]
        if hs:
            nhole += 1
            maxhole = max(maxhole, max(b - a + 1 for a, b in hs))
            parts.append('%3d-%3d*%2d' % (L, R, max(b - a + 1 for a, b in hs)))
        else:
            parts.append('%3d-%3d  ' % (L, R))
    return parts, nhole, maxhole


def main():
    fs = sorted(glob.glob(os.path.join(FRAMES, '*.png')))
    if not fs:
        print('no frames')
        return
    a = int(sys.argv[1]) - 1 if len(sys.argv) > 1 else 0
    b = int(sys.argv[2]) if len(sys.argv) > 2 else len(fs)
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 40
    print('rows  : ' + ' '.join('%3d' % y for y in ROWS))
    for i in range(a, min(b, len(fs)), step):
        parts, nh, mh = summarize(fs[i])
        print('%4d  ' % (i + 1) + ' '.join(parts) + '   hole_rows=%d max=%d' % (nh, mh))


if __name__ == '__main__':
    main()
