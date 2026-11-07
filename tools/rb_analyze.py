# -*- coding: utf-8 -*-
"""环岛调试：把存下来的二值帧打印成 ASCII / 逐行白色游程，方便离线看几何。

用法：
    python rb_analyze.py                 # 最新一帧
    python rb_analyze.py f_0120.png      # 指定帧
    python rb_analyze.py 0120 --runs     # 打印每行白色游程
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, 'frames_round')


def load(name=None):
    if not name:
        fs = sorted(glob.glob(os.path.join(FRAMES, '*.png')))
        name = fs[-1]
    elif not os.path.isabs(name):
        name = os.path.join(FRAMES, name)
        if not os.path.exists(name):
            name = os.path.join(FRAMES, 'f_%04d.png' % int(sys.argv[1]))
    im = Image.open(name).convert('L')
    return name, im


def rows_of(im):
    w, h = im.size
    px = im.load()
    out = []
    for y in range(h):
        out.append([1 if px[x, y] > 127 else 0 for x in range(w)])
    return out


def runs(row):
    r = []
    s = None
    for x, v in enumerate(row):
        if v and s is None:
            s = x
        elif not v and s is not None:
            r.append((s, x - 1))
            s = None
    if s is not None:
        r.append((s, len(row) - 1))
    return r


def ascii_art(rows, sx=2):
    w = len(rows[0])
    lines = []
    for y in range(0, len(rows), 1):
        line = []
        for x in range(0, w, sx):
            seg = rows[y][x:x + sx]
            line.append('#' if sum(seg) * 2 >= len(seg) else '.')
        lines.append('%3d %s' % (y, ''.join(line)))
    return '\n'.join(lines)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    name, im = load(args[0] if args else None)
    rows = rows_of(im)
    print('frame:', os.path.basename(name), im.size)
    print(ascii_art(rows))
    print('     ' + ''.join(str((x // 2) % 10) for x in range(0, im.size[0], 2)))
    if '--runs' in sys.argv:
        print()
        for y in range(im.size[1]):
            rr = runs(rows[y])
            print('%3d n=%d %s' % (y, len(rr), ' '.join('%d-%d' % t for t in rr)))


if __name__ == '__main__':
    main()
