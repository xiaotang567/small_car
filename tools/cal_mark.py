# -*- coding: utf-8 -*-
"""从"手标红圈"的图里取 4 个角（每个角画了很多圈 -> 取平均）

输入  tools/cal/xxx_x8.png   （放大 8 倍、带网格；用户在上面用红笔圈了 4 个角）
输出  tools/cal/mark_check.png   （把识别到的 4 个点画回去，给人确认）
      tools/cal/mark.json        （4 个角的图像坐标 u,v + x8 坐标）

用法： python tools\\cal_mark.py [图名（可省，默认最新 _x8.png）]
"""
import io
import json
import os
import sys
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, 'cal')
K = 8                       # _x8 的放大倍数
MERGE_PX = 120              # 两个红笔画之间小于这个距离就算同一个角


def red_mask(a):
    r = a[:, :, 0].astype(np.int16)
    g = a[:, :, 1].astype(np.int16)
    b = a[:, :, 2].astype(np.int16)
    return (r >= 110) & (r - g >= 45) & (r - b >= 45)


def components(mask):
    """8 邻域连通域（纯 numpy+deque，不依赖 scipy）"""
    h, w = mask.shape
    seen = np.zeros((h, w), dtype=bool)
    out = []
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if seen[y0, x0]:
            continue
        q = deque([(y0, x0)])
        seen[y0, x0] = True
        n = 0
        sx = sy = 0
        while q:
            y, x = q.popleft()
            n += 1
            sx += x
            sy += y
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        q.append((yy, xx))
        if n >= 6:
            out.append((n, float(sx) / n, float(sy) / n))
    return out


def merge(comps, dist):
    """把靠得很近的小笔画并成一个角（按红点总数加权平均）"""
    groups = []
    for n, cx, cy in sorted(comps, reverse=True):
        for g in groups:
            if (g['cx'] - cx) ** 2 + (g['cy'] - cy) ** 2 <= dist * dist:
                g['n'] += n
                g['sx'] += cx * n
                g['sy'] += cy * n
                g['cx'] = g['sx'] / g['n']
                g['cy'] = g['sy'] / g['n']
                break
        else:
            groups.append({'n': n, 'sx': cx * n, 'sy': cy * n, 'cx': cx, 'cy': cy})
    groups.sort(key=lambda g: -g['n'])
    return groups


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
        if not os.path.isabs(path):
            path = os.path.join(CAL, path)
    else:
        cand = [os.path.join(CAL, f) for f in os.listdir(CAL) if f.endswith('_x8.png')]
        path = max(cand, key=os.path.getmtime)

    im = Image.open(path).convert('RGB')
    a = np.asarray(im)
    m = red_mask(a)
    print('图：%s  %dx%d  红点 %d 个' % (os.path.basename(path), im.size[0], im.size[1], m.sum()))

    comps = components(m)
    groups = merge(comps, MERGE_PX)
    print('红笔画 %d 段 → 合并成 %d 个角' % (len(comps), len(groups)))
    groups = groups[:4]
    if len(groups) < 4:
        print('✗ 只找到 %d 个角，红圈没画够或者被别的东西冒充了' % len(groups))
        return 1

    pts = []
    for g in groups:
        cx, cy = g['cx'], g['cy']
        u = (cx + 0.5) / K - 0.5          # x8 像素中心 -> 图像像素坐标
        v = (cy + 0.5) / K - 0.5
        pts.append({'u': round(u, 2), 'v': round(v, 2), 'x8': [round(cx, 1), round(cy, 1)],
                    'n': g['n']})
        print('  角: u=%6.2f v=%6.2f   (x8 坐标 %.0f,%.0f, 红点 %d)' % (u, v, cx, cy, g['n']))

    # 排成 左上 / 右上 / 右下 / 左下（v 小的是远，u 小的是左）
    by_v = sorted(pts, key=lambda p: p['v'])
    far = sorted(by_v[:2], key=lambda p: p['u'])
    near = sorted(by_v[2:], key=lambda p: p['u'])
    order = [far[0], far[1], near[1], near[0]]

    d = ImageDraw.Draw(im)
    col = (0, 255, 0)
    for i, p in enumerate(order):
        x, y = p['x8']
        d.line([(x - 26, y), (x + 26, y)], fill=col, width=3)
        d.line([(x, y - 26), (x, y + 26)], fill=col, width=3)
        d.ellipse([x - 9, y - 9, x + 9, y + 9], outline=col, width=2)
        d.text((x + 14, y - 26), str(i + 1), fill=col)
    for i in range(4):
        x1, y1 = order[i]['x8']
        x2, y2 = order[(i + 1) % 4]['x8']
        d.line([(x1, y1), (x2, y2)], fill=(0, 200, 255), width=2)

    out_png = os.path.join(CAL, 'mark_check.png')
    im.save(out_png)
    js = {'src': os.path.basename(path),
          'size': [im.size[0] // K, im.size[1] // K],
          'order': ['far_left', 'far_right', 'near_right', 'near_left'],
          'pts': order}
    with io.open(os.path.join(CAL, 'mark.json'), 'w', encoding='utf-8') as f:
        json.dump(js, f, ensure_ascii=False, indent=2)

    print('\n四个角（图像坐标 u=列, v=行）：')
    for i, p in enumerate(order):
        print('  %d  %-10s u=%6.2f  v=%6.2f' % (i + 1, js['order'][i], p['u'], p['v']))
    print('\n近边宽 = %.1f px, 远边宽 = %.1f px, 近边行 %0.1f, 远边行 %.1f'
          % (order[2]['u'] - order[3]['u'], order[1]['u'] - order[0]['u'],
             order[2]['v'], order[0]['v']))
    print('已存: %s' % out_png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
