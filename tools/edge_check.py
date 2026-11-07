import sys
import numpy as np
from PIL import Image

p = r'C:\Users\SHUIGUI\Documents\small_car\tools\ui_t255.png'
im = Image.open(p).convert('RGB')
a = np.asarray(im).astype(np.int16)
H, W, _ = a.shape

R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
red = (R > 140) & (G < 110) & (B < 110)

# 1) 找图像区：纯黑(0,0,0) 与 纯白(255,255,255) 组成的大矩形
pureblack = (R == 0) & (G == 0) & (B == 0)
purewhite = (R == 255) & (G == 255) & (B == 255)
content = pureblack | purewhite

# 逐行统计 content 的连续跨度，取最宽的那一段（=图传区域）
best = None
for y in range(H):
    xs = np.where(content[y])[0]
    if len(xs) < 100:
        continue
    x0, x1 = xs.min(), xs.max()
    span = x1 - x0
    if best is None or span > best[0]:
        best = (span, x0, x1, y)
span, x0, x1, ytop = best

# 找纵向范围：在该 x 范围内，content 覆盖的行
colslice = content[:, x0:x1 + 1]
rowcov = colslice.mean(axis=1)
ys = np.where(rowcov > 0.5)[0]
y0, y1 = ys.min(), ys.max()

print('图像区: x %d..%d  y %d..%d   (%dx%d)' % (x0, x1, y0, y1, x1 - x0 + 1, y1 - y0 + 1))
CW, CH = 188, 120
sx = (x1 - x0 + 1) / CW
sy = (y1 - y0 + 1) / CH
print('每列 %.2f px, 每行 %.2f px' % (sx, sy))

# 2) 逐逻辑行：还原二值，取红点，比较
print('\nrow  left_red right_red   white_runs(起-止)          判定')
bad_l = bad_r = 0
for r in range(0, CH, 6):
    ys0 = int(y0 + r * sy)
    ys1 = int(y0 + (r + 1) * sy)
    row = a[ys0:ys1, x0:x1 + 1]
    rr = (row[:, :, 0] > 140) & (row[:, :, 1] < 110) & (row[:, :, 2] < 110)
    ww = row[:, :, 0] > 200      # 近白
    # 红点所在的逻辑列
    cols_red = np.where(rr.mean(axis=0) > 0.05)[0]
    left_red = right_red = None
    if len(cols_red):
        cl = [c / sx for c in cols_red]
        left_red, right_red = min(cl), max(cl)
    # 该行白像素的连续段（逻辑列）
    wcol = ww.mean(axis=0) > 0.5
    runs = []
    s = None
    for i, v in enumerate(wcol):
        if v and s is None:
            s = i
        elif (not v) and s is not None:
            runs.append((s / sx, (i - 1) / sx))
            s = None
    if s is not None:
        runs.append((s / sx, (len(wcol) - 1) / sx))
    runs = [(round(x, 1), round(y, 1)) for x, y in runs if (y - x) > 3]
    print('%3d  %8s %8s   %-28s' % (
        r,
        '%.1f' % left_red if left_red is not None else '-',
        '%.1f' % right_red if right_red is not None else '-',
        str(runs[:4])))
print('\n红点总行数:', int(red.sum()))
