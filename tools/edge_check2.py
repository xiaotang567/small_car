import sys
import numpy as np
from PIL import Image

p = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\SHUIGUI\Documents\small_car\tools\ui_new.png'
im = Image.open(p).convert('RGB')
a = np.asarray(im).astype(np.int16)
H, W, _ = a.shape

R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
pureblack = (R == 0) & (G == 0) & (B == 0)
purewhite = (R == 255) & (G == 255) & (B == 255)
content = pureblack | purewhite

best = None
for y in range(H):
    xs = np.where(content[y])[0]
    if len(xs) < 100:
        continue
    x0_, x1_ = int(xs.min()), int(xs.max())
    span = x1_ - x0_
    if best is None or span > best[0]:
        best = (span, x0_, x1_, y)
span, x0, x1, ytop = best

colslice = content[:, x0:x1 + 1]
rowcov = colslice.mean(axis=1)
ys = np.where(rowcov > 0.5)[0]
y0, y1 = int(ys.min()), int(ys.max())

CW, CH = 188, 120
sx = (x1 - x0 + 1) / CW
sy = (y1 - y0 + 1) / CH
print('image area x %d..%d y %d..%d  (%.2f px/col, %.2f px/row)' % (x0, x1, y0, y1, sx, sy))

def row_runs(ya, yb):
    row = a[ya:yb, x0:x1 + 1]
    ww = (row[:, :, 0] > 200).mean(axis=0) > 0.5
    out, s = [], None
    for i, v in enumerate(ww):
        if v and s is None:
            s = i
        elif (not v) and s is not None:
            out.append((s / sx, (i - 1) / sx))
            s = None
    if s is not None:
        out.append((s / sx, (len(ww) - 1) / sx))
    return [(round(x, 1), round(y, 1)) for x, y in out if (y - x) > 3]

print('')
print('row   red_L   red_R   white_runs')
ok = tot = 0
for r in range(0, CH, 6):
    ya = int(y0 + r * sy)
    yb = max(ya + 1, int(y0 + (r + 1) * sy))
    row = a[ya:yb, x0:x1 + 1]
    rr = (row[:, :, 0] > 140) & (row[:, :, 1] < 110) & (row[:, :, 2] < 110)
    cols = np.where(rr.mean(axis=0) > 0.05)[0]
    if len(cols):
        rl, rR = round(cols.min() / sx, 1), round(cols.max() / sx, 1)
    else:
        rl = rR = None
    runs = row_runs(ya, yb)
    print('%3d  %6s  %6s  %s' % (r, rl, rR, runs[:4]))
    if runs and rl is not None:
        tot += 1
        if (runs[0][0] - 6) <= rl and rR <= (runs[-1][1] + 6):
            ok += 1

if tot:
    print('')
    print('red line inside track run: %d/%d' % (ok, tot))
