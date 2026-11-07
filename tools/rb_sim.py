# -*- coding: utf-8 -*-
"""把固件里的巡线/拟合/打舵逻辑在 PC 上复刻一份，用来离线试参数。

对应关系（改代码时两边一起改）：
    lf_row_runs    <-> Core/Src/cam_f4.c  lf_row_runs()
    trace          <-> cam_f4.c           lf_trace_track()
    row_center     <-> cam_f4.c           lf_row_center()
    track_eval     <-> cam_f4.c           cam_track_eval()
    steer          <-> servo_f4.c         ctrl_steer_from_cam()

用法：
    python rb_sim.py                # 最新一帧
    python rb_sim.py f_2300.png -v  # 指定帧 + 逐行打印
"""
import os
import sys
import glob

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, 'frames_round')

CAM_W, CAM_H = 188, 120
TXW = 188
CX = TXW // 2

LF_GAP_MERGE = 4
LF_MIN_RUN = 10
LF_MAX_RUNS = 12

STEER_R_NEAR = 112
STEER_R_AHEAD = 75
STEER_W_STR = 0.20
STEER_W_CUR = 0.80
STEER_E_LIMIT = 90.0


def row_runs(row):
    runs = []
    s = None
    e = -1
    for c in range(TXW):
        if row[c] == 255:
            if s is None:
                s = c
            e = c
        elif s is not None:
            runs.append([s, e])
            s = None
    if s is not None:
        runs.append([s, e])
    out = []
    for a, b in runs:
        if out and a - out[-1][1] - 1 <= LF_GAP_MERGE:
            out[-1][1] = b
        elif len(out) < LF_MAX_RUNS:
            out.append([a, b])
    return out


class Track(object):
    def __init__(self):
        self.w_est = 120.0
        self.bound_l = [255] * CAM_H
        self.bound_r = [255] * CAM_H
        self.center = [255] * CAM_H
        self.pl = [255] * CAM_H
        self.pr = [255] * CAM_H
        self.pok = False

    # ---------------- 追踪 ----------------
    def trace(self, img):
        self.bound_l = [255] * CAM_H
        self.bound_r = [255] * CAM_H
        r0 = CAM_H - 5
        row = img[r0]
        l = rg = -1
        for c in range(TXW):
            if row[c] == 255:
                l = c
                break
        for c in range(TXW - 1, -1, -1):
            if row[c] == 255:
                rg = c
                break
        if l < 0 or rg < 0 or rg <= l:
            return False
        self.bound_l[r0] = l
        self.bound_r[r0] = rg
        rows = [(r0, l, rg)]
        r = r0 - 1
        while r >= 0:
            runs = row_runs(img[r])
            ul = ur = -1
            for a, b in runs:
                ov = min(b, rg) - max(a, l) + 1
                if ov <= 0:
                    continue
                if ul < 0 or a < ul:
                    ul = a
                if ur < 0 or b > ur:
                    ur = b
            if ul < 0:
                break
            l, rg = ul, ur
            self.bound_l[r] = l
            self.bound_r[r] = rg
            rows.append((r, l, rg))
            r -= 1
        self.rows = rows
        return True

    def temporal(self):
        if not self.pok:
            self.pl = list(self.bound_l)
            self.pr = list(self.bound_r)
            self.pok = True
            return
        for r in range(CAM_H):
            l = self.bound_l[r]
            rr = self.bound_r[r]
            if l != 255 and self.pl[r] != 255 and abs(l - self.pl[r]) <= 4:
                l = (l * 3 + self.pl[r]) // 4
            if rr != 255 and self.pr[r] != 255 and abs(rr - self.pr[r]) <= 4:
                rr = (rr * 3 + self.pr[r]) // 4
            self.bound_l[r] = l
            self.bound_r[r] = rr
            self.pl[r] = l
            self.pr[r] = rr

    # ---------------- 行中心 ----------------
    def row_center(self, r, s_pl, s_pr):
        l = self.bound_l[r] if self.bound_l[r] != 255 else -1
        rr = self.bound_r[r] if self.bound_r[r] != 255 else -1
        lok = 1 if (l >= 0 and l >= 6) else 0
        rok = 1 if (rr >= 0 and rr <= TXW - 6) else 0
        out = 255
        if lok and rok:
            w = float(rr - l)
            if self.w_est > 8.0 and w > self.w_est * 1.6:
                dl = abs(l - s_pl) if s_pl >= 0 else 9999
                dr = abs(rr - s_pr) if s_pr >= 0 else 9999
                if dl <= dr:
                    rr = l + int(self.w_est)
                else:
                    l = rr - int(self.w_est)
                if rr > l + 4:
                    out = (l + rr) // 2
            else:
                if (w > 20.0 and w < TXW - 4.0 and
                        (w > self.w_est * 0.6) and (w < self.w_est * 1.4)):
                    self.w_est = self.w_est * 0.9 + w * 0.1
                out = (l + rr) // 2
        elif lok and self.w_est > 8.0:
            out = min(l + int(self.w_est * 0.5), TXW - 2)
        elif rok and self.w_est > 8.0:
            out = max(rr - int(self.w_est * 0.5), 1)
        return out, l, rr

    # ---------------- 拟合 ----------------
    def eval(self):
        self.center = [255] * CAM_H
        n = 0
        r_lo = r_hi = -1
        sx = sy = sxx = sxy = 0.0
        s_pl = s_pr = -1
        rows = []
        for r in range(CAM_H):
            cen, l, rr = self.row_center(r, s_pl, s_pr)
            s_pl, s_pr = l, rr
            self.center[r] = cen
            rows.append((r, cen, l, rr))
            if cen != 255:
                sx += r
                sy += cen
                sxx += r * r
                sxy += r * cen
                n += 1
                if r_lo < 0:
                    r_lo = r
                r_hi = r
        self.fit_rows = rows
        self.n = n
        if n < 8 or r_lo == r_hi:
            return None
        den = n * sxx - sx * sx
        if -0.5 < den < 0.5:
            return None
        k = (n * sxy - sx * sy) / den
        b = (sy - k * sx) / n
        res = 0.0
        for r in range(r_lo, r_hi + 1):
            if self.center[r] != 255:
                d = abs(k * r + b - self.center[r])
                res = max(res, d)
        en = int(k * r_hi + b - CX)
        ef = int(k * r_lo + b - CX)
        # 近处斜率
        r0 = max(r_hi - 20, r_lo)
        nx = ny = nxx = nxy = 0.0
        nn = 0
        for r in range(r0, r_hi + 1):
            if self.center[r] != 255:
                c = self.center[r]
                nx += r
                ny += c
                nxx += r * r
                nxy += r * c
                nn += 1
        nd = nn * nxx - nx * nx
        if nn >= 6 and abs(nd) > 0.5:
            kn = (nn * nxy - nx * ny) / nd
        else:
            kn = k
        return dict(k=k, b=b, res=res, en=en, ef=ef, kn=kn,
                    r_lo=r_lo, r_hi=r_hi, n=n,
                    curved=1 if res > 1.6 else 0)

    def steer(self, ev):
        e_near = ev['k'] * STEER_R_NEAR + ev['b'] - CX
        e_far = ev['k'] * STEER_R_AHEAD + ev['b'] - CX
        w = STEER_W_CUR if ev['curved'] else STEER_W_STR
        e = e_near * (1 - w) + e_far * w
        e = max(-STEER_E_LIMIT, min(STEER_E_LIMIT, e))
        return e_near, e_far, e


def load(name=None):
    fs = sorted(glob.glob(os.path.join(FRAMES, '*.png')))
    if not name:
        name = fs[-1]
    elif not os.path.isabs(name):
        name = os.path.join(FRAMES, name)
    im = Image.open(name).convert('L')
    px = im.load()
    img = [[255 if px[x, y] > 127 else 0 for x in range(CAM_W)]
           for y in range(CAM_H)]
    return name, img


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    name, img = load(args[0] if args else None)
    t = Track()
    if not t.trace(img):
        print('trace failed')
        return
    t.temporal()
    ev = t.eval()
    print('frame:', os.path.basename(name))
    if not ev:
        print('eval failed; n=%d' % t.n)
        return
    print('rows=%d r_lo=%d r_hi=%d w_est=%.1f k=%.3f b=%.1f res=%.1f curved=%d'
          % (ev['n'], ev['r_lo'], ev['r_hi'], t.w_est, ev['k'], ev['b'],
             ev['res'], ev['curved']))
    print('en=%+d ef=%+d kn=%.3f' % (ev['en'], ev['ef'], ev['kn']))
    e_near, e_far, e = t.steer(ev)
    print('steer: e_near=%.1f e_far=%.1f e=%.1f' % (e_near, e_far, e))
    if '-v' in sys.argv:
        print('%-4s %-6s %-8s %-8s' % ('row', 'center', 'L', 'R'))
        for r, cen, l, rr in t.fit_rows:
            print('%-4d %-6s %-8s %-8s' % (r,
                  '-' if cen == 255 else cen,
                  '-' if l < 0 else l,
                  '-' if rr < 0 else rr))


if __name__ == '__main__':
    main()
