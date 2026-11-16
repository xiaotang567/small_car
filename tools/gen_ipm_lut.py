# -*- coding: utf-8 -*-
"""生成固件用的逆透视查找表 Core/Src/ipm_lut.c

表的内容：俯视图每个像素 (r,c) 对应相机 188×120 图像里的哪个像素 idx = v*188 + u
          （0xFFFF = 地平线以上/无效）

表就是仿真里 sim/ipm.py 那套几何算出来的，参数完全一致：
   相机 188×120 / HFOV 60° / 装在车头前 0.20m、高 0.13m / 低头 18°
   俯视 前向 0.05~1.60m、横向 ±0.60m、10mm/像素
"""
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sim'))

from ipm import IPM

OUT_C = os.path.join(HERE, '..', 'Core', 'Src', 'ipm_lut.c')
OUT_H = os.path.join(HERE, '..', 'Core', 'Inc', 'ipm_lut.h')


def main():
    # ★ 俯视图做成**和相机图一样的 120 行 × 188 列**，这样固件里可以直接顶上 disp_img，
    #   图传那边一个字节都不用改，上位机看到的就变成俯视图了。
    RES_X = 1.20 / 187.0
    RES_Y = 1.50 / 119.0
    ipm = IPM(bilinear=False, bev_x=(0.05, 0.05 + 119 * RES_Y),
              bev_y=187 * RES_X / 2.0, res=RES_X, res_y=RES_Y)
    nx, ny = ipm.nx, ipm.ny
    u = np.rint(ipm.map_u).astype(np.int32)
    v = np.rint(ipm.map_v).astype(np.int32)
    ok = ipm.valid & (u >= 0) & (u < 188) & (v >= 0) & (v < 120)
    idx = np.full((nx, ny), 0xFFFF, dtype=np.uint16)
    idx[ok] = (v[ok] * 188 + u[ok]).astype(np.uint16)

    lines = []
    lines.append('/*******************************************************************************')
    lines.append('* 文件名   ipm_lut.c   （**自动生成，别手改** -> tools/gen_ipm_lut.py）')
    lines.append('* 说明     逆透视查找表：俯视图 (r,c) -> 相机 188x120 像素 idx = v*188+u')
    lines.append('*          0xFFFF = 地平线以上/视野外（无效）')
    lines.append('*          几何：相机 188x120 / HFOV 60 / 前 0.20m 高 0.13m 低头 18')
    lines.append('*                俯视 0.05~1.60m x +-0.60m / 10mm 每像素')
    lines.append('*          %d 行 x %d 列，%d 字节 flash' % (nx, ny, nx * ny * 2))
    lines.append('*******************************************************************************/')
    lines.append('#include "ipm_lut.h"')
    lines.append('')
    lines.append('const uint16_t ipm_lut[IPM_BEV_ROWS][IPM_BEV_COLS] = {')
    for r in range(nx):
        row = ','.join('%u' % idx[r, c] for c in range(ny))
        lines.append('  {%s},' % row)
    lines.append('};')
    lines.append('')

    with io.open(OUT_C, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))

    h = ['#ifndef __IPM_LUT_H', '#define __IPM_LUT_H', '',
         '#include <stdint.h>', '',
         '#define IPM_BEV_ROWS   %d' % nx,
         '#define IPM_BEV_COLS   %d' % ny,
         '#define IPM_SRC_W      188',
         '#define IPM_SRC_H      120', '',
         'extern const uint16_t ipm_lut[IPM_BEV_ROWS][IPM_BEV_COLS];', '',
         '#endif', '']
    with io.open(OUT_H, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(h))

    print('生成 %s' % os.path.normpath(OUT_C))
    print('  俯视 %d x %d，有效像素 %.1f%%，flash %d 字节'
          % (nx, ny, 100.0 * (idx != 0xFFFF).mean(), nx * ny * 2))


if __name__ == '__main__':
    main()
