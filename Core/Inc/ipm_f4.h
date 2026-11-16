/*********************************************************************************************************************
* 文件名              ipm_f4.h
* 说明                **逆透视（Bird's-Eye View）扫描** —— 把 188×120 相机图当俯视图来巡线
*
*   视角表是离线生成的（tools/gen_ipm_lut.py -> Core/Src/ipm_lut.c），几何和仿真里
*   sim/ipm.py 完全一致：相机 188×120 / HFOV 60° / 前 0.20m 高 0.13m 低头 18°，
*   俯视覆盖 前向 0.05~1.60m、横向 ±0.60m、10mm/像素（156 行 × 121 列）。
*
*   ★ 为什么值得这么做：俯视图里赛道是**等宽**的（0.5m 赛道每行都约 50 像素），
*     "近处宽远处窄 / 弯道行宽变宽 / 弦中点把中线带偏"这些透视误差根本不存在。
*
*   ★ 用法（在 cam_f4.c 的 board_cam_analyze 里，二值化之后调）：
*       ipm_bev_scan(disp_img[0], 255, &rows, &e_near, &e_far, &curved);
*     返回 1 且 rows >= 8 时，用 e_near/e_far 覆盖 cam_err_near/cam_err_far
*     （单位已换算成"等效相机像素"，所以原来的 PID 增益不用改）。
*     返回 0 = 这一帧俯视图里没找到线 ⇒ **沿用透视那条链的结果**（安全兜底）。
********************************************************************************************************************/
#ifndef __IPM_F4_H
#define __IPM_F4_H

#include <stdint.h>

/* 俯视 → 等效相机像素 的换算：1 列 = 10mm，250 px/m ⇒ 1 列 = 2.5 等效像素 */
#define IPM_PX_PER_COL      2.5f
#define IPM_COL_CENTER      60          /* 俯视图中心列（y = 0，也就是车的位置）*/
#define IPM_NEAR_ROW        65          /* 前向 0.70m 的那一行 */
#define IPM_FAR_ROW         125         /* 前向 1.30m 的那一行 */
#define IPM_MIN_ROWS        8           /* 少于这么多行就不认这一帧 */
#define IPM_CURVE_DEG       40.0f       /* 远近误差差超过这么多等价像素就判弯道 */

typedef struct
{
    int32_t rows;        /* 这次扫到多少行赛道 */
    float   e_near;      /* 近处偏差（等效相机像素，右为正）*/
    float   e_far;       /* 远处偏差 */
    uint8_t curved;      /* 1 = 判为弯道 */
    uint8_t ok;          /* 1 = 这一帧俯视扫描可用 */
    float   e_m;         /* 近处偏差（米，正=车偏右）*/
} ipm_scan_t;

/* 扫一帧俯视图（img 是 188×120 的二值化图，255 = 赛道）*/
void ipm_bev_scan (const uint8_t *img, uint8_t thresh, ipm_scan_t *out);

#endif /* __IPM_F4_H */
