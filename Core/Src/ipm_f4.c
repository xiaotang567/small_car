/*********************************************************************************************************************
* 文件名              ipm_f4.c
* 说明                **逆透视（BEV）扫描** —— 直接在俯视图坐标上找赛道中线
*
*   流程（每帧一次，156 行 × 121 列 = 18876 次查表 + 少量判断）：
*     ① 从最近的一行往远处走，每行把 121 个像素按表取出来（只取 255）→ 得到若干白段
*     ② 第一行：取包含中心列的那一段（没有就取最宽的一段）
*     ③ 之后：取和上一行中心"重叠（±3 像素）"的段，取并集（跨过中间的黑洞）
*     ④ 断了就停（不猜、不外推）
*     ⑤ 近处行 / 远处行的中心减中心列 ⇒ 偏差，换算成"等效相机像素"
*
*   为什么用**并集**：赛道中间有黑缝（斑马线、环岛岛边）时一行会被切成两段，
*   两段都和上一行相连；只挑一段会让左右边界一起塌到黑缝边上。
********************************************************************************************************************/
#include "ipm_f4.h"

#include "ipm_lut.h"

/* 一行里最多认几段（够了，赛道一行不会碎成很多段）*/
#define IPM_MAX_RUNS    8U

/* ★★ 2232：**传感器原图的朝向补偿**
   用户指出：原图是"上下颠倒 + 左右相反"。
   查表是按仿真相机的朝向算出来的 ⇒ 要把坐标变换一下再取像素。
      bit0 = 左右镜像   bit1 = 上下翻转   (3 = 180°，当前默认)
   开机默认 3；万一装法不同，用命令 `br 0..3` 现场试，不用重烧。*/
volatile uint8_t g_ipm_rot = 3U;

static inline uint16_t ipm_xform (uint16_t idx)
{
    uint16_t u, v;

    if ((0U == g_ipm_rot) || (0xFFFFU == idx)) { return idx; }

    u = (uint16_t)(idx % IPM_SRC_W);
    v = (uint16_t)(idx / IPM_SRC_W);
    if (0U != (g_ipm_rot & 1U)) { u = (uint16_t)(IPM_SRC_W - 1U - u); }
    if (0U != (g_ipm_rot & 2U)) { v = (uint16_t)(IPM_SRC_H - 1U - v); }
    return (uint16_t)(v * IPM_SRC_W + u);
}

static uint8_t s_rs[IPM_MAX_RUNS];
static uint8_t s_re[IPM_MAX_RUNS];

static uint8_t ipm_row_runs (const uint8_t *img, uint8_t y)
{
    const uint16_t *lut = ipm_lut[y];
    uint8_t  n = 0U;
    int32_t  c, s = -1, e = -1;

    for (c = 0; c < (int32_t)IPM_BEV_COLS; c++)
    {
        uint16_t idx = lut[c];
        uint8_t  white = 0U;

        if (idx != 0xFFFFU)
        {
            idx = ipm_xform(idx);          /* ★ 2232：按固件里的图像朝向取像素 */
            white = (img[idx] == 255U) ? 1U : 0U;
        }

        if (0U != white)
        {
            if (s < 0) { s = c; }
            e = c;
        }
        else if (s >= 0)
        {
            if (n < IPM_MAX_RUNS) { s_rs[n] = (uint8_t)s; s_re[n] = (uint8_t)e; n ++; }
            s = -1;
        }
    }
    if (s >= 0)
    {
        if (n < IPM_MAX_RUNS) { s_rs[n] = (uint8_t)s; s_re[n] = (uint8_t)e; n ++; }
    }
    return n;
}

void ipm_bev_scan (const uint8_t *img, uint8_t thresh, ipm_scan_t *out)
{
    int32_t r, prev = -1;
    int32_t rows = 0;
    int32_t cen_near = -1, cen_far = -1;
    uint8_t started = 0U;

    (void)thresh;      /* 图已经二值化成 0/255 了 */

    out->ok = 0U;
    out->rows = 0;
    out->e_near = 0.0f;
    out->e_far = 0.0f;
    out->curved = 0U;
    out->e_m = 0.0f;

    for (r = 0; r < (int32_t)IPM_BEV_ROWS; r++)
    {
        uint8_t  n = ipm_row_runs(img, (uint8_t)r);
        int32_t  a = -1, b = -1, i;

        if (0U == n)
        {
            if (0U != started) { break; }        /* 远处断了 ⇒ 结束 */
            continue;
        }
        started = 1U;

        if (prev < 0)
        {
            /* 第一行：优先包含中心列的那段，否则取最宽 */
            int32_t best = -1;
            for (i = 0; i < (int32_t)n; i++)
            {
                if ((s_rs[i] <= IPM_COL_CENTER) && (s_re[i] >= IPM_COL_CENTER)) { a = s_rs[i]; b = s_re[i]; break; }
                if (best < 0 || (s_re[i] - s_rs[i]) > (s_re[best] - s_rs[best])) { best = i; }
            }
            if (a < 0 && best >= 0) { a = s_rs[best]; b = s_re[best]; }
        }
        else
        {
            /* 之后：和上一行中心重叠（±3）的段取并集 */
            for (i = 0; i < (int32_t)n; i++)
            {
                if ((s_rs[i] <= (uint8_t)(prev + 3)) && (s_re[i] >= (uint8_t)(prev - 3)))
                {
                    if (a < 0 || s_rs[i] < a) { a = s_rs[i]; }
                    if (b < 0 || s_re[i] > b) { b = s_re[i]; }
                }
            }
            if (a < 0) { break; }
        }

        if ((b - a) < 4) { continue; }           /* 太窄 = 噪声 */

        prev = (a + b) / 2;
        rows ++;
        if (r <= IPM_NEAR_ROW) { cen_near = prev; }
        if (cen_far < 0 && r >= IPM_FAR_ROW) { cen_far = prev; }
    }

    out->rows = rows;
    if (rows < IPM_MIN_ROWS || cen_near < 0) { return; }

    if (cen_far < 0) { cen_far = cen_near; }

    /* ★ 符号约定：目标在左 ⇒ e > 0（和固件原来的 cam_err_near 一致） */
    out->e_near = ((float)cen_near - IPM_COL_CENTER_F) * IPM_PX_PER_COL;
    out->e_far  = ((float)cen_far  - IPM_COL_CENTER_F) * IPM_PX_PER_COL;
    out->e_m    = out->e_near / 250.0f;
    out->curved = ((out->e_far - out->e_near) > IPM_CURVE_DEG ||
                   (out->e_far - out->e_near) < -IPM_CURVE_DEG) ? 1U : 0U;
    out->ok = 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 把俯视图画成一张 188×120 的图（给图传用）
//   行 = 前向距离（第 0 行 = 车前 5cm，第 119 行 = 前方 1.55m）
//   列 = 横向（第 93/94 列 = 车正前方那条线）
//   地形以上/视野外 = 0（黑）
// 开销：120×188 = 22560 次查表，纯内存操作，不到 1ms
//-------------------------------------------------------------------------------------------------------------------
void ipm_bev_render (const uint8_t *img, uint8_t *dst)
{
    int32_t r, c;

    for (r = 0; r < (int32_t)IPM_BEV_ROWS; r++)
    {
        const uint16_t *lut = ipm_lut[r];
        uint8_t        *row = &dst[r * (int32_t)IPM_BEV_COLS];

        for (c = 0; c < (int32_t)IPM_BEV_COLS; c++)
        {
            uint16_t idx = lut[c];

            if (idx != 0xFFFFU) { idx = ipm_xform(idx); }
            row[c] = ((idx != 0xFFFFU) && (img[idx] == 255U)) ? 255U : 0U;
        }
    }
}
