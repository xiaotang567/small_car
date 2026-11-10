/*********************************************************************************************************************
* 文件名              state_f4.h
* 适用平台            STM32F407
* 说明                **行驶状态判定 + 状态相关的控制量**（2026-11-10 从 servo_f4.c / pid_f4.c 里拆出来）

  为什么单独一个文件（用户要求："把判断以及控制行走状态的单独做一个文件"）：
    以前"这帧是直道还是弯道"这个判断散在三处：
      · cam_f4.c   cam_track_eval() 里算残差 ⇒ cam_curved（判据本身）
      · servo_f4.c 里按 cam_curved 选舵机增益和远近权重
      · pid_f4.c   里按 cam_curved 选速度环占空比上限
    ⇒ 想改"弯道怎么处理"要翻三个文件。现在统一到 state_f4.c：
      cam_f4.c  只负责算 `cam_curved` / `cam_line_res`（**测量**）
      state_f4.c 负责判状态、并把该状态要用的控制量（舵机 kp/ki/kd、远近权重、占空比上限）算好（**决策**）
      servo_f4.c / pid_f4.c 直接用 `g_drv_*`（**执行**）

  状态：
    DRV_LOST     = 没线（cam_track_ok=0 或有效行数 <8）—— 舵机保持、积分泄放
    DRV_STRAIGHT = 直道（中线拟合残差小）
    DRV_CURVE    = 弯道（残差 > CAM_CURVE_RES）
********************************************************************************************************************/
#ifndef STATE_F4_H
#define STATE_F4_H

#include <stdint.h>

/* 行驶状态 */
typedef enum
{
    DRV_LOST     = 0,      /* 没线 */
    DRV_STRAIGHT = 1,      /* 直道 */
    DRV_CURVE    = 2       /* 弯道 */
} drv_state_t;

extern volatile uint8_t  g_drv_state;       /* 当前状态（drv_state_t）*/
extern volatile uint32_t g_drv_cnt_lost;    /* 各状态出现过的帧数（诊断/调参用）*/
extern volatile uint32_t g_drv_cnt_str;
extern volatile uint32_t g_drv_cnt_cur;

extern volatile float    g_drv_steer_w;     /* 舵机远近误差融合权重：弯道 0.80 / 直道 0.20 */
extern volatile float    g_drv_kp;          /* 本状态该用的舵机 P */
extern volatile float    g_drv_kd;          /* 本状态该用的舵机 D */
extern volatile float    g_drv_ki;          /* 本状态该用的舵机 I */
extern volatile int32_t  g_drv_duty_max;    /* 本状态该用的速度环占空比上限 */
extern volatile float    g_drv_res;         /* 最近一帧的中线拟合残差（判弯道的原始量）*/

/* 每帧（或每拍）调一次：读相机测量量 → 判状态 → 算控制量 */
void         drv_state_update (void);

/* "LOST" / "STR" / "CUR"（遥测、OLED、日志用）*/
const char * drv_state_name   (void);

#endif /* STATE_F4_H */
