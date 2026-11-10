/*********************************************************************************************************************
* 文件名              state_f4.c
* 适用平台            STM32F407
* 说明                **行驶状态判定 + 该状态要用的控制量**（直道 / 弯道 / 没线）

  数据流（一句话）：
      相机测量（cam_f4.c）────────► 状态判定（这个文件）────────► 执行（servo_f4.c / pid_f4.c）
      cam_curved / cam_line_res     g_drv_state / g_drv_kp..    舵机 PID、速度环上限
      cam_track_ok / cam_rows_n     g_drv_steer_w / duty_max

  ⚠️ 这里**只做决策**，不碰硬件：
     · 不改相机、不改电机、不发舵机命令；
     · 改"弯道/直道各用什么参数"就只改这一个文件（或者用命令 `sp`/`sc` 改增益，本文件会自动跟着用）。

  判据回顾（详细注释在 cam_f4.c 的 cam_track_eval 里）：
     对整条中线做最小二乘直线拟合，最大残差 res > CAM_CURVE_RES(1.6px) ⇒ 弯道（cam_curved=1）。
********************************************************************************************************************/
#include "state_f4.h"
#include "ctrl_f4.h"      /* 舵机增益 g_steer_*、占空比上限 CTRL_SPEED_DUTY_MAX* */
#include "board_f4.h"     /* cam_curved / cam_track_ok / cam_rows_n / cam_line_res */

/*--------------------------------------------------------------------------------------------------------------------
 * 状态与输出量（默认值 = 直道那一套，开机第一帧之前也不会乱来）
 *------------------------------------------------------------------------------------------------------------------*/
volatile uint8_t  g_drv_state    = (uint8_t)DRV_STRAIGHT;
volatile uint32_t g_drv_cnt_lost = 0U;
volatile uint32_t g_drv_cnt_str  = 0U;
volatile uint32_t g_drv_cnt_cur  = 0U;

volatile float    g_drv_steer_w  = 0.20f;
volatile float    g_drv_kp       = 5.0f;
volatile float    g_drv_kd       = 6.0f;
volatile float    g_drv_ki       = 0.50f;
volatile int32_t  g_drv_duty_max = CTRL_SPEED_DUTY_MAX;
volatile float    g_drv_res      = 0.0f;

/* 直线/弯道的远近权重（和原来 servo_f4.c 里的常量保持一致）*/
#define DRV_W_STR   0.20f
#define DRV_W_CUR   0.80f

/*--------------------------------------------------------------------------------------------------------------------
 * 判状态 + 选控制量
 *   调用位置：ctrl_f4.c 的 ctrl_tick()（20ms 一拍），紧挨着 ctrl_steer_from_cam() 之前
 *------------------------------------------------------------------------------------------------------------------*/
void drv_state_update (void)
{
    uint8_t st;

    /* ① 判状态 —— 只用相机给的测量量，别在这里加"猜"的逻辑 */
    if ((0U == cam_track_ok) || (cam_rows_n < 8))
    {
        st = (uint8_t)DRV_LOST;
        g_drv_cnt_lost ++;
    }
    else if (0U != cam_curved)
    {
        st = (uint8_t)DRV_CURVE;
        g_drv_cnt_cur ++;
    }
    else
    {
        st = (uint8_t)DRV_STRAIGHT;
        g_drv_cnt_str ++;
    }

    g_drv_state = st;
    g_drv_res   = cam_line_res;

    /* ② 按状态选控制量 —— 直道/弯道各一套；没线时退回直道那套（别打到限位）*/
    switch (st)
    {
        case (uint8_t)DRV_CURVE:
            g_drv_steer_w  = DRV_W_CUR;
            g_drv_kp       = g_steer_kp_cur;
            g_drv_kd       = g_steer_kd_cur;
            g_drv_ki       = g_steer_ki_cur;          /* ★ 2209：弯道专用积分 */
            g_drv_duty_max = (int32_t)CTRL_SPEED_DUTY_MAX_CUR;   /* ★ 2211：弯里阻力大，上限放到 80% */
            break;

        case (uint8_t)DRV_STRAIGHT:
        default:
            g_drv_steer_w  = DRV_W_STR;
            g_drv_kp       = g_steer_kp;
            g_drv_kd       = g_steer_kd;
            g_drv_ki       = g_steer_ki;
            g_drv_duty_max = (int32_t)CTRL_SPEED_DUTY_MAX;       /* 直道 50% */
            break;
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * 状态名（遥测/OLED 打印用）
 *------------------------------------------------------------------------------------------------------------------*/
const char * drv_state_name (void)
{
    switch (g_drv_state)
    {
        case (uint8_t)DRV_LOST:  return "LOST";
        case (uint8_t)DRV_CURVE: return "CUR";
        default:                 return "STR";
    }
}
