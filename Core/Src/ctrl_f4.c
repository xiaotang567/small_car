/*********************************************************************************************************************
* 文件名              ctrl_f4.c
* 适用平台            STM32F407
* 说明               **只保留启动/初始化与统一控制节拍**
*                    （2026-11-07 按用户要求拆分：电机→motor_f4.c、编码器→encoder_f4.c、
*                      舵机→servo_f4.c、PID→pid_f4.c、保护→protect_f4.c、
*                      OLED→oled_f4.c、按键→io_f4.c、IMU/yaw→imu_f4.c）
********************************************************************************************************************/

#include "ctrl_f4.h"
#include <stdio.h>
#include <string.h>

//===================================================★ 微秒延时（DWT）===================================================
void ctrl_dwt_init (void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void ctrl_delay_us (uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);

    while ((DWT->CYCCNT - start) < ticks) { }
}

void ctrl_all_init (void)
{
    ctrl_dwt_init();

    ctrl_io_init();          /* 蜂鸣器/按键最先，上电不叫 */
    ctrl_motor_init();       /* 电机 PWM=0、DIR=0，不会突然转 */
    ctrl_encoder_init();
    ctrl_imu_init();

    /* OLED：清屏 + **立刻画状态页**（约 40ms，只在开机做一次）
       ⚠️ 必须在这里画，不能等主循环：`ctrl_all_init()` 之后紧接着就是
          `wifi_f4_init()`，它最长会阻塞几十秒，那段时间主循环根本不在转。 */
    ctrl_oled_init();
    ctrl_oled_clear();
    ctrl_oled_first_page();
    ctrl_oled_boot(1U);      /* ★ 开机阶段 1：OLED 已点亮 */

    /* ★ 舵机放最后：它上电要 ~1 秒才肯说话，`ctrl_servo_init()` 里会等 1 秒再扫 ID
       ⇒ 先把屏幕点亮，别让这一秒黑着。 */
    ctrl_servo_init();
    ctrl_oled_boot(2U);      /* ★ 开机阶段 2：车模控制全部初始化完成 */

    ctrl_speed_init();
}

/* ★ 主循环每轮调一次，内部 20ms 节流；各模块的节拍都在自己的文件里 */
void board_ctrl_tick (void)
{
    static uint32_t t_last     = 0U;
    static uint32_t t_last_cyc = 0U;
    uint32_t now = HAL_GetTick();
    uint32_t dt_us;

    if ((now - t_last) < CTRL_PERIOD_MS) { return; }

    {
        uint32_t cyc = DWT->CYCCNT;

        dt_us      = (uint32_t)((cyc - t_last_cyc) / (SystemCoreClock / 1000000UL));
        t_last_cyc = cyc;
        t_last     = now;
    }

    if (0U == dt_us)      { dt_us = (uint32_t)CTRL_PERIOD_MS * 1000U; }
    if (dt_us > 200000UL) { dt_us = 200000UL; }

    ctrl_encoder_tick(dt_us);    /* 1) 编码器：增量 + 用真实 µs 间隔算转速 */
    ctrl_speed_tick();           /* 2) 速度环 */
    ctrl_protect_tick();         /* 3) 堵转保护 + 按键重启 */
    ctrl_imu_tick();             /* 4) IMU + yaw */
    ctrl_servo_tick();           /* 5) 舵机（同步轮询） */
    ctrl_steer_from_cam();       /* 5.5) ★ 2174 摄像头巡线 → 舵机目标位置 */
    ctrl_oled_status();          /* 6) OLED 状态页 */
}
