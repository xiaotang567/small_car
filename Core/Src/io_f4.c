/*********************************************************************************************************************
* 文件名              io_f4.c
* 适用平台            STM32F407
* 说明                按键（PC2/PC3，高电平有效）与蜂鸣器（PC1）
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"


//===================================================★ 按键 / 蜂鸣器===================================================
void ctrl_io_init (void)
{
    GPIO_InitTypeDef gi;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_2 | GPIO_PIN_3;         /* 按键：**高电平有效（接 3.3V）**
                                                       ★ 2026-11-07（FW=2122）：用户确认按键按下去是 3.3V，
                                                         所以配**下拉**：松开=0，按下=1。
                                                         （原来按"接 GND/上拉"写，实测 k1 恒为 0，
                                                           正好说明板上就是下拉、按键给高。）*/
    gi.Mode      = GPIO_MODE_INPUT;
    gi.Pull      = GPIO_PULLDOWN;
    gi.Speed     = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = 0;
    HAL_GPIO_Init(GPIOC, &gi);

    gi.Pin  = GPIO_PIN_1;                           /* 蜂鸣器 */
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gi);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

/* ★ 2026-11-07（FW=2122）：按键**高电平有效**（按下 = 3.3V）⇒ 返回 1 = 按下 */
uint8_t ctrl_key (uint8_t idx)
{
    if (1U == idx) { return (0U != (GPIOC->IDR & GPIO_PIN_2)) ? 1U : 0U; }
    if (2U == idx) { return (0U != (GPIOC->IDR & GPIO_PIN_3)) ? 1U : 0U; }
    return 0U;
}

void ctrl_buzzer (uint8_t on)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (0U != on) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//===================================================★ PC3 = **直接启动按钮** ===================================================
//   用户要求："PC3 改为直接启动的按钮，我不想每次都要连上电脑才能跑"
//   行为（完全脱机，不需要上位机）：
//     · 按一下 PC3 ⇒ 闭环启动，目标转速 = CTRL_RUN_RPM（100 RPM），蜂鸣一声
//     · 再按一下 ⇒ 停车（目标 0 + 占空比 0），蜂鸣一声
//     · 如果当前处于"堵转停机"，按 PC3 先清堵转再启动（等于原来的重启功能）
//   按键消抖：连续两次采样间距 CTRL_PERIOD_MS，要求电平稳定 ≥2 拍（40ms）才认；上升沿视为按下。
//   ⚠️ PC2 保持原来的"堵转后重启"用途，没动。
//------------------------------------------------------------------------------------------------------------------
void ctrl_key_tick (void)
{
    static uint8_t  s_run    = 0U;      /* 1 = 按键启动的巡航状态 */
    static uint8_t  s_prev   = 0U;      /* 上一拍 PC3 电平 */
    static uint8_t  s_stable = 0U;      /* 稳定计数 */
    static uint8_t  s_beep   = 0U;      /* 蜂鸣剩余拍数 */

    uint8_t k = ctrl_key(2U);           /* PC3 */

    if (k != s_prev)
    {
        s_prev   = k;
        s_stable = 0U;
    }
    else if (s_stable < 2U)
    {
        s_stable ++;
    }

    /* 确认按下（上升沿 + 稳定）*/
    if ((1U == k) && (2U == s_stable))
    {
        if (0U == s_run)
        {
            ctrl_motor_clear_stall();           /* 堵转停机状态下也能一键起来 */
            ctrl_open_loop(0U);                 /* 闭环 */
            ctrl_speed_set_target((int16_t)CTRL_RUN_RPM);
            s_run = 1U;
        }
        else
        {
            ctrl_speed_set_target(0);
            ctrl_motor_set(1U, 0);
            s_run = 0U;
        }
        s_stable = 3U;                          /* 等松手，避免长按连续触发 */
        s_beep   = 8U;                          /* 约 160ms 蜂鸣 */
    }

    if (s_beep > 0U)
    {
        s_beep --;
        ctrl_buzzer(1U);
    }
    else
    {
        ctrl_buzzer(0U);
    }
}
