/*********************************************************************************************************************
* 文件名              encoder_f4.c
* 适用平台            STM32F407
* 说明                编码器（TIM5 正交编码 + PC0 Z 相计数）
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"


//===================================================★ 编码器（TIM5 正交 + Z 相）===================================================
//   SMS=011 = 编码器模式 3：TI1、TI2 的**每个边沿**都计数 ⇒ 4 倍频。
//   TIM5 是 32 位，不用管溢出。
//   Z 相（PC0/EXTI0）**只数次数，不清 CNT** —— 清 CNT 会把这一拍的速度增量吃掉。
static int32_t  g_enc_delta   = 0;      /* 上一个 20ms 窗口的增量 */
static int32_t  g_enc_total   = 0;      /* 累计 */
static int16_t  g_enc_rpm_x10 = 0;      /* 0.1RPM */
static volatile uint32_t g_enc_z = 0U;

void ctrl_encoder_init (void)
{
    GPIO_InitTypeDef gi;

    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_0 | GPIO_PIN_1;         /* PA0/PA1 = TIM5_CH1/CH2 */
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_HIGH;
    gi.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin       = GPIO_PIN_0;                      /* PC0 = Z 相 */
    gi.Mode      = GPIO_MODE_IT_RISING;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = 0;
    HAL_GPIO_Init(GPIOC, &gi);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 7U, 0U);       /* 比摄像头低，绝不能抢采集中断 */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    TIM5->CR1   = 0U;
    TIM5->PSC   = 0U;
    TIM5->ARR   = 0xFFFFFFFFUL;
    /* CC1S=01 CC2S=01；IC1F/IC2F=0110（fDTS/4, N=8）滤毛刺 —— 原工程 QUAD 模式踩过没滤波的坑 */
    TIM5->CCMR1 = (uint16_t)(0x01U | (0x06U << 4) | (0x01U << 8) | (0x06U << 12));
    TIM5->CCER  = (uint16_t)(0x0001U | 0x0010U);    /* CC1E | CC2E，上升沿 */
    TIM5->SMCR  = (uint16_t)(0x0003U | (0x05U << 4));   /* SMS=011 编码器模式3 | TS=101 */
    TIM5->CNT   = 0U;
    TIM5->EGR   = 0x0001U;
    TIM5->CR1   = 0x0001U;
}

int32_t ctrl_encoder_total (void)  { return g_enc_total; }
int32_t ctrl_encoder_delta (void)  { return g_enc_delta; }
int32_t ctrl_encoder_rpm_x10 (void){ return (int32_t)g_enc_rpm_x10; }

void ctrl_encoder_clear (void)
{
    __disable_irq();
    TIM5->CNT = 0U;
    __enable_irq();
    g_enc_delta = 0;
    g_enc_total = 0;
}

uint8_t ctrl_encoder_z_cnt (void) { return (uint8_t)(g_enc_z & 0xFFU); }

/* 每 20ms 由 board_ctrl_tick() 调：取增量 → 清 CNT → 算 M 法转速 */
void ctrl_encoder_tick (uint32_t dt_us)
{
    int32_t d;

    __disable_irq();
    d = (int32_t)TIM5->CNT;
    TIM5->CNT = 0U;
    __enable_irq();

#if CTRL_ENC_DIR_INV
    d = -d;
#endif

    g_enc_delta  = d;
    g_enc_total += d;

    if (0U == dt_us) { dt_us = (uint32_t)CTRL_PERIOD_MS * 1000U; }

    /* RPM×10 = d×600000000 / (CPR × **实际周期µs**)
       ★★★ 2026-11-07（FW=2117）：**改到 µs 精度** ★★★
       之前用 HAL_GetTick() 的 ms 算，20ms 窗口里 ±1ms 就是 ±5% ⇒ 500RPM 正好 ±25 的抖动，
       实测日志里 rpm 就是 466~526 在摆。改用 DWT 周期计数器（168MHz）算 dt，
       分辨率约 6ns，测速噪声立刻降下来。*/
    g_enc_rpm_x10 = (int16_t)(((int64_t)d * 600000000LL) /
                              ((int64_t)CTRL_ENC_CPR * (int64_t)dt_us));
}

/* stm32f4xx_it.c 的 EXTI0_IRQHandler 转发到这里 */
void ctrl_exti0_irq (void)
{
    if (0U != (EXTI->PR & (1UL << 0)))
    {
        EXTI->PR = (1UL << 0);
        g_enc_z ++;
    }
}
