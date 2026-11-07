/*********************************************************************************************************************
* 文件名              motor_f4.c
* 适用平台            STM32F407
* 说明                电机驱动（TIM2_CH3/CH4 两路 PWM + PB0/PB1 方向）
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"


//===================================================★ 电机（TIM2_CH3/CH4 + DIR）===================================================
//   为什么用 TIM2：TIM3_CH1(PA6)/TIM4_CH1(PB6)/TIM8_CH1(PC6) 全撞 DCMI，
//   TIM2 的 CH3/CH4 正好在 PA2/PA3。两路同一定时器 ⇒ 同频 17kHz（和原工程一致，超出人耳）。
static int16_t g_motor_duty[2] = {0, 0};

static void motor_apply (uint8_t idx, int16_t duty)
{
    int32_t  d = (int32_t)duty;
    uint8_t  dir;
    uint32_t ccr;

    if (d >  CTRL_PWM_FULL) { d =  CTRL_PWM_FULL; }
    if (d < -CTRL_PWM_FULL) { d = -CTRL_PWM_FULL; }

    dir = (uint8_t)((d >= 0) ? 1U : 0U);           /* 1 = 正转（和原工程 MOTOR_DIR_FORWARD 一致） */
    if (0U == idx) { dir ^= (uint8_t)CTRL_M1_DIR_INV; }
    else           { dir ^= (uint8_t)CTRL_M2_DIR_INV; }

    ccr = (uint32_t)(((d >= 0) ? d : -d) * (int32_t)CTRL_PWM_ARR / (int32_t)CTRL_PWM_FULL);

    if (0U == idx)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (0U != dir) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        TIM2->CCR3 = ccr;
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (0U != dir) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        TIM2->CCR4 = ccr;
    }

    g_motor_duty[idx] = duty;
}

void ctrl_motor_init (void)
{
    GPIO_InitTypeDef gi;

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_2 | GPIO_PIN_3;         /* PA2/PA3 = TIM2_CH3/CH4 */
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_HIGH;
    gi.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin       = GPIO_PIN_0 | GPIO_PIN_1;         /* PB0/PB1 = DIR */
    gi.Mode      = GPIO_MODE_OUTPUT_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = 0;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    TIM2->CR1   = 0U;
    TIM2->PSC   = 0U;
    TIM2->ARR   = CTRL_PWM_ARR;
    TIM2->CCR3  = 0U;
    TIM2->CCR4  = 0U;
    /* CH3/CH4：PWM 模式 1（OCxM=110）+ 预装载（OCxPE=1）⇒ 0x68 / 0x6800 */
    TIM2->CCMR2 = (uint16_t)(0x0068U | 0x6800U);
    TIM2->CCER  = 0U;
    TIM2->EGR   = 0x0001U;
    TIM2->CCER  = (uint16_t)(0x0100U | 0x1000U);    /* CC3E | CC4E */
    TIM2->CR1   = 0x0001U;                          /* CEN */

    motor_apply(0U, 0);
    motor_apply(1U, 0);
}

void ctrl_motor_set (uint8_t id, int16_t duty)
{
    if (1U == id)      { motor_apply(0U, duty); }
    else if (2U == id) { motor_apply(1U, duty); }
}

void ctrl_motor_stop (void)
{
    motor_apply(0U, 0);
    motor_apply(1U, 0);
}

int16_t ctrl_motor_get (uint8_t id)
{
    if (1U == id) { return g_motor_duty[0]; }
    if (2U == id) { return g_motor_duty[1]; }
    return 0;
}
