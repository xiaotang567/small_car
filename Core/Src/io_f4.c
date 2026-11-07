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
