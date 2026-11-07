/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* ★★★ 2026-11-07（FW=2100）：**主循环看门狗**（软件）★★★
   现场：并上 PC10/PC11 后 OLED 直接定格 ⇒ 主循环停了，但模块还揣着 TCP
   ⇒ 上位机永远"已连接 + 0 B/s"，只能断电。
   这里在 SysTick（优先级 0，谁也抢不走）里盯着 app_loop 的脚印：
     · g_wdt_on    = 1 之后才开始看（开机那段几十秒的初始化不算）
     · g_wdt_pause = 1 时暂停（只包住必然长时间阻塞的 wifi_f4_init）
     · 6 秒没踩脚印 ⇒ NVIC_SystemReset()（软件复位，心跳里 RST=04）
   效果：卡死最多 6 秒后自己重启、自己重连，不用再断电。*/
extern volatile uint32_t g_loop_tick;
extern volatile uint8_t  g_wdt_on;
extern volatile uint8_t  g_wdt_pause;
extern volatile uint8_t  g_loop_mark;      /* 主循环走到第几步（面包屑）*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* ★★★ 2026-11-02：把出错现场存进 RTC 备份寄存器 → 软复位 → 开机由心跳打印 ★★★
     为什么不用 while(1)：那样只知道"挂了"，不知道"挂在哪"。
     备份寄存器（RTC->BKPxR）掉电前不会丢、软复位更不会丢 ⇒ 下次上电就能读出来。 */
  uint32_t *sp;

  sp = (uint32_t *)__get_MSP();   /* 本工程不用 RTOS，永远走主栈（ARMCC5 没有 __get_LR）*/

  RCC->APB1ENR |= (1UL << 28);            /* PWREN：备份域访问要先开这个 */
  PWR->CR      |= (1UL << 8);             /* DBP=1：解除备份域写保护 */

  RTC->BKP0R = 0xA5A5A5A5UL;              /* 魔数：确实发生过 HardFault */
  RTC->BKP1R = sp[6];                     /* ★ 出错处的 PC（栈帧里的返回地址）*/
  RTC->BKP2R = sp[5];                     /* LR */
  RTC->BKP3R = SCB->CFSR;                 /* 可配置故障状态：位段告诉是哪类故障 */
  RTC->BKP4R = SCB->HFSR;
  RTC->BKP5R = SCB->BFAR;              /* ★ 出错的具体地址（BusFault 时有效）*/

  NVIC_SystemReset();                     /* 立刻重启，别卡在 while(1) */
  while (1) { }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* ★ 2026-11-07（FW=2100）：**故障里绝不能 while(1)**。
     故障处理器的优先级是负数（比任何中断都高）⇒ 一旦在这里死等，
     连 SysTick 都被挡住、主循环看门狗也救不回来 ⇒ 只能断电。
     改成"记现场 + 软复位"，和 HardFault_Handler 一致。*/
  RCC->APB1ENR |= (1UL << 28);
  PWR->CR      |= (1UL << 8);
  RTC->BKP0R = 0xA5A5A5A5UL;
  RTC->BKP3R = SCB->CFSR;
  RTC->BKP4R = SCB->HFSR;
  RTC->BKP5R = SCB->BFAR;
  NVIC_SystemReset();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* ★ 同理：这里 while(1) 会把整个系统钉死（故障优先级最高）⇒ 改成记现场 + 软复位 */
  RCC->APB1ENR |= (1UL << 28);
  PWR->CR      |= (1UL << 8);
  RTC->BKP0R = 0xA5A5A5A5UL;
  RTC->BKP3R = SCB->CFSR;
  RTC->BKP4R = SCB->HFSR;
  RTC->BKP5R = SCB->BFAR;
  NVIC_SystemReset();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* ★ 同理：记现场 + 软复位，绝不死等 */
  RCC->APB1ENR |= (1UL << 28);
  PWR->CR      |= (1UL << 8);
  RTC->BKP0R = 0xA5A5A5A5UL;
  RTC->BKP3R = SCB->CFSR;
  RTC->BKP4R = SCB->HFSR;
  RTC->BKP5R = SCB->BFAR;
  NVIC_SystemReset();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* ★ 主循环看门狗（见文件头 PV 段说明）：SysTick 优先级已提到 0，这个判断一定跑得到 */
  if ((0U != g_wdt_on) && (0U == g_wdt_pause) &&
      ((HAL_GetTick() - g_loop_tick) > 6000U))
  {
    /* ★ 先把"卡在第几步"记进 RTC 备份寄存器（掉电不丢），开机心跳里会显示成 LM= */
    RCC->APB1ENR |= (1UL << 28);
    PWR->CR      |= (1UL << 8);
    RTC->BKP6R    = (uint32_t)g_loop_mark;
    NVIC_SystemReset();                 /* 主循环停了 6 秒 ⇒ 立刻重启（RST=04 可查）*/
  }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* ==================== 摄像头（DCMI）的中断 — 2026-10-13 加 ====================
 *   这两个处理函数在 board_f4.c 里，这里只是转发。
 *   ⚠️ 放在 USER CODE 段里，CubeMX 重新生成不会被冲掉。
 *   ⚠️ 但如果你以后在 CubeMX 里**正式配了 DCMI 的 DMA/中断**，CubeMX 会自己生成
 *      同名函数 → 会重复定义。届时把下面这两段删掉即可。
 */
extern void board_cam_dma_irq(void);
extern void board_cam_irq(void);

void DMA2_Stream1_IRQHandler(void)
{
  board_cam_dma_irq();
}

void DCMI_IRQHandler(void)
{
  board_cam_irq();
}

/* ★ 2026-11-03：188 模式下用 TIM6 每 50µs 轮询 DCMI->SR 的 VSYNC 位
   （PB7 必须留在 DCMI，上一版把它改成 TIM4 就彻底没图了）*/
extern void board_vsync_poll_irq(void);

void TIM6_DAC_IRQHandler(void)
{
  board_vsync_poll_irq();
}

/* ★ 2026-11-05：车模控制层（ctrl_f4.c）的两个中断
   · UART4   = 幻尔总线舵机（PC10=TX / PC11=RX，1Mbps）—— 收/发都走这里，非阻塞
   · EXTI0   = 编码器 Z 相（PC0）—— 只数圈，不清 CNT（清了会吃掉这一拍的速度增量）*/
extern void ctrl_uart4_irq(void);
extern void ctrl_exti0_irq(void);

void UART4_IRQHandler(void)
{
  ctrl_uart4_irq();
}

void EXTI0_IRQHandler(void)
{
  ctrl_exti0_irq();
}
/* USER CODE END 1 */
