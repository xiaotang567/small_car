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
/* USER CODE END 1 */
