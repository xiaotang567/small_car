/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>            /* sprintf */
#include "wifi_f4.h"          /* 逐飞 WiFi6B21-SPI 模块驱动（F407/HAL 移植版） */
#include "board_f4.h"         /* 板级：时钟树 168MHz + 总钻风摄像头(DCMI) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCMI_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ★ 上次 HardFault 的现场（由 HardFault_Handler 写进 RTC 备份寄存器，开机读回）*/
volatile uint32_t flt_pc   = 0U;      /* 出错处的 PC */
volatile uint32_t flt_lr   = 0U;
volatile uint32_t flt_cfsr = 0U;      /* 可配置故障状态 */
volatile uint32_t flt_hfsr = 0U;

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // ★ 重新配时钟到 168MHz（CubeMX 生成的 SystemClock_Config 里 HSE 值可能不对）
  //   调试时看这两个全局变量：
  //      board_clock_src  = 1 用了外部晶振 / 2 晶振没起来退回了内部 HSI
  //      board_sysclk_hz  = 实际系统时钟，应该是 168000000
  board_clock_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DCMI_Init();
  /* USER CODE BEGIN 2 */

  // ★★★ 记录"这次到底是**为什么复位**的"（硬件诊断第一手资料）★★★
  //   RCC->CSR 的高位是**粘滞**的复位来源标志，读一次就知道：
  //     bit25 BORRSTF  = **欠压/掉电**（供电被拉低 —— 摄像头电流大时最可能）
  //     bit26 PINRSTF  = 复位引脚被拉低（按键/RST）
  //     bit27 PORRSTF  = 上电复位
  //     bit28 SFTRSTF  = 软件复位
  //     bit29 IWDGRSTF = 独立看门狗
  //     bit30 WWDGRSTF = 窗口看门狗
  //     bit31 LPWRRSTF = 低功耗复位
  //   ⚠️ 读完要写 RMVF 清掉，否则下次看到的还是老标志。
  cam_rst_flags = (uint32_t)((RCC->CSR >> 25) & 0x7FU);

  /* ★ 开机读"上次是不是 HardFault 挂的"，以及挂在哪 */
  {
    RCC->APB1ENR |= (1UL << 28);
    PWR->CR      |= (1UL << 8);
    if (0xA5A5A5A5UL == RTC->BKP0R)
    {
      flt_pc   = RTC->BKP1R;
      flt_lr   = RTC->BKP2R;
      flt_cfsr = RTC->BKP3R;
      flt_hfsr = RTC->BKP4R;
      RTC->BKP0R = 0U;                    /* 只报一次 */
    }
  }
  RCC->CSR |= RCC_CSR_RMVF;

  // ---- 板载 LED（PB2）：没有屏幕，靠它看状态 ----
  board_led_init();

  // ==================== 摄像头（DCMI + DMA）====================
  //   MX_DCMI_Init() 已经把 DCMI 外设配好了（8位 / 硬件同步 / VSYNC低有效 / PCLK上升沿），
  //   这里补的是：DMA + 两个中断 + 启动第一帧。
  board_cam_init();

  // ==================== WiFi 模块：复位 → 连热点 → 连上位机 ====================
  //   ⚠️ 阻塞，最长约 60 秒。⚠️ **上位机（VOFA）要先把 TCP 服务端开着**，否则这一步会失败。
  //   返回值 = 卡在哪一步，直接由 LED 闪出来：
  //       1 = 模块没应答(SPI/供电/RST) → 闪 1 下
  //       2 = 连不上热点(SSID/密码/不是2.4G) → 闪 2 下
  //       3 = 连不上电脑(IP不对 / 上位机没开 / 防火墙) → 闪 3 下
  //       0 = 成功 → LED 常亮
  {
    uint8_t ret = wifi_f4_init();
    board_led_show((0 == ret) ? LED_CODE_READY : ret);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    app_loop();

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
