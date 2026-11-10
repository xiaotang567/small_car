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
#include "ctrl_f4.h"          /* 车模控制：电机 / 编码器 / 舵机 / MPU9250 / 速度 PID（从 MSPM0 移植） */
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
volatile uint32_t flt_bfar = 0U;      /* BusFault 出错地址 */
/* ★ 2026-11-07（FW=2100）：**主循环看门狗复位时，卡在哪一步**（"面包屑"）
   由 SysTick 看门狗写进 RTC->BKP6R，开机读回来，心跳里显示成 LM= */
volatile uint32_t flt_mark = 0U;

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* ★★★ 2026-11-07（FW=2098）：**SysTick 必须是最高优先级** ★★★
     HAL_Init() 把 SysTick 设成 15（最低），而 UART4=6、DCMI=5、DMA=6 都比它高。
     后果：只要这几个中断密集一点（舵机 1Mbps 线的回显、噪声，摄像头行中断），
           SysTick 就被插不进来 ⇒ **HAL_GetTick() 停止前进** ⇒
           而整个工程里所有"超时保护"都是 `while (HAL_GetTick() - t0 < N)` ⇒
           它们**永远不超时** ⇒ 主循环卡死，上位机看到的就是"已连接但 0 B/s"。
     把 SysTick 提到 0，等于给所有超时兜底：谁也别想把时间基准饿死。 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);
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
    /* ★ 主循环看门狗复位时记下的"卡在第几步"（RTC 备份寄存器，掉电不丢）*/
    flt_mark   = RTC->BKP6R;
    RTC->BKP6R = 0U;
    if (0xA5A5A5A5UL == RTC->BKP0R)
    {
      flt_pc   = RTC->BKP1R;
      flt_lr   = RTC->BKP2R;
      flt_cfsr = RTC->BKP3R;
      flt_hfsr = RTC->BKP4R;
      flt_bfar = RTC->BKP5R;
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

  // ==================== 车模控制（从 MSPM0 工程移植）====================
  //   电机 TIM2_CH3/4 + DIR、编码器 TIM5 + Z、舵机 UART4、MPU9250 软 I2C、速度 PID / yaw。
  //   ★ 放在摄像头之后、WiFi 之前：WiFi 初始化最长阻塞 60 秒，电机必须在它之前就已经
  //     被置成"停"（PWM=0 + DIR=0），否则这 60 秒里车可能会乱动。
  ctrl_all_init();

  /* ★★ 2212：**上电默认就开电机**（用户要求：测试新 DRV888 驱动）
       · 开环占空比 40%（`w 4000 0`，量纲 ±10000 = ±100%）
       · 保护默认关（g_protect_enable = 0，见 protect_f4.c）
       · 断链也不强制停车（pid_f4.c 里那一处已经用 g_protect_enable 门控）
     ⚠️ 上车/上电之前**先把轮子架空**，或者准备好按【停止】/【急停】。
     想恢复"上电不转"：把下面这 3 行注释掉，或者把 CTRL_BOOT_RUN_DUTY 改成 0。*/
  #define CTRL_BOOT_RUN_DUTY   0           /* ★ 2217：改用 app.c 的 APP_BOOT_RUN_DUTY（延时 2 秒，等舵机/相机初始化完）*/
  if (CTRL_BOOT_RUN_DUTY > 0)
  {
      ctrl_open_loop(1U);                   /* 开环：速度环让位 */
      ctrl_motor_set(1U, (int16_t)CTRL_BOOT_RUN_DUTY);
      ctrl_motor_set(2U, 0);
  }
  /* ★ 2210：把上次存进 Flash 的 PID 参数读回来（没有存过就用代码里的默认值）。
     存的动作是命令 ss / 界面上的'存到板子'按钮；存在内部 Flash 扇区 11（0x080E0000）。*/
  (void)pid_store_load();

  // ==================== WiFi 模块：复位 → 连热点 → 连上位机 ====================
  //   ⚠️ 阻塞，最长约 60 秒。⚠️ **上位机（VOFA）要先把 TCP 服务端开着**，否则这一步会失败。
  //   返回值 = 卡在哪一步，直接由 LED 闪出来：
  //       1 = 模块没应答(SPI/供电/RST) → 闪 1 下
  //       2 = 连不上热点(SSID/密码/不是2.4G) → 闪 2 下
  //       3 = 连不上电脑(IP不对 / 上位机没开 / 防火墙) → 闪 3 下
  //       0 = 成功 → LED 常亮
  {
    uint8_t ret;

    ctrl_oled_boot(3U);                  /* ★ 开机阶段 3：正在连 WiFi / 连上位机（最长 60 秒）*/
    ret = wifi_f4_init();
    ctrl_oled_boot(4U);                  /* ★ 开机阶段 4：WiFi 初始化返回，准备进主循环 */
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
