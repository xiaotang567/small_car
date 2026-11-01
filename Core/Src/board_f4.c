/*********************************************************************************************************************
* 文件名称          board_f4
* 适用平台          STM32F407
* 说明              板级支持：时钟树 + 总钻风摄像头(DCMI)采集。见 board_f4.h 的说明。
*
* 修改记录
* 日期              作者                备注
* 2026-10-13        织命                first version
********************************************************************************************************************/

#include "board_f4.h"
#include <stdio.h>
#include <string.h>

// CubeMX 在 main.c 里定义的 DCMI 句柄（外设本身由 MX_DCMI_Init() 配好了）
extern DCMI_HandleTypeDef hdcmi;

//===================================================内部变量===================================================

volatile uint8_t  board_clock_src = 0;          // 1=HSE  2=退回HSI
volatile uint32_t board_sysclk_hz = 0;


// 诊断用（排"f=0"的关键线索）

// 4 字节对齐（DCMI 走 32 位 DMA，必须对齐，否则 HardFault）
//===================================================内部变量===================================================


//===================================================时钟树===================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重新配置到 168MHz（放在 USER CODE BEGIN SysInit 里调用）
// 备注信息     优先用 HSE；起不来就退回 HSI，保证程序还能跑（board_clock_src 会告诉你用了哪个）
//-------------------------------------------------------------------------------------------------------------------
//===================================================摄像头===================================================


//===================================================板载 LED（PB2）===================================================

#define LED_PORT    GPIOB
#define LED_PIN     GPIO_PIN_2

static uint8_t  led_code = LED_CODE_RETRY;
static uint32_t led_t0   = 0;

void board_clock_init (void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    uint8_t use_hse = 0;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // ---- 0. 先把系统切回 HSI、并关掉 PLL ----
    //   ⚠️ 这一步不能省！CubeMX 生成的 SystemClock_Config() 很可能**已经把 PLL 打开了**
    //      （哪怕 SYSCLK 走的是 HSI）。而 ST 的 HAL_RCC_OscConfig 在发现
    //      "PLL 已使能、但配置和我要设的不一样" 时会**直接返回 HAL_ERROR**，
    //      于是下面重配时钟整段失效 → 结果以为"HSE 起不来"，白退回 HSI。
    {
        RCC_ClkInitTypeDef tmp = {0};
        tmp.ClockType    = RCC_CLOCKTYPE_SYSCLK;
        tmp.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
        HAL_RCC_ClockConfig(&tmp, FLASH_LATENCY_0);

        __HAL_RCC_PLL_DISABLE();
        while (0 != __HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) { }    // 等 PLL 真的关掉
    }

    // ---- 先试 HSE + PLL ----
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;

#if   (BOARD_HSE_MHZ == 8)
    osc.PLL.PLLM = 8;                           // 8MHz  / 8  = 1MHz
#elif (BOARD_HSE_MHZ == 25)
    osc.PLL.PLLM = 25;                          // 25MHz / 25 = 1MHz
#else
    #error "BOARD_HSE_MHZ 只能是 8 或 25"
#endif
    osc.PLL.PLLN = 336;                         // 1MHz  × 336 = 336MHz
    osc.PLL.PLLP = RCC_PLLP_DIV2;               // 336 / 2 = **168MHz**
    osc.PLL.PLLQ = 7;                           // 336 / 7 = 48MHz（USB 用）

    if (HAL_RCC_OscConfig(&osc) == HAL_OK)
    {
        use_hse = 1;
    }
    else
    {
        // HSE 起不来（晶振没焊/焊错/坏了）→ 退回内部 HSI，至少还能跑
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState       = RCC_HSI_ON;
        osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM       = 16;                // HSI 16MHz / 16 = 1MHz
        osc.PLL.PLLN       = 336;
        osc.PLL.PLLP       = RCC_PLLP_DIV2;
        osc.PLL.PLLQ       = 7;
        HAL_RCC_OscConfig(&osc);
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;       // AHB  = 168MHz
    clk.APB1CLKDivider = RCC_HCLK_DIV4;         // APB1 = 42MHz（SPI2 在这条上）
    clk.APB2CLKDivider = RCC_HCLK_DIV2;         // APB2 = 84MHz
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);

    board_clock_src = use_hse ? 1 : 2;
    board_sysclk_hz = HAL_RCC_GetHCLKFreq();
    SystemCoreClockUpdate();
}
//===================================================时钟树===================================================


//===================================================摄像头（DCMI + DMA）===================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DCMI 采到一整帧的回调（HAL 的弱函数，这里覆盖它）
// 备注信息     SNAPSHOT 模式：采完一帧就停，由主循环处理完再调 board_cam_start() 重新开
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA 搬满整圈缓冲 = 采到一帧
// 备注信息     ★ 配合"连续采集 + 循环 DMA"用：DCMI 一直采，DMA 一直往缓冲里转圈，
//               每转满一圈就回调一次 ⇒ **不需要手动重启**，也不用依赖 DCMI 的 FRAME 中断
//               （那个中断默认没使能，之前用 FrameEventCallback 其实可能一次都没进来过）。
//-------------------------------------------------------------------------------------------------------------------

void board_led_init (void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    gi.Pin   = LED_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gi);
}

// 切换要显示的状态码（换码时把计时清零，保证每次从"第一下"开始数）
void board_led_show (uint8_t code)
{
    if (led_code != code)
    {
        led_code = code;
        led_t0   = HAL_GetTick();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     LED 闪烁节奏（主循环每轮调一次，不阻塞）
//   就绪     -> 常亮
//   1/2/3 下 -> 对应 wifi_f4_init() 的返回码（模块/热点/电脑）
//   慢闪     -> 还没就绪，正在自动重试
//-------------------------------------------------------------------------------------------------------------------
void board_led_task (void)
{
    uint32_t now = HAL_GetTick();
    uint32_t t   = now - led_t0;

    if (LED_CODE_READY == led_code)
    {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);     // 常亮
        return;
    }

    if ((LED_CODE_NO_MODULE == led_code) ||
        (LED_CODE_NO_WIFI   == led_code) ||
        (LED_CODE_NO_PC     == led_code))
    {
        const uint32_t ON   = 120;                              // 亮 120ms
        const uint32_t OFF  = 250;                              // 灭 250ms
        const uint32_t GAP  = 1200;                             // 一组之间的停顿
        uint8_t  n   = led_code;                                // 闪几下 = 状态码
        uint32_t cyc = ((uint32_t)n * (ON + OFF)) + GAP;
        uint32_t p   = t % cyc;
        uint8_t  lit = 0;

        if (p < ((uint32_t)n * (ON + OFF)))
        {
            lit = (((p % (ON + OFF)) < ON) ? 1 : 0);
        }
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, lit ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return;
    }

    // 慢闪：1Hz
    HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                      (((t % 1000) < 500) ? GPIO_PIN_SET : GPIO_PIN_RESET));
}
//===================================================板载 LED===================================================
