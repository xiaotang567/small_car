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
static DMA_HandleTypeDef hdma_dcmi;

volatile uint8_t  board_clock_src = 0;          // 1=HSE  2=退回HSI
volatile uint32_t board_sysclk_hz = 0;

volatile uint32_t cam_frame_cnt   = 0;
volatile uint8_t  cam_ready       = 0;

// 诊断用（排"f=0"的关键线索）
volatile uint32_t cam_err_cnt        = 0;
volatile uint32_t cam_vsync_cnt      = 0;
volatile uint32_t cam_fne_cnt        = 0;
volatile uint8_t  cam_last_start_err = 0;

// 4 字节对齐（DCMI 走 32 位 DMA，必须对齐，否则 HardFault）
__attribute__((aligned(4))) uint8_t cam_image[CAM_H][CAM_W];
//===================================================内部变量===================================================


//===================================================时钟树===================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重新配置到 168MHz（放在 USER CODE BEGIN SysInit 里调用）
// 备注信息     优先用 HSE；起不来就退回 HSI，保证程序还能跑（board_clock_src 会告诉你用了哪个）
//-------------------------------------------------------------------------------------------------------------------
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
void HAL_DCMI_ConvCpltCallback (DCMI_HandleTypeDef *hdcmi_)
{
    (void)hdcmi_;
    cam_frame_cnt++;
    cam_ready = 1;
}

// DCMI 出错（同步错 / FIFO 溢出）时会进这里 —— 计数，别让它默默过去
void HAL_DCMI_ErrorCallback (DCMI_HandleTypeDef *hdcmi_)
{
    (void)hdcmi_;
    cam_err_cnt++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     轮询 DCMI 状态 + DMA 完成标志（主循环每轮调一次，只读寄存器，很快）
// 备注信息     🔑 "f 一直 0" 的坑：DMA 搬满一圈之后，HAL 走的是
//                 "使能 DCMI 的 FRAME 中断 → 等那个中断" 这条路才回调
//                 `HAL_DCMI_ConvCpltCallback` 的，而 **FRAME 中断默认根本没使能**
//               ⇒ 回调一次都不会进 ⇒ 现象就是"数据明明在搬，`f` 却永远是 0"。
//               ✅ 这里**直接数 DMA 自己的"传输完成"标志**（`DMA2->LISR` 的 TCIF1），
//                  完全不依赖 DCMI 的中断，**也不去写 DCMI 的 ICR**
//                  （上一版我加了 ICR 清标志，结果把已经能跑的局面又弄关了 —— 别动它）。
//-------------------------------------------------------------------------------------------------------------------
// 🔥 2026-10-15 新增两个"丈量"用的计数器 —— 用来**实测摄像头一行到底多少字节**，
//    不用再靠猜（752 / 376 / 188 我们都试错了）。
//       cam_bytes = 从 DMA 的 NDTR 增量累加出来的**搬运字节总数**
//       cam_line  = DCMI 报"一行结束"的次数（RISR 的 LINE 位）
//    ⇒ 心跳里算 **Δbytes ÷ Δline = 每行字节数 = 真实行宽**。
volatile uint32_t cam_bytes = 0;
volatile uint32_t cam_line  = 0;

// 🔥 **帧边界在缓冲里的位置（第几行）** —— 轮询 DCMI 的 FRAME 标志拿到，0xFFFF = 还没抓到。
//   🔑 为什么用轮询而不是 VSYNC 中断：VSYNC 那条路实测不可靠（日志里 `ec` 抖得厉害）。
//      `RISR` 的 FRAME 位是**硬件置起**的，主循环里盯它，置起那一刻读一次 NDTR
//      ⇒ 就是"上一帧刚写完、下一帧即将开始"的位置 ✓ 从这一行开始读 = 整帧从帧首开始。
//   ⚠️ 只清 FRAME 这一位（`ICR = 0x01`），别的位一概不动（LINE 那一位由下面单独清）。
volatile uint16_t cam_frame_row = 0xFFFFU;

// 活引脚"占空比"采样（主循环每轮采一次，很便宜）
volatile uint32_t live_hi[4] = {0};
volatile uint32_t live_n     = 0;

static void cam_frame_start (void);     // 前置声明（定义在后面；它做"帧首重置 + 武装"）

void board_cam_poll (void)
{
    static uint16_t nd_prev = 0;
    uint16_t nd  = (uint16_t)(hdma_dcmi.Instance->NDTR & 0xFFFFU);
    uint32_t ris;

    // ---- 0. ★ 活引脚采样：PA6(PCLK) / PB7(VSYNC) / PA4(HREF) / PC6(D0) ----
    //   🔑 这一项能**实时**回答"HREF 到底跳不跳"：
    //      占空比 ~80% ⇒ 在跳（好）；恒 0 或 恒 100 ⇒ 不跳（坏）。
    //      （万用表读到的 2.68V 既可能是"81% 占空比的平均值"、也可能是"静态分压"，
    //        只有这个占空比能区分。）
    if (GPIOA->IDR & GPIO_PIN_6) { live_hi[0]++; }
    if (GPIOB->IDR & GPIO_PIN_7) { live_hi[1]++; }
    if (GPIOA->IDR & GPIO_PIN_4) { live_hi[2]++; }
    if (GPIOC->IDR & GPIO_PIN_6) { live_hi[3]++; }
    live_n++;

    // ---- 1. 用 NDTR 的增量算"到底搬了多少字节" ----
    //   循环模式下 NDTR 从 X 递减到 0 再重装，所以：
    //     本次 ≤ 上次  ⇒ 正常递减，增量 = 上次 - 本次
    //     本次 >  上次  ⇒ 回绕了，增量先只算"上次减到 0"那一段（下一轮会接着算）
    //   ⚠️ NDTR 的单位是**数据项**（我们配的是 32 位字）⇒ 要 ×4 才是字节。
    if (nd <= nd_prev) { cam_bytes += (uint32_t)(nd_prev - nd) * 4UL; }
    else               { cam_bytes += (uint32_t)nd_prev * 4UL; }
    nd_prev = nd;

    // ---- 2. 数"DCMI 报了几次一行结束"（RISR bit4 = LINE）----
    //   ⚠️ **只清 LINE 这一个位**（往 ICR 写 0x10），别的位一概不动。
    ris = hdcmi.Instance->RISR;
    if (ris & 0x10UL)
    {
        cam_line++;
        hdcmi.Instance->ICR = 0x10UL;
    }

    // ---- 2.5 ★★ 盯 DCMI 的 **FRAME** 标志（bit0）⇒ 锁住"帧边界在缓冲的第几行" ★★ ----
    //   🔥 这是"图片是同一张图的不同部分/整帧转了个圈"的正解：
    //      摄像头一帧的**帧首**在缓冲里的位置，和 DMA 的**绕回点**并不是同一个地方
    //      （两者差一个固定的相移）⇒ 从绕回点开始读，画面就是整帧**被转了个圈**、
    //      接缝落在画面中间 ⇒ 用户看到的"同一张图的不同部分" ✓
    //   ⇒ 抓到帧首的行号，读取时**从帧首开始** ⇒ 接缝被推到画面最底边 ✓
    //   ⚠️ 用这一轮**已经读好的** `ris`（前面读 RISR 时 FRAME 位就在里面），别重复读寄存器。
    if (ris & 0x01UL)
    {
        hdcmi.Instance->ICR = 0x01UL;                       // 只清 FRAME 这一位
        cam_frame_row = (uint16_t)board_cam_write_row();    // 记下帧首在缓冲里的行号
        cam_frame_cnt++;                                    // ★ 这里才是**唯一可靠**的帧计数
        cam_frame_start();                                  // ★★ 帧首处理：重置收割计数 + 武装
                                                            //    （原来写在 VSYNC 中断里，而那个中断
                                                            //      从没使能过 ⇒ 这段以前从没执行过！）
    }

    // ---- 3. 原来的状态位 & DMA 完成计数 ----
    if (__HAL_DCMI_GET_FLAG(&hdcmi, DCMI_FLAG_VSYNC)) { cam_vsync_cnt++; }
    if (__HAL_DCMI_GET_FLAG(&hdcmi, DCMI_FLAG_FNE))   { cam_fne_cnt++;   }

    // ---- 2.7 ★★ 帧首的**第二条来源**（不依赖任何中断）：轮询 VSYNC 状态位的**边沿** ★★
    //   🔑 为什么这条靠得住：VSYNC 的"有效窗口"约占一帧的 10% ⇒ 一帧 16.7ms 里约 **1.7ms**，
    //      而主循环一轮只要 ~0.1ms ⇒ **一定能在窗口内看到它**，不会漏（这和"电平持续时间短"不同）。
    //   ⚠️ 它只负责**对齐一次**（`cam_frame_start()` 里的 `cam_aligned` 保证），
    //      之后帧首由"行计数器回绕"自检测负责（见 board_cam_harvest）。
    //      为什么不每帧都用它对齐：轮询总有 ~0.1ms 延迟 ≈ 3 行，若每帧都归零，
    //      一帧内只能走完 480-3 行 ⇒ `disp_fill` 到不了 120 ⇒ 永远等超时 ⇒ 又会出缝。
    {
        static uint8_t vs_prev = 0U;
        uint8_t        vs_now  = (__HAL_DCMI_GET_FLAG(&hdcmi, DCMI_FLAG_VSYNC)) ? 1U : 0U;

        if ((0U != vs_now) && (0U == vs_prev)) { cam_frame_start(); }   // 上升沿 = 帧首
        vs_prev = vs_now;
    }

    //   ⚠️ 位号：`DMA2->LISR` 低 6 位是 Stream0、**bits[11:6] 才是 Stream1**
    //      ⇒ Stream1 的传输完成 = **TCIF1 = bit11 = 0x800**。
    //   ⚠️ 2026-10-14：**这里不再 `cam_frame_cnt++`**（改到上面 FRAME 标志那一段去数）。
    //      原因：`board_cam_poll()` 是主循环里轮询的，而 HAL 的 DMA 中断会**先把 TCIF 清掉**
    //      ⇒ 这段基本永远看不到标志（`f` 一直是 0 就是这个原因），
    //        而 FRAME 标志是**硬件置起、我们不使能它的中断**、所以一定等得到 ⇒ 一个都不漏 ✓
    //      ⚠️ 两处都数会**重复计数**（BPF 就会算成一半），所以只能留一处。
    if (DMA2->LISR & 0x00000800UL)
    {
        DMA2->LIFCR = 0x00000800UL;         // 写 1 清标志
        cam_ready = 1;                      // 限流交给 main.c 按时间做
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     引脚"悬浮/断线"检测 —— 给线轮流加内部上拉/下拉，看占比会不会被"拉过去"
// 备注信息     🔑 为什么要这么做：**悬空的输入脚会被邻近走线耦合，读出来也像 ~50% 在跳**，
//               和真的 PCLK 看上去一模一样 —— 光看"占比"分不出来。
//               但真信号在驱动时，STM32 内部那个 ~40kΩ 的拉电阻**根本拉不动它**，
//               于是"无拉 / 上拉 / 下拉"三次占比几乎一样；
//               而悬空的线会被直接拉到 100% / 0% —— **一眼就能分开**。
//               为不挡 DCMI 干活，开机后只跑约 9 秒（3 轮）就自动收手并恢复"无拉"。
//-------------------------------------------------------------------------------------------------------------------
volatile uint32_t dbg_flt[4][3]  = {{0}};
volatile uint8_t  dbg_flt_done   = 0;
volatile uint32_t dbg_vsync_edge = 0;
static   uint8_t  flt_last_vs    = 0;

// 被查的 4 根线：0=PA6(PCLK)  1=PB7(VSYNC)  2=PA4(HREF)  3=PC6(D0)
static GPIO_TypeDef *const flt_port [4] = { GPIOA,      GPIOB,      GPIOA,      GPIOC      };
static const uint16_t      flt_shift[4] = { 12,         14,         8,          12         };  // PUPDR 内的位偏移
static const uint16_t      flt_mask [4] = { GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_6 };

#define FLT_PHASE_MS    200U        // 每一小段 200ms
#define FLT_TOTAL_MS    9000U       // 开机后最多测 9 秒就收手

// 给第 p 根线设拉（0=无拉 1=上拉 2=下拉），其余三根恢复"无拉"
static void flt_set_pull (uint8_t p, uint32_t pull)
{
    uint8_t i;

    for (i = 0; i < 4U; i++)
    {
        uint32_t v = (i == p) ? pull : 0U;

        flt_port[i]->PUPDR = (flt_port[i]->PUPDR & ~(3UL << flt_shift[i])) | (v << flt_shift[i]);
    }
}

void board_float_probe (void)
{
    static uint8_t  ph_pin = 0, ph_idx = 0;             // ph_idx: 0=无拉 1=上拉 2=下拉
    static uint32_t ph_t0 = 0, ph_hi = 0, ph_n = 0, t_boot = 0;
    uint32_t now = HAL_GetTick();

    if (0 == t_boot) { t_boot = now; ph_t0 = now; }

    // VSYNC 边沿数一直数（不管当前在测哪根）—— 证明这是"真场同步"而不是噪声
    {
        uint8_t v = (GPIOB->IDR & GPIO_PIN_7) ? 1 : 0;

        if (v != flt_last_vs) { dbg_vsync_edge++; flt_last_vs = v; }
    }

    if ((now - t_boot) > FLT_TOTAL_MS)                  // 测够了，收手，别再动这几根脚
    {
        if (0 == dbg_flt_done) { flt_set_pull(0, 0U); dbg_flt_done = 1U; }
        return;
    }

    if ((now - ph_t0) >= FLT_PHASE_MS)                  // 这一段结束 → 结算，并切到下一段
    {
        dbg_flt[ph_pin][ph_idx] = (0U == ph_n) ? 0UL : (ph_hi * 100UL / ph_n);
        ph_hi = 0; ph_n = 0; ph_t0 = now;

        ph_idx++;
        if (ph_idx >= 3U) { ph_idx = 0U; ph_pin = (uint8_t)((ph_pin + 1U) & 0x3U); }
        flt_set_pull(ph_pin, (0U == ph_idx) ? 0U : ((1U == ph_idx) ? 1U : 2U));
    }

    if (flt_port[ph_pin]->IDR & flt_mask[ph_pin]) { ph_hi++; }
    ph_n++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     抓一份 DCMI / DMA 的寄存器快照（每轮调，很便宜）
// 备注信息     🔑 用来回答"启动函数明明返回成功，为什么 DCMI 一个字节都不搬"
//-------------------------------------------------------------------------------------------------------------------
volatile uint32_t dbg_dcmi_cr    = 0;
volatile uint32_t dbg_dma_ndtr   = 0;
volatile uint32_t dbg_dma_err    = 0;
volatile uint8_t  dbg_dcmi_state = 0;
volatile uint8_t  dbg_dma_state  = 0;
volatile uint32_t dbg_pb7_moder  = 0;
volatile uint32_t dbg_pb7_af     = 0;
volatile uint32_t dbg_dcmi_clk   = 0;
volatile uint32_t dbg_dcmi_sr    = 0;
volatile uint32_t dbg_dcmi_ris   = 0;
volatile uint8_t  dbg_pol_idx    = 0;
volatile uint8_t  dbg_pol_locked = 0;
volatile uint32_t dbg_pin_bad    = 0;
volatile uint32_t dbg_dma_en     = 0;
volatile uint32_t dbg_dma_lisr   = 0;

void board_cam_dbg (void)
{
    uint32_t bad = 0;

    dbg_dcmi_cr    = hdcmi.Instance->CR;
    dbg_dma_ndtr   = hdma_dcmi.Instance->NDTR;
    dbg_dma_err    = hdma_dcmi.ErrorCode;
    dbg_dcmi_state = (uint8_t)hdcmi.State;
    dbg_dma_state  = (uint8_t)hdma_dcmi.State;

    // PB7 的复用情况 + DCMI 时钟（DCMI->SR 全 0 时靠这三个定位）
    dbg_pb7_moder  = (GPIOB->MODER >> 14) & 0x3U;     // 期待 2
    dbg_pb7_af     = (GPIOB->AFR[0] >> 28) & 0xFU;    // 期待 13
    dbg_dcmi_clk   = (RCC->AHB2ENR >> 0) & 0x1U;      // 期待 1

    dbg_dcmi_sr    = hdcmi.Instance->SR;              // B0=HSYNC B1=VSYNC B2=FNE
    dbg_dcmi_ris   = hdcmi.Instance->RISR;            // 原始中断标志：B1=FRAME B2=OVR B3=ERR B4=VSYNC B5=LINE

    // ---- 11 根 DCMI 引脚逐根查：MODER 必须 = 2(复用) **且 AFR 必须 = 13(DCMI)** ----
    //   ⚠️ 上一版只查了 MODER（是不是复用模式），**复用号也可能是错的** ——
    //      引脚在复用模式，但复用的却是 SPI/USART 之类 ⇒ DCMI 照样一个字节都收不到。
    if ((2U != ((GPIOA->MODER >>  8) & 0x3U)) || (13U != ((GPIOA->AFR[0] >> 16) & 0xFU))) { bad |= (1UL << 0); }  // PA4  HSYNC
    if ((2U != ((GPIOA->MODER >> 12) & 0x3U)) || (13U != ((GPIOA->AFR[0] >> 24) & 0xFU))) { bad |= (1UL << 1); }  // PA6  PIXCLK
    if ((2U != ((GPIOB->MODER >> 12) & 0x3U)) || (13U != ((GPIOB->AFR[0] >> 24) & 0xFU))) { bad |= (1UL << 2); }  // PB6  D5
    if ((2U != ((GPIOB->MODER >> 14) & 0x3U)) || (13U != ((GPIOB->AFR[0] >> 28) & 0xFU))) { bad |= (1UL << 3); }  // PB7  VSYNC
    if ((2U != ((GPIOC->MODER >> 12) & 0x3U)) || (13U != ((GPIOC->AFR[0] >> 24) & 0xFU))) { bad |= (1UL << 4); }  // PC6  D0
    if ((2U != ((GPIOC->MODER >> 14) & 0x3U)) || (13U != ((GPIOC->AFR[0] >> 28) & 0xFU))) { bad |= (1UL << 5); }  // PC7  D1
    if ((2U != ((GPIOC->MODER >> 16) & 0x3U)) || (13U != ((GPIOC->AFR[1] >>  0) & 0xFU))) { bad |= (1UL << 6); }  // PC8  D2
    if ((2U != ((GPIOC->MODER >> 18) & 0x3U)) || (13U != ((GPIOC->AFR[1] >>  4) & 0xFU))) { bad |= (1UL << 7); }  // PC9  D3
    if ((2U != ((GPIOE->MODER >>  8) & 0x3U)) || (13U != ((GPIOE->AFR[0] >> 16) & 0xFU))) { bad |= (1UL << 8); }  // PE4  D4
    if ((2U != ((GPIOE->MODER >> 10) & 0x3U)) || (13U != ((GPIOE->AFR[0] >> 20) & 0xFU))) { bad |= (1UL << 9); }  // PE5  D6
    if ((2U != ((GPIOE->MODER >> 12) & 0x3U)) || (13U != ((GPIOE->AFR[0] >> 24) & 0xFU))) { bad |= (1UL << 10); } // PE6  D7
    dbg_pin_bad = bad;

    // ---- DMA 流到底使能没有 + DMA 的原始标志 ----
    dbg_dma_en   = (hdma_dcmi.Instance->CR >> 0) & 0x1U;   // EN 位。期待 1
    dbg_dma_lisr = DMA2->LISR;                             // 低 6 位 = Stream1 的 FEIF/DMEIF/TEIF/HTIF/TCIF
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把极性组合写到 DCMI 的 CR（i 的 bit0/1/2 = PCKPOL / HSPOL / VSPOL）
// 备注信息     🔑 **只改 CR 这三个位，不 Stop、不重启！**
//               之前那版每 2 秒 `HAL_DCMI_Stop()` + 重启，实测会把 DCMI 的同步状态搞坏
//               （`SR` 直接冻住），搜索本身反而成了干扰源 —— 那一轮的"极性都试过了"结论不作数。
//-------------------------------------------------------------------------------------------------------------------
static void board_cam_apply_polarity (uint8_t i)
{
    uint32_t cr = DCMI->CR;

    cr &= ~(DCMI_CR_PCKPOL | DCMI_CR_HSPOL | DCMI_CR_VSPOL);
    if (i & 0x1U) { cr |= DCMI_CR_PCKPOL; }
    if (i & 0x2U) { cr |= DCMI_CR_HSPOL;  }
    if (i & 0x4U) { cr |= DCMI_CR_VSPOL;  }
    DCMI->CR = cr;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     极性自动搜索：每 2 秒换一种组合，采到帧就锁死
// 备注信息     🔑 参考手册对 HSPOL/VSPOL 的措辞是自相矛盾的
//               （"the level when the data are not valid" vs "active low/high"），
//               所以**不猜了，让板子自己试**：8 种组合扫一遍，采到帧就停在那儿。
//               起始那组用手册推出的 PCKPOL=1/HSPOL=1/VSPOL=0（也就是 CR=0x4061）。
//               锁定后： dbg_pol_locked=1，dbg_pol_idx 就是**正确的那组** ⇒ 照着写死即可。
//               全程不 Stop / 不重启，不会破坏 DCMI 状态。
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★ 极性组号的"手动按钮"：**存进 RTC 备份寄存器 ⇒ 按一下复位键就换下一组** ★★
// 备注信息     🔥 为什么必须这么做（2026-10-14 踩的坑）：
//   DCMI **一旦在跑，中途改 PCKPOL/HSPOL/VSPOL 是没法重新同步的** ——
//   改动"迟一拍"才生效，你在标着第 N 组的那几秒里看到的画面，其实是**上一组**配出来的。
//   ⇒ 我就是照着这个错位的观察把极性锁到第 1 组，结果**直接黑屏**（第 1 组根本不同步）。
//   ⇒ 正解：**极性必须在 `board_cam_start()` 之前就配好**，让 DCMI 从第一帧就按它同步。
//      而"换组"就等价于"重启一次" ⇒ 用 RTC 备份寄存器（**能扛复位**）存组号：
//        开机：读出组号 v ⇒ 用 v 启动采集；同时把 v+1 存回去
//        ⇒ **按一下 RST = 开机 = 下一组**，不用重编译、不用接线、画面干净。
//   ⚠️ `RTC->BKPxR` 写入必须先 `PWR->CR |= PWR_CR_DBP`（备份域写保护）。
//   ⚠️ 掉电（不是复位）时若无 VBAT，备份域复位 ⇒ 组号回到默认值；按 RST 换组不受影响。
//-------------------------------------------------------------------------------------------------------------------
#define CAM_POL_MAGIC       0xA500U     // 用来判断"备份寄存器里到底有没有存过"
#define CAM_POL_DEFAULT_IDX 0U          // ★ 首次上电用哪一组 —— **2026-10-14 实测：只有第 0 组能出图**

static uint8_t cam_pol_stored (void)
{
    uint32_t v;

    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;                       // 打开备份域写保护
    v = RTC->BKP0R;
    PWR->CR &= ~PWR_CR_DBP;

    if ((v & 0xFF00U) != CAM_POL_MAGIC) { return CAM_POL_DEFAULT_IDX; }   // 没存过 ⇒ 默认组
    return (uint8_t)(v & 0x07U);
}

// 把某一组记为"要用的组"（扛复位 ⇒ 掉电/按 RST 之后还从它起）
static void cam_pol_store (uint8_t idx)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;
    RTC->BKP0R = (uint32_t)CAM_POL_MAGIC | (uint32_t)(idx & 0x07U);
    PWR->CR &= ~PWR_CR_DBP;
}

// 📌 早先还有一个 `cam_pol_store_next()`（存"当前组+1"），现在 `board_cam_pol_search()`
//    直接调 `cam_pol_store(下一组)` 就够了 —— 删掉，免得留个"声明了没引用"的编译警告。

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     极性组号（供 `board_cam_init()` 在**启动采集之前**调用）
//-------------------------------------------------------------------------------------------------------------------
uint8_t board_cam_pol_index (void)
{
    return cam_pol_stored();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     极性：**普查模式** 或 **自动寻找模式**（主循环里调一次）
// 备注信息     🔥 两条实测结论，缺一不可（2026-10-14 三个版本对照得出来的）：
//   ① **极性必须在"采集启动之后"再写一次**，DCMI 才按它同步
//      （只写在 `Start` 之前 ⇒ `SR=0000 RIS=0000 er=0`、缓冲全 0 ⇒ 换遍 8 组都没图）。
//   ② 用户实测：**"极性连续交换那版有不是花的图"** ⇒ 干净的那一组确实存在，
//      但"**只要有数据就锁定**"这个判据太弱 —— 有些组能出数据、图却是花的
//      （同步对了但采样边沿/行边界没对）⇒ 自动模式会**锁死在一个"花的"组上**。
//   ★ 所以默认开 **普查模式**：**每 2 秒无条件换下一组**，8 组轮着走；
//      PC 端按组号存 8 张 PNG（`cam_pol%d.png`）⇒ **一眼挑出哪一组是干净的**，
//      挑出来之后把它写进 `CAM_POL_STORE` 并把 `CAM_POL_SURVEY` 改回 0。
//   ★ 自动模式（`CAM_POL_SURVEY = 0`）：一有数据就锁死 —— 只在"已经知道哪组对"之后才用。
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 🔥🔥 2026-10-15 **极性轮换已关闭**（`CAM_POL_ROTATE = 0`）—— 遗留问题的清理
//   当年为什么必须要它：摄像头"PCLK 消隐=0" ⇒ DCMI 分帧错乱 ⇒ 只能靠**一次次换极性组去撞**，
//   用户实测"删掉轮换就黑屏"是**那个前提下的结论** ✓
//   现在前提已经不存在了：传感器被 SCCB 正确配成 188×120（心跳 `完整帧` 一直在涨 ✓）⇒
//   DCMI 从一开始就能正确分帧 ⇒ **继续每 1.5 秒写一次 `DCMI->CR` 只剩干扰风险** ✗
//   ⚠️⚠️ **关轮换的同时必须把"起始组"设成当前能用的那一组**（`CAM_POL_PRIMER` 改成 1），
//       否则 `board_cam_init()` 会用 primer 启动 ⇒ 可能落在"会花/黑屏"的组上 ✗✗
//   📌 回退：把 `CAM_POL_ROTATE` 改回 1 即可恢复老行为。
//-------------------------------------------------------------------------------------------------------------------
#define CAM_POL_ROTATE      0       // ★ 1 = 老行为（轮转一整圈、停在目标组）；0 = 只用 PRIMER 组、完全不轮换
#define CAM_POL_MODE        4       // （仅在 CAM_POL_ROTATE=1 时起作用）0 = 普查；4 = 轮转一圈停在目标组
#define CAM_POL_PRIMER      1U      // ★ 起始组 = **1**（= 当前实测能出图的那一组，原文是 0）
#define CAM_POL_STORE       1U      // ★ 目标组：**第 1 组**（实测图最清晰的那组）
#define CAM_POL_STEP_MS     1500UL  // 轮转间隔（每次换一组 = 一次"变化"）
#define CAM_POL_MIN_STEPS   8U      // ★ **必须走满一整圈（8 步）之后**才允许停下来

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     极性：普查 / **先打底、再交替几次、最后停在目标组**（主循环里调一次）
// 备注信息     🔥🔥🔥 这里踩过的坑（2026-10-14，整整绕了三轮），顺序很重要：
//   **① 极性"直接设定"没用 —— DCMI 只在"运行中发生**变化**"的那一刻才重新同步。**
//        证据：极性轮换那版**一直有图**（它一直在变）；任何"直接锁定成某一组"的版本都**没图**；
//        普查时第 1 组清晰，是因为**它前面正好是第 0 组**（那个 0→1 的"变化"才是关键）。
//   **② 单次切换还不够**：用户实测"**还没出图就变成 1 了**"依然没图 ——
//        说明切换时**第 0 组还没有真正同步稳**（切换的"起点状态"也算数）。
//   ⇒ 所以正确动作是：**先用打底组稳住 → 再反复 0/1 交替（给多次"切换进第 1 组"的机会）
//      → 最后停在第 1 组且不再写。**
//   ⚠️ 第 ② 条说明：**切换是有"起点状态"依赖的**，不是随便怎么切都能锁上；
//      所以"重现某组清晰"必须把**整个动作序列**一起重现，而不只是那个组号。
//   ⚠️ 换摄像头/换板子/换接线之后，两个组号都要重新普查（`CAM_POL_MODE = 0`）。
//-------------------------------------------------------------------------------------------------------------------
void board_cam_pol_search (void)
{
    static uint8_t started = 0;

#if (CAM_POL_ROTATE == 0)
    // ★ 轮换已关闭：只把"起始组"配一次，之后**不再动 DCMI**（见上面那段的理由）✓
    (void)cam_pol_store;      // 仅轮换模式用得到（引用一下，免"定义了没引用"的警告）
    if (0U == started)
    {
        started = 1U;
        board_cam_apply_polarity(dbg_pol_idx);   // dbg_pol_idx 在 board_cam_init 里 = CAM_POL_PRIMER
    }
    return;
#else
    static uint32_t t0   = 0;
    static uint8_t  step = 0;           // 已经轮转了几步
    static uint8_t  done = 0;           // 1 = 已经停在目标组（此后不再动）

    if (0U == started)
    {
        started = 1U;
        dbg_pol_idx = (CAM_POL_MODE == 0) ? cam_pol_stored() : CAM_POL_PRIMER;
        board_cam_apply_polarity(dbg_pol_idx);
        t0 = HAL_GetTick();
        return;
    }

    if ((HAL_GetTick() - t0) < CAM_POL_STEP_MS) { return; }
    t0 = HAL_GetTick();

    if (0U == done)
    {
        // ★ 轮转一格：这一"写"就是一次**变化** ⇒ DCMI 会重新同步一次
        step ++;
        dbg_pol_idx = (uint8_t)((dbg_pol_idx + 1U) & 0x07U);
        cam_pol_store((uint8_t)((dbg_pol_idx + 1U) & 0x07U));

#if (CAM_POL_MODE != 0)
        // ★ 走满一整圈之后，**一旦来到目标组就停住**（0→1→2→…→7→0→1 ⇒ 停在 1）
        //   🔑 为什么非要"走满一整圈"才允许停：
        //      实测切换是**有"起点状态"依赖的**，而**整圈轮转是唯一被证实"一定有图"的动作**
        //      ⇒ 用整圈保证最后一次"0→1 的切换"（和普查成功时完全一样的条件）能落到实处。
        //      ⚠️ 之前试过"只交替 0/1"、"打底后单次切换" —— **都不出图**，别退回去。
        if ((step >= CAM_POL_MIN_STEPS) && (dbg_pol_idx == CAM_POL_STORE))
        {
            done           = 1U;
            dbg_pol_locked = 1U;
        }
#endif

        board_cam_apply_polarity(dbg_pol_idx);
    }
    else
    {
        // ★ 已经停在目标组：**不再换值**，但**每 1.5 秒把同一个值重写一遍**（保险）
        //   🔑 为什么留着这一写：实测"写同一个值"**既不会破坏已经同步好的状态**
        //      （早先"锁 0"那版就是每 1.5 秒重写、图一直在），**也可能正好是维持同步的那一下**
        //      ⇒ 留着最稳；反正它不会换组、不会再把画面搅乱。
        board_cam_apply_polarity(dbg_pol_idx);
    }
#endif
}

// 把两个中断转发进来（真正的处理函数在 stm32f4xx_it.c 里，见那边的 USER CODE BEGIN 1）
void board_cam_dma_irq (void) { HAL_DMA_IRQHandler(&hdma_dcmi); }
void board_cam_irq     (void) { HAL_DCMI_IRQHandler(&hdcmi); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动一次采集（SNAPSHOT：采满一帧自动停）
// 返回参数     uint8   0 = 成功
// 备注信息     ⚠️ HAL 这个 Length 参数的单位是 **32 位字**，所以要 /4
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VSYNC 回调 —— **实时记录"帧边界"落在缓冲的第几行**
// 备注信息     🔥 解决"图上有一道线、线的上下是同一张图颠倒的两半"：
//   根因：**摄像头一帧比我们的缓冲大**（90140B 装不下一帧）
//        ⇒ DMA 循环搬运时会在缓冲**中间绕回** ⇒ 那道线就是"绕回点"，
//          线的上下就是"帧尾 + 帧首"，所以看着像同一张图被颠倒拼在一起。
//   ⇒ 对策：在帧边界（VSYNC）那一刻读出 **DMA 已经写了多少字**，换算成"第几行"存进 `cam_off`；
//      上传时**从这一行开始取**（见 main.c 的行采样）⇒ 图从"帧首"开始，
//      那道缝被推到缓冲的最末尾（＝画面最底边），基本看不见了。
//   ⚠️ 这里**不做 Stop/Start** —— 只读一下 NDTR，开销极小，所以可以放心地每帧都进。
//-------------------------------------------------------------------------------------------------------------------
volatile uint16_t cam_off = 0;          // （历史遗留，不再使用；保留定义免得别处引用悬空）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 全画面缩略图的"逐行收割" —— 边采边抽点，把整帧装进 22.5KB
// 备注信息     🔥 为什么要这么做：整帧 752×480 = **360KB**，F407 的 128KB RAM **物理装不下**。
//   ⇒ 唯一办法：让 DMA 继续用那个 90KB 环形缓冲当"滑动窗口"（存最近 120 行），
//     主循环**在每一行被覆盖之前**（一条行只活 120 行 × 35µs = 4.2ms）把它抽点成
//     **每 4 列取 1 列**（752→188），再按**每 4 行取 1 行**（480→120）填进 `disp_img`
//     ⇒ **整幅画面 188×120 = 22560B**，比例与整帧一致（1.57:1）✓
//   🔑 帧首对齐：VSYNC 中断那一刻，DMA 的写入位置就是"新一帧第 0 行"的起点
//     ⇒ `disp_ln = 0`、`disp_next_row = 当时的写入行` ⇒ 之后主循环逐行收割就对齐 ✓
//   ⚠️ 主循环必须至少每 4.2ms 跑一圈（否则行会在被收割前被覆盖 ⇒ 漏行）。
//-------------------------------------------------------------------------------------------------------------------
__attribute__((section(".ccmram"))) uint8_t disp_img[CAM_H][CAM_TXW];  /* ★ 放 CCM：只被 CPU 读写，省 22KB 主 SRAM */      // ★ 全画面 188×120（22560B，顶替原来的 cam_snap）
volatile uint32_t disp_frame  = 0;               // 全画面的帧数（VSYNC 一次 +1）
volatile uint8_t  disp_hold   = 0;               // 1 = 冻结收割（发图期间，保证整帧来自同一次）
volatile uint32_t disp_pass   = 0;               // 行号回绕次数（**仅诊断用**，别拿它判"凑齐"）
volatile uint32_t disp_lag_max = 0;              // ★ 单次收割最多补了几行（≥30 ⇒ 主循环太慢，在漏行）
volatile uint8_t  disp_fill   = 0;               // ★★ 本帧已经填到第几行（1..CAM_H）；**VSYNC 时清 0**
volatile uint8_t  disp_done   = 0;               // ★★ 1 = **本帧的 120 行全部来自同一帧**，可以拿去发
volatile uint8_t  disp_vsync_flag = 0;   // ★ VSYNC 中断置位（真实帧首信号）
volatile uint16_t disp_in_frame   = 0;   // ★ 本帧内已收行数（0..119），VSYNC 归零
volatile uint8_t  disp_armed  = 0;               // ★★ 1 = 已经跨过帧首，允许拷贝/计数（见下面说明）
// ★★ 120 位的"哪些 dst 行已经填过"位图（15 字节）—— **去重计数**用，见 `board_cam_harvest()` ★★
//   🔥 2026-10-15：这是"没图了 / 图里有横线"的根因修复 —— 旧的"max(dst+1)"计数在
//      主循环**突发补行跨越 disp_ln 回绕点**时会被清零 ⇒ 永远数不到 120 ⇒ `disp_done` 不置位 ✗
static   uint8_t  disp_map[15] = {0};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 开始新一轮采集：**计数与位图只在这里清零**（相位机进入相 1 时调用）★
//   为什么要抽成一个函数：`disp_fill`/`disp_map` 的"清零时刻"必须和"计数起点"严格配套，
//   以前在回绕处顺手清零 ⇒ 突发跨回绕就丢进度（详见 board_cam_harvest 里的说明）✓
//-------------------------------------------------------------------------------------------------------------------
void board_cam_new_pass (void)
{
    uint8_t i;

    disp_fill  = 0U;
    disp_in_frame = 0U;
    for (i = 0U; i < 15U; i++) { disp_map[i] = 0U; }

    disp_done  = 0U;      // 锁存的"采齐"标志（只在这里清）
    disp_armed = 0U;      // 等下一个帧边界再武装
    disp_hold  = 0U;      // 解冻
}
static   uint16_t disp_next_row = 0;             // 下一条要收割的缓冲行（0..CAM_H-1）
static   uint32_t disp_ln       = 0;             // 正在填的是帧里的第几行（0 .. cam_span-1）

//-------------------------------------------------------------------------------------------------------------------
// ★★ 一帧到底有多少条**摄像头行** —— 2026-10-15 用"内容重复周期"实测定为 **240** ★★
//   证据（在 PC 端对拼出来的 188×120 图做 12 行块匹配）：
//       块 0-11  ↔ 块 60-71    corr = 1.00
//       块 12-23 ↔ 块 72-83    corr = 1.00
//       块 24-35 ↔ 块 84-95    corr = 1.00
//     ⇒ **dst 0..59 与 dst 60..119 完全重复** ⇒ 我们走的 480 行里装了**两帧**
//     ⇒ 真实一帧 = **240 行** ✓（同时排除了"一行里装两条摄像头行"：
//        图内左半 vs 右半相关只有 -0.16 ⇒ 缓冲一行就是一条完整行 ✓）
//   ⚠️ 之前写死"一帧 480 行、每 4 行取 1 行"，实际是"**240 行、每 2 行取 1 行**"
//      ⇒ 一帧只走到一半就"数满 120 行" ⇒ **画面里同一场景出现两遍** ✓ 就是这个 bug。
//   📌 做成**可实时改**（上位机命令 `z <n>`）⇒ 以后再纠结"到底多少行"不用重编译。
//-------------------------------------------------------------------------------------------------------------------
volatile uint16_t cam_span  = 240U;              // 一帧的摄像头行数（实测 240）
static   uint16_t cam_dstep = 2U;                // 抽行步长 = cam_span / CAM_H（240/120 = 2）

void board_cam_set_span (uint16_t rows)
{
    if (rows < (uint16_t)CAM_H) { rows = (uint16_t)CAM_H; }

    cam_span  = rows;
    cam_dstep = (uint16_t)(rows / (uint16_t)CAM_H);
    if (0U == cam_dstep) { cam_dstep = 1U; }

    disp_ln    = 0U;                             // 相位归零：从下一次回绕重新数
    disp_fill  = 0U;
    disp_armed = 0U;
    disp_done  = 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// ★★★ 运行时的"采集模式" —— 0 = 相机全分辨率(752×480，走环形缓冲+抽行)，1 = 相机就是 188×120 ★★★
//   🔑 为什么需要它：手册 5.1 明确写了摄像头有三档**同视野**分辨率
//      752×480 / 376×240 / **188×120**，而且 **FPS 可设 1~500**。
//      ⇒ 只要用配置串口把摄像头设成 **188 列 × 120 行**，那么：
//          · 一帧 = 188×120 = **22560 字节**（正好等于我们现在的缓冲大小）
//          · 缓冲里**天然就是一整帧** ⇒ **不需要环形缓冲、不需要抽行、不需要那套相位机**
//          · 每行 188 字节 ⇒ 横向也不用抽点（`cam_dcol = 1`）
//          · 一帧 120 行 ⇒ `cam_span = 120`（行计数器回绕周期 = 帧周期）
//      ⇒ 整条链路从"几百行时序博弈"退化成"拷一份内存" ✓✓✓
//   ⚠️ 默认仍然是模式 0（752 宽），**必须先确认摄像头真的被配成了 188×120 再切**，
//      否则固件按 188 收、相机却吐 752 ⇒ 数据错位（画面会花/黑）。
//   ⚠️ 切换时 DMA 的传输长度也必须跟着改 ⇒ 必须先 Stop 再用新长度 Start（见 `board_cam_set_mode`）。
//-------------------------------------------------------------------------------------------------------------------
volatile uint16_t cam_row_bytes = 752U;      // 摄像头一行的**字节数**（= 图像列数）
volatile uint16_t cam_ring_rows = 120U;      // ★ DMA 环形缓冲能装的行数 = 90240B / cam_row_bytes
                                             //   752 宽 → 120 行；188 宽 → **480 行（= 一整帧）** ✓
volatile uint16_t cam_mode      = 0U;        // 0 = 752 宽；1 = 188 宽
static   uint16_t cam_dcol      = 4U;        // 横向抽点：752/188 = 4；188 宽时 = 1
// ⚠️ `cam_aligned` 的**定义提前到这里**（`board_cam_set_mode()` 要用它来重置相位对齐）
static   uint8_t  cam_aligned   = 0U;

void board_cam_set_mode (uint8_t mode)
{
    if (mode > 1U) { mode = 0U; }

    cam_mode      = mode;
    cam_row_bytes = (0U == mode) ? 752U : 188U;
    cam_dcol      = (uint16_t)((0U == mode) ? 4U : 1U);
    cam_ring_rows = (uint16_t)(((uint32_t)CAM_W * (uint32_t)CAM_H) / (uint32_t)cam_row_bytes);
    // ★ 188 模式：传感器输出 188×480（只做列 bin4）→ 缓冲区 90240B ÷ 188B = **480 行 = 一整帧** ✓
    //    显示仍是 188×120，靠 set_span(480) 自动得到 dstep=4（每 4 行取 1 行）✓

    // ⚠️ DMA 的传输长度 = 一行字节数 × 行数，模式一变就必须重启 DMA（HAL 不支持在线改长度）
    // ★ 2026-10-15：改用 `board_cam_restart()` —— 传感器刚被改过寄存器（输出格式/时序都变了），
    //   DCMI 若停在不同步状态就**再也等不到 VSYNC**（心跳里 V=0/f=0 就是这个症状）✓
    cam_last_start_err = board_cam_restart();

    cam_aligned = 0U;                            // 允许重新对齐一次相位
    board_cam_set_span((uint16_t)((0U == mode) ? 240U : 120U));   // ★ 传感器实际帧长=120 行（行 bin4 ✓）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★ 重启 DCMI（Stop + Start，带重试）—— 传感器改完寄存器后**必须**调它 ★★
//   🔑 为什么需要：传感器的输出格式/时序变了（binning 改了行数和像素时钟），
//      DCMI 若停在不同步的状态，就**再也等不到 VSYNC**（症状：心跳 `V=0 f=0`、画面全无）✓
//   ⚠️ `HAL_DCMI_Stop()` 在 DCMI 正处于一帧中间时可能返回 BUSY/ERROR ⇒ **重试 3 次** ✓
//-------------------------------------------------------------------------------------------------------------------
uint8_t board_cam_restart (void)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        (void)HAL_DCMI_Stop(&hdcmi);
        HAL_Delay(2);

        if (0U == board_cam_start()) { return 0U; }
        HAL_Delay(5);
    }
    cam_last_start_err = 1U;
    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★ 帧首处理：重置"收割计数"并武装 ★★
//   🔥🔥🔥 2026-10-15 **最关键的一次修正**：这段逻辑原来写在 `HAL_DCMI_VsyncEventCallback()` 里，
//     可它**从来不会执行** —— 整个工程**从没使能过 DCMI 的 VSYNC 中断**
//     （没有 `__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_VSYNC)`，`stm32f4xx_it.c` 里也没有 DCMI_IRQHandler）。
//     ⇒ 后果依次是（今天踩的三次坑，其实是**同一个根因**）：
//         ① `disp_ln` 永远归不了零 ⇒ 画面**分段错位 / 接缝**
//         ② `disp_fill` 永远是满值 ⇒ 收割刚拷一行就判"完成" ⇒ 画面**像死的不更新** + 满屏横条
//         ③ `disp_armed` 永远是 0    ⇒ 一行都不拷 ⇒ **全黑**
//   ✅ 现在改到 `board_cam_poll()` 里**轮询 FRAME 标志**来触发 —— 这条路是**被验证过能用的**
//      （`cam_frame_cnt` 一直靠它计数、BPF 也算得对；FRAME 是硬件置起、我们没使能它的中断
//       ⇒ 它一直挂着直到被清 ⇒ **一个都不会漏** ✓）
//   ⚠️ 两个刻意的取舍：
//     1) **不重置 `disp_ln`**（只在开机对齐一次）。原因：轮询总比真实帧首晚 ~L 行（L≈3~5），
//        若每帧都归零，一帧内只走得完 480-L 行 ⇒ `dst` 到不了 119 ⇒ `disp_fill` 永远差一点
//        ⇒ **永远等超时** ⇒ 又是接缝。`disp_ln` 自由跑即可：一帧 480 行、它每行 +1，
//        所以它 mod 480 天然等于"帧内第几行" ✓ 不需要每帧重置。
//     2) **不清 `disp_done`**（锁存），由相位机开始新一轮时清 —— 否则它会形成"146µs 可见窗口"。
//-------------------------------------------------------------------------------------------------------------------
// ⚠️ `cam_aligned` 的**定义在上面**（disp 全局量那一段），这里不再重复定义

static void cam_frame_start (void)
{
    if (0U == cam_aligned)                          // ★ 开机第一次：把收割相位对齐到帧首
    {
        disp_ln       = 0U;
        disp_next_row = (uint16_t)board_cam_write_row();
        cam_aligned   = 1U;
    }

    disp_fill  = 0U;                                // 本帧的"已填行数"重新数
    disp_armed = 1U;                                // ★★ 从现在起允许拷贝/计数
    disp_frame ++;
}

// ⚠️ 这个回调**目前永远不会被调用**（VSYNC 中断没使能，见上面的说明）。
//    留着它纯粹是为了：万一以后有人把 `DCMI_IT_VSYNC` 打开，帧首就是这条更精确的路径。
// ⚠️ 这个回调**是否真的被调用**，就靠 `cam_vsync_isr_cnt` 说话（心跳里显示成 V=）：
//    V 在涨 ⇒ VSYNC 中断确实在工作（帧首是硬件精确的）；V 恒 0 ⇒ 它根本没进来，
//    此时**帧首完全靠下面的"行计数器回绕自检测"兜底**（见 board_cam_harvest 里那一句）。
volatile uint32_t cam_vsync_isr_cnt = 0;

void HAL_DCMI_VsyncEventCallback (DCMI_HandleTypeDef *hdcmi_)
{
    disp_vsync_flag = 1U;                 // ★ 告诉收割：真实帧首到了（用来对齐一趟）
    (void)hdcmi_;
    cam_vsync_isr_cnt ++;
    cam_frame_start();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★ "武装位" disp_armed 存在的唯一理由 ★★
//   🔥 2026-10-15 踩到的坑：`disp_fill`（本帧已填行数）**只在 VSYNC 时清零**，
//     而一轮刚开始时它还可能留着上一轮的 **120** ⇒ 收割**刚拷完一行**就满足
//     `disp_fill >= CAM_H` ⇒ **立刻又冻住**（`disp_done=1`）⇒ 相位机就把这幅几乎没变的图发出去
//     ⇒ 现象：**画面像死的一样不更新**（转摄像头也没反应），并且满是横条。
//   ✅ 解法：只有在**跨过一个帧首之后**（ISR 里 `disp_armed = 1`）才允许拷贝和计数
//     ⇒ 计数一定是在"帧首 → 帧尾"这一次完整扫描里累积的 ⇒ 单调而且必定完整 ✓
//     相位机在开始新一轮时把 `disp_armed`/`disp_done`/`disp_fill` 一起清 0。
//-------------------------------------------------------------------------------------------------------------------

// 主循环每轮调一次：把 DMA 已经写完的行收割进 disp_img（每 4 行取 1 行、每 4 列取 1 列）
//
//   🔥🔥 2026-10-15 修"图上仍有一道分割线 / 像上一张没覆盖完"：
//     **旧的"凑齐"判据是错的** —— 原来拿 `disp_ln` 回绕 480→0 当"凑齐"，
//     可 VSYNC 会在**半路**把 `disp_ln` 清零 ⇒ 计数到 480 的那一刻，disp_img 里那 120 行
//     **未必都来自同一帧**（后半截还留着上一帧的内容）
//     ⇒ 画面上一道"上半新、下半旧"的分割线，正是这个。
//   ✅ 新判据：`disp_fill` = 本帧已填过的**最大行号+1**，**VSYNC 时清零**；
//     `disp_done` = disp_fill 达到 CAM_H ⇒ **这一帧的 120 行确实都齐了** ⇒ 才允许拿去发。
void board_cam_harvest (void)
{
    uint16_t w    = (uint16_t)board_cam_write_row();   // DMA 当前写到第几行（还没写完的那行）
    uint32_t nlag = 0U;

    /* ★★★ 用**真实帧边界（VSYNC）**对齐一趟 ★★★
       以前用 disp_ln 回绕（每 120 行）当帧边界，但那个相位和传感器的实际帧边界**无关**
       ⇒ 一趟常常横跨两帧 ⇒ 画面出现一条**横向接缝**（位置随机，重启几次碰巧对齐就消失）✗
       现在：VSYNC 一到 ⇒ 本趟结束、从 0 重新数 ⇒ **一趟 = 一帧** ✓ */
    if (0U != disp_vsync_flag)
    {
        disp_vsync_flag = 0U;
        disp_in_frame   = 0U;
        disp_fill       = 0U;
        (void)memset(disp_map, 0, sizeof(disp_map));
        disp_armed      = 1U;
        disp_pass      ++;
    }

    // ⚠️ 用 while 不是 if：主循环每圈可能已经过去好几行 ⇒ 一次把落下的全补齐
    while (disp_next_row != w)
    {
        // ⚠️ `disp_armed` 是**必须**的条件：没跨过帧首之前一律不拷、不计数
        //    （否则会拿上一轮残留的 disp_fill=120 立刻"完成" ⇒ 画面像死的）
        //
        //   ★★ 抽行步长 = `cam_dstep` = 一帧行数(cam_span) / 输出行数(CAM_H) ★★
        //      实测一帧 **240 行**、输出 120 行 ⇒ 步长 2（每 2 行取 1 行）。
        //      ⚠️ 这里以前写死 `& 3U` / `>> 2`（= 步长 4，假设一帧 480 行）
        //         ⇒ 一帧只走了一半就"数完 120 行" ⇒ **画面里同一场景出现两遍** ✓ 就是这个 bug。
        if ((0U == (disp_ln % (uint32_t)cam_dstep)) && (0U == disp_hold) && (0U != disp_armed))
        {
            /* ★★ 摄像头装反了（画面上下颠倒）⇒ 收割时翻转行号 ★★
               放在这里翻的好处：后面的 Otsu / 找边 / 路线识别的"近段=下方、远段=上方"假设
               全部自动成立，不用再改 analyze 和 classify_route ✓
               若以后把摄像头装正，把下一行注释掉即可回到原样 ✓ */
            uint32_t dst = (uint32_t)disp_in_frame;   // ★ 帧内行号（VSYNC 归零）⇒ 一趟正好一帧 ✓
            dst = (uint32_t)((uint32_t)CAM_H - 1U) - dst;   /* ← 上下翻转（装反时用）*/
            // ★★ 源行地址按**运行时的行宽**算：752 模式 = 752 字节/行；188 模式 = 188 字节/行。
            //    用"扁平指针 + 行宽乘法"而不是 `cam_image[row][col]`，因为数组的固定行距是 752，
            //    188 模式下 DMA 是**按 188 连续写**的 ⇒ 必须手动乘行宽才对得上 ✓
            const uint8_t *src = &cam_image[0][0] + (uint32_t)disp_next_row * (uint32_t)cam_row_bytes;
            uint32_t c;
            for (c = 0U; c < (uint32_t)CAM_TXW; c++)
            {
                // ★ 横向抽点 `cam_dcol`：752 模式取 c*4（752→188）；188 模式取 c（不抽）
                disp_img[dst][c] = src[c * (uint32_t)cam_dcol];
            }
            // ★★★ 用"120 位的位图"记"哪些 dst 行已经填过" —— **去重计数** ★★★
            //   🔥🔥 为什么必须这样（2026-10-15 的根因）：
            //     主循环是**突发补行**（一轮补 ~27 行），补行一旦**跨过 `disp_ln` 的回绕点**，
            //     旧的"max(dst+1)"计数就会被清零 ⇒ **永远数不到 120** ⇒ `disp_done` 永不置位
            //     ⇒ 现象：① 以前"有图"其实是靠 400ms 超时硬发出来的**混合帧**（横线/竖线都来自它）
            //              ② 把超时发送关掉之后 → **没图了** ✓（正是用户看到的现象）
            //   位图对"跨回绕的突发"免疫：哪一行填过就置哪一位，填过的**不再重复计数** ✓
            if (0U == (disp_map[(uint32_t)dst >> 3U] & (uint8_t)(1U << ((uint32_t)dst & 7U))))
            {
                disp_map[(uint32_t)dst >> 3U] |= (uint8_t)(1U << ((uint32_t)dst & 7U));
                if (disp_in_frame < (uint16_t)CAM_H) { disp_in_frame ++; }   // ★ 帧内行号推进
                if (disp_fill < (uint8_t)CAM_H) { disp_fill ++; }

                if (disp_fill >= (uint8_t)CAM_H)
                {
                    // ★★ **自己冻住** —— 填满 120 行的**这一瞬间**就冻，一点延迟都不能有：
                    //   如果等主循环看到 `disp_done` 再冻，中间那几百微秒里收割会继续往 dst 0.. 写
                    //   下一帧的内容 ⇒ 上面几行是新帧、下面是旧帧 ⇒ **又是一道缝** ✓
                    disp_hold  = 1U;
                    disp_done  = 1U;
                    disp_armed = 0U;
                }
            }
        }
        disp_next_row = (uint16_t)((disp_next_row + 1U) % (uint16_t)cam_ring_rows);
        disp_ln       = (disp_ln + 1U) % (uint32_t)cam_span;
        nlag ++;

        if (0U == disp_ln)
        {
            // ★★★ 帧边界只做一件事：**武装** ★★★
            //   ⚠️ **绝不再碰计数/位图**（那是"突发跨回绕丢进度"的根源）
            //   ⇒ 计数与位图**只由 `board_cam_new_pass()` 在相位机开始新一轮时清零** ✓
            disp_armed = 1U;
            disp_pass ++;                                  // （仅诊断：绕了几圈）
        }
    }

    // 🔑 滞后计数：一次收割补了几行。**≥30 就说明主循环跑得太慢、已经在漏行**
    //   （一条行只活 120 行 ≈ 4.2ms；主循环周期 T ⇒ 每次要补 T/36.5µs 行。
    //      T=1ms → 27 行；T=2ms → 55 行（漏！）；T=4.2ms → 115 行（几乎全漏））
    if (nlag > disp_lag_max) { disp_lag_max = nlag; }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读出"DMA 当前写到缓冲的第几行" —— 也就是**缓冲里最老的那一行**
// 返回参数     uint32   行号（0 ~ CAM_H-1）
// 备注信息     🔥 这才是"图上有一道线、线的上下像两张不同的图"的**正解**：
//   循环 DMA 的缓冲永远只装得下**最近 CAM_H 行**，而且它是**环形**的：
//     地址上从 0 读到末尾 ⇒ 读到的**不是时间顺序** —— 中间夹着一个"绕回点"
//       ⇒ 那道线就是绕回点；绕回点两边的时间差了整整 CAM_H 行 ⇒ 像两张不同的图。
//   ⇒ 从**当前写入位置**开始读 ⇒ 正好是"从最老到最新"的时间顺序 ✓
//      那道缝被推到**画面最底边**（最后一行），基本看不见。
//   ⚠️ 注意：它必须是"**读取那一刻**"的写入位置，**不能**用上一次 VSYNC 记的值
//      —— 缓冲每 4.4ms 就被整圈覆盖一次，隔一帧就过期了。
//-------------------------------------------------------------------------------------------------------------------
uint32_t board_cam_write_row (void)
{
    // ★ 用**运行时**的一行字节数（752 或 188）—— 模式切换后 DMA 长度也跟着变了
    uint32_t words = (uint32_t)((uint32_t)cam_row_bytes * CAM_H / 4U);
    uint32_t nd    = (uint32_t)(hdma_dcmi.Instance->NDTR & 0xFFFFU);

    if (nd > words) { return 0U; }                       // 保险：NDTR 不会大于重装值

    return (((words - nd) * 4U) / (uint32_t)cam_row_bytes) % (uint32_t)cam_ring_rows;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动采集（连续模式 + 循环 DMA）
// 返回参数     uint8   0 = 成功
// 备注信息     ⚠️ HAL 这个 Length 参数的单位是 **32 位字**，所以要 /4
//-------------------------------------------------------------------------------------------------------------------
uint8_t board_cam_start (void)
{
    // 连续模式 + 循环 DMA：DCMI 一直采、DMA 在缓冲里转圈；帧边界位置由上面的回调实时记录。
    // 🔥🔥 2026-10-15 修：**DMA 长度必须用"运行时行宽"**（`cam_row_bytes`），
    //    原来写死的是编译期的 `CAM_W`(=752) ⇒ 切到 188 宽模式后，
    //    DMA 仍然搬 752×120 那么多字节 ⇒ `board_cam_write_row()` 算出来的行号完全不对
    //    ⇒ 逐行收割一行都填不进去（现象：心跳里 `F=0` 恒不变）✗✗
    //    ⚠️ 教训：**同一个"宽度"概念在多个地方用到时，改成运行时量就必须**逐处**核对**，
    //       漏一处（这里就是漏了 DMA 长度）整个链路就静默失效。
    // ★ 整块缓冲作为 DMA 环：188×480 = 752×120 = 90240 字节（两模式通用）✓
    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,
                           (uint32_t)cam_image,
                           (uint32_t)(((uint32_t)CAM_W * (uint32_t)CAM_H) / 4U)) != HAL_OK)
    {
        cam_last_start_err = 1;
        return 1;
    }
    cam_last_start_err = 0;

    /* ★★★ 关键：把"收割行游标"同步到 DMA 当前位置 ★★★
       【文档依据】《总钻风摄像头使用说明 V2.7》9.3.3：
         "摄像头上的 51 有 eeprom…每次上电的时候加载 eeprom 的配置数据"
       我们这块**拆了 51** ⇒ 上电是出厂默认 752×480，要等固件用 SCCB 改完才变 188×120
       ⇒ 改完配置重启 DCMI 时，**环里还混着 752 宽的旧数据**，而收割游标还停在上一次的位置
       ⇒ 新旧数据拼在一起 ⇒ **横竖线**（拼到哪儿取决于上电时序 ⇒ "开机随机出现"）✓✓
       这里把游标直接对齐到"DMA 现在写的行"，旧数据一律不碰 ✓ */
    disp_next_row = (uint16_t)board_cam_write_row();
    disp_ln       = 0U;

    // ★ **每帧都要** VSYNC 中断（用来记录帧边界位置）—— 不 Stop、不重启，开销极小
    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_VSYNC);

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     量 PA6 上那根线的实际频率（kHz）—— 判断它到底是不是"像素时钟"
// 备注信息     🔑 方法：把 PA6 **临时**借给 TIM3_CH1，用 TIM3 的"外部时钟模式1"当硬件计数器，
//               量 20ms 再读计数 —— 全程硬件计数，不受主循环抖动影响。
//               量完立刻把 PA6 还给 DCMI（AF13），所以不影响后面的采集。
//   判读： PCLK 应该是 **MHz 级**（188×120@60fps 约 1.4MHz，满分辨率更高）
//          若读到 **约 60**    ⇒ 这根线上接的是 VSYNC
//          若读到 **约 7000**  ⇒ 接的是 HREF
//          若读到 **0**       ⇒ 这根线根本没在跳
//-------------------------------------------------------------------------------------------------------------------
volatile uint32_t dbg_pclk_khz = 0;

static void board_pclk_measure (void)
{
    GPIO_InitTypeDef gi = {0};
    uint32_t t0, last, total = 0;
    uint16_t c;

    __HAL_RCC_TIM3_CLK_ENABLE();

    // ---- 1. PA6 临时改成 TIM3_CH1（AF2）----
    gi.Pin       = GPIO_PIN_6;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gi);

    // ---- 2. TIM3 配成外部时钟模式1：TI1FP1 上升沿计数，1 分频 ----
    TIM3->CR1   = 0x0000;
    TIM3->PSC   = 0x0000;
    TIM3->ARR   = 0xFFFF;
    TIM3->CCMR1 = (TIM3->CCMR1 & ~0x000000FFUL) | 0x00000001UL;   // CC1S=01 → IC1 映射到 TI1
    TIM3->CCER  = 0x0001;                                         // CC1E=1, 上升沿
    TIM3->SMCR  = 0x0057;                                         // TS=101(TI1FP1) | SMS=111(外部时钟模式1)
    TIM3->EGR   = 0x0001;                                         // UG：产生更新事件
    TIM3->SR    = 0x0000;
    TIM3->CR1   = 0x0001;                                         // CEN：开始计数

    // ---- 3. 量 20ms（每读一次都算 16 位回绕安全的增量，避免漏计）----
    t0   = HAL_GetTick();
    last = TIM3->CNT;
    while ((HAL_GetTick() - t0) < 20U)
    {
        c      = (uint16_t)TIM3->CNT;
        total += (uint16_t)(c - (uint16_t)last);
        last   = c;
    }

    TIM3->CR1 &= ~0x0001UL;                                       // 停
    dbg_pclk_khz = total / 20UL;                                  // 20ms 的计数 = kHz

    // ---- 4. 把 PA6 还给 DCMI（后面 board_cam_init() 里还会再配一遍 AF13）----
    gi.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &gi);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对外接口：量一次 PA6 的频率并返回（kHz）—— **判断传感器还活着的金标准**
//   判读（列 bin4 生效时 PCLK 应为满分辨率时的 1/4）：
//     K≈0      ⇒ **传感器根本没输出像素时钟**（死了/停了）✗
//     K≈5300   ⇒ 满分辨率 21MHz ÷4 ⇒ **列 bin4 生效、传感器活着** ✓
//     K≈21000  ⇒ 还是满分辨率时钟 ⇒ **列 bin 没生效** ✗
//     K≈60     ⇒ 这根线上接的是 VSYNC（接线错位）✗
//   ⚠️ 会占用 PA6 约 20ms 并把它短暂改成 TIM3 复用 ⇒ **别在采集正常时频繁调** ✓
//-------------------------------------------------------------------------------------------------------------------
uint32_t board_cam_pclk_khz (void)
{
    board_pclk_measure();
    return dbg_pclk_khz;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 量 PB7（= DCMI 的 VSYNC 输入）上的**实际边沿频率**（Hz，20ms，借 TIM4_CH2）★
//   🔑 判读（这是"传感器到底成不成帧"的**直接证据**）：
//     V2≈50      ⇒ 传感器在正常出帧（50fps）✓ ⇒ 问题在 DCMI 配置/同步 ✗
//     V2≈0       ⇒ **传感器根本没发 VSYNC**（没在成帧）⇒ 传感器侧问题 ✓
//   ⚠️ 与 PA6 的量法同理：量完立刻把 PB7 还给 DCMI（AF13），别在采集正常时频繁调 ✓
//-------------------------------------------------------------------------------------------------------------------
static void board_vsync_measure (void)
{
    GPIO_InitTypeDef gi = {0};
    uint32_t t0, last, total = 0;
    uint16_t c;

    __HAL_RCC_TIM4_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_7;                    // PB7 = TIM4_CH2 (AF2) = DCMI VSYNC
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gi);

    TIM4->CR1   = 0x0000;
    TIM4->PSC   = 0x0000;
    TIM4->ARR   = 0xFFFF;
    TIM4->CCMR1 = (TIM4->CCMR1 & ~0x0000FF00UL) | 0x00000100UL;   // CC2S=01 → IC2 映射到 TI2
    TIM4->CCER  = 0x0010;                                         // CC2E=1, 上升沿
    TIM4->SMCR  = 0x0067;                                         // TS=110(TI2FP2) | SMS=111(外部时钟模式1)
    TIM4->EGR   = 0x0001;
    TIM4->SR    = 0x0000;
    TIM4->CR1   = 0x0001;

    t0   = HAL_GetTick();
    last = TIM4->CNT;
    while ((HAL_GetTick() - t0) < 20U)
    {
        c      = (uint16_t)TIM4->CNT;
        total += (uint16_t)(c - (uint16_t)last);
        last   = c;
    }

    TIM4->CR1 &= ~0x0001UL;
    dbg_vsync_khz = total * 50UL;                                 // 20ms 内的边数 ⇒ Hz（×50）✓ 原来的整除会把 1 条边沿吃成 0

    gi.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOB, &gi);
}

volatile uint32_t dbg_vsync_khz = 0;

uint32_t board_vsync_hz (void)
{
    board_vsync_measure();
    return dbg_vsync_khz * 1000UL;                                // 换算回 Hz
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头初始化（DMA + 两个中断 + 启动第一帧）
// 返回参数     uint8   0 = 成功
// 备注信息     ⚠️ 必须在 MX_DCMI_Init() 之后调用（DCMI 外设本身由 CubeMX 初始化好了）
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过 UART5(PC12=TX / PD2=RX) 给总钻风下发配置
// 备注信息     🔑 为什么必须做这一步：
//               1. 分辨率是**存在摄像头自己 EEPROM 里**的，出厂/没配过就不是 188×120
//                  （实测它现在吐的是全分辨率，PCLK 21.13MHz ⇒ 约 752×480，
//                   而 752×480=360960 字节，F407 的 128KB SRAM 一帧都装不下！）
//               2. 🔥🔥 手册第 20 页第 7 条：**"PCLK 消隐信号"用 STM32 的 DCMI 采集时必须设为 1**，
//                  而驱动里的默认值是 0 ⇒ 这一位不设，DCMI 采不到东西。
//               协议：4 字节/包 = 0xA5 | 命令 | 数据高8 | 数据低8，包间隔 ≥2ms；
//                     全部发完后摄像头重新初始化，完成后回 3 字节 0xA5 0xFF 0xFF。
//             ⚠️ 接线：摄像头 TXD → PD2(UART5_RX)，摄像头 RXD → PC12(UART5_TX)（交叉）。
//-------------------------------------------------------------------------------------------------------------------
volatile uint8_t  cam_cfg_state = 0;      // 0=还没做 1=成功 2=没收到应答(超时)
volatile uint32_t cam_cfg_ver   = 0;      // 摄像头固件版本（没读到就是 0）

#define CAMCFG_BAUD         (9600U)

// 波特率档位表 —— 手册只写了 9600，其余档是"手册没写全"踩怕了备着的；
//   2026-10-15 再加 4800 / 230400（"接了却不理我们"的一种常见原因就是波特率不符）
#define CAMCFG_BAUD_N       7U
static const uint32_t camcfg_bauds[CAMCFG_BAUD_N] = {9600U, 115200U, 57600U, 38400U, 19200U, 4800U, 230400U};

// UART5 配置串口：按档位打开 / 实时换波特率（UART5 挂在 APB1 上，用 PCLK1 算分频）
//   🔑 拆成"可按档位调用"是为了**实时调试**：上位机发 `b 0` 就切到 9600，发 `b 1` 就切 115200，
//      不用重编固件就能把"到底哪档能通"试一遍。
void cam_cfg_uart_open (uint8_t baud_idx)
{
    GPIO_InitTypeDef gi = {0};
    uint32_t baud, pclk1, div, mant, frac;

    if (baud_idx >= CAMCFG_BAUD_N) { baud_idx = 0U; }

    baud  = camcfg_bauds[baud_idx];
    pclk1 = HAL_RCC_GetPCLK1Freq();
    div   = (pclk1 * 25U) / (4U * baud);                   // 定点 ×100
    mant  = div / 100U;
    frac  = ((div - mant * 100U) * 16U + 50U) / 100U;

    // 引脚只配一次（重复配也无害，幂等）
    if (0U == (RCC->APB1ENR & RCC_APB1ENR_UART5EN))
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_UART5_CLK_ENABLE();

        gi.Mode      = GPIO_MODE_AF_PP;
        gi.Pull      = GPIO_PULLUP;
        gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gi.Alternate = GPIO_AF8_UART5;
        gi.Pin = GPIO_PIN_12;  HAL_GPIO_Init(GPIOC, &gi);      // PC12 = UART5_TX
        gi.Pin = GPIO_PIN_2;   HAL_GPIO_Init(GPIOD, &gi);      // PD2  = UART5_RX
    }

    UART5->CR1 = 0;
    UART5->BRR = (mant << 4) | (frac & 0x0FU);             // 42MHz/9600 → 0x1117
    UART5->CR1 = USART_CR1_UE;                             // ① 先使能（手册要求 UE 先于 TE/RE）
    UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;   // ② 再开发送/接收
}

// 老名字保留（内部用；现在的实现就是"按 9600 打开"）
static void camcfg_uart5_init (void) { cam_cfg_uart_open(0U); }

static void camcfg_putc (uint8_t c)
{
    while (0U == (UART5->SR & USART_SR_TXE)) { }
    UART5->DR = c;
}

// 发一个 4 字节配置包： A5 | cmd | 高8 | 低8
static void camcfg_send (uint8_t cmd, uint16_t data)
{
    camcfg_putc(0xA5);
    camcfg_putc(cmd);
    camcfg_putc((uint8_t)(data >> 8));
    camcfg_putc((uint8_t)data);
}

// 收一个字节，带超时(ms)；返回 -1 = 超时
static int camcfg_getc (uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < ms)
    {
        if (UART5->SR & USART_SR_RXNE) { return (int)(UART5->DR & 0xFFU); }
    }
    return -1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 单个 4 字节配置包（实时"手动"下发用）
// 参数说明     cmd = 命令（0x00 INIT / 0x02 曝光 / 0x03 帧率 / 0x04 列 / 0x05 行 / 0x09 PCLK模式 …）
//              val = 16 位数据
// 备注信息     🔑 上位机发 `c 3 10` 就能把"帧率"这一项单独改掉，然后立刻看图有没有变化 ——
//              这是把这个"从来不回话"的口试出来的唯一低风险路径。
//-------------------------------------------------------------------------------------------------------------------
void cam_cfg_send_pkt (uint8_t cmd, uint16_t val)
{
    camcfg_send(cmd, val);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 发 GET_VERSION(0xF2) 并收回原始应答（实时探活）
// 返回参数     uint32   收到的字节数（0 = 它一声不响）
// 备注信息     收到的原始字节存进 cam_cfg_ver（低 24 位），心跳里显示成 RV=xxxxxxxx：
//               RV=00000000 ⇒ 它根本没答（查接线/波特率/供电）；RV=A5xxxx ⇒ **它答了**，只是我们没解析对。
//-------------------------------------------------------------------------------------------------------------------
uint32_t cam_cfg_read_ver (void)
{
    uint32_t rx = 0, n = 0;
    uint8_t  i;
    int      c;

    while (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; }     // 把陈货清掉，别把旧字节当成应答
    camcfg_send(0xF2, 0x0000);                                 // GET_VERSION

    for (i = 0; i < 30U; i++)
    {
        c = camcfg_getc(20);
        if (c < 0) { break; }
        rx = (rx << 8) | (uint32_t)(uint8_t)c;
        n ++;
    }

    cam_cfg_ver   = rx & 0x00FFFFFFUL;
    cam_cfg_state = n ? 1U : 2U;                               // 1=它答了  2=没应答
    return n;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读配置串口两根线的**静态电平**（UART 空闲时两根都该是高）
// 返回参数     uint8   bit1 = PC12(TX，我们发出去的)   bit0 = PD2(RX，摄像头发回来的)
// 备注信息     🔑 判"到底哪一侧坏了"：
//                bit1=0 ⇒ 我们这侧的 TX 就是低的（先把 PC12 的焊接/复用查清楚）
//                bit0=0 ⇒ 摄像头那根 TX 是低的（查 PD2 的接线 / 摄像头有没有上电）
//-------------------------------------------------------------------------------------------------------------------
uint8_t cam_cfg_line_state (void)
{
    return (uint8_t)(((GPIOC->IDR & GPIO_PIN_12) ? 2U : 0U) |
                     ((GPIOD->IDR & GPIO_PIN_2)  ? 1U : 0U));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ 配置串口"探活" —— 一次把三件事分清 ★★★
//   背景：这口一直是 `CST=2 / RV=00000000`（一句都不回）。但"不回"有三种完全不同的原因，
//         必须分开，否则永远在瞎试：
//     ① **线根本没接**（摄像头 TXD → PD2 那根不存在/虚焊）    ⇒ 线上**一次电平变化都没有**
//     ② **线接对了，但波特率/格式不对**                     ⇒ 线上**有变化**，且 UART 会报
//                                                             **FE(帧错误)/NE(噪声)/ORE(溢出)**
//     ③ **它回话了**（我们的解析问题）                       ⇒ 收到字节数 > 0
//   🔑 怎么做到"不管波特率对不对，都能知道线在不在动"：
//      不依赖接收解析，直接**读 PD2 引脚电平**、数它的下降沿和低电平占比。
//      （PD2 在 UART 复位模式下 IDR 照样反映真实电平 ✓；我们内部 40k 上拉很弱，
//        摄像头 3.3V CMOS 输出一拉就拉得动 ⇒ 有回话就一定看得到边沿。）
//   ⚠️ 顺带修正一个我自己的错：原来拿 **0xF2** 当"GET_VERSION"去探活 ——
//      那个命令**文档里根本没有**（文档/示例里只有 INIT/EXP/FPS/SET_COL/SET_ROW/GAIN/PCLK/
//      GET_STATUS 这些，且**每个包都会回一个 `A5 FF FF`**）⇒ 拿一个无效命令去问，
//      摄像头当然当垃圾丢掉。现在改用**文档里确有的 `SET_COL`(0x04)**，值填**当前值**(752/188)，
//      属于"写了也不会改变现状"的安全命令 ✓
//   参数：baud_idx 0..4 = 9600/115200/57600/38400/19200
//   结果：都写进下面的全局量，由心跳/命令回显给上位机
//-------------------------------------------------------------------------------------------------------------------
volatile uint32_t cam_probe_edges = 0;   // 【阶段A】发完命令静听 60ms：PD2 的电平变化次数（0 ⇒ 线上没动静）
volatile uint32_t cam_probe_low   = 0;   // 【阶段A】PD2 低的采样次数
volatile uint32_t cam_probe_n     = 0;   // 【阶段A】总采样次数
volatile uint32_t cam_probe_rx    = 0;   // 【阶段A】收到几个字节（>0 ⇒ 它回话了）
volatile uint8_t  cam_probe_err   = 0;   // 【阶段A】bit0=FE  bit1=NE  bit2=ORE
// 【阶段B】一边**持续发 0x55**（产生方波）一边采样 —— 专门用来做"自己验自己"：
//   🔑 用一根杜邦线把 **PC12 和 PD2 直接短接**（把摄像头那两根线全拔掉），再发一次 `t`：
//        阶段B 的 EDGES 很大、RX>0  ⇒ **我们这侧的收发通路完全正常** ⇒ 问题必在摄像头那根线/摄像头
//        阶段B 仍然 EDGES=0        ⇒ **是我们自己这侧的问题**（引脚/复用/焊接），跟摄像头无关
volatile uint32_t cam_probe_b_edges = 0;
volatile uint32_t cam_probe_b_rx    = 0;
volatile uint32_t cam_probe_b_n     = 0;
volatile uint32_t cam_probe_b_tx_edges = 0;   // 【阶段B】**PC12（我们自己的 TX）**上的边沿数
                                              //   >0 ⇒ 我们的 TX 确实在驱动这个脚 ✓
// 【阶段C】PD2 "三态检测" —— 分辨"PD2 上到底有没有东西接着"（不依赖收发是否正常）
//   PU=1 且 PD=0 ⇒ **悬空**（跳线/摄像头那根线根本没接上，或这个脚没引出来）
//   PU=1 且 PD=1 ⇒ 有东西在**驱动高**（跳线接到 PC12、或摄像头 TX 空闲高）✓
//   PU=0 且 PD=0 ⇒ 被驱动**低**（或短路到 GND）
volatile uint8_t  cam_probe_c_np = 0;         // 无拉时读到的电平
volatile uint8_t  cam_probe_c_pu = 0;         // 上拉时
volatile uint8_t  cam_probe_c_pd = 0;         // 下拉时

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ PD2 "三态检测"（抽成独立函数，扫描和单档探活都用）★★★
//   把 PD2 临时改成普通输入，分别用 无拉 / 上拉 / 下拉 读电平：
//     PU=1 且 PD=0 ⇒ **PD2 悬空**（上面什么都没接 / 这个脚没引出来）
//     PU=1 且 PD=1 ⇒ 有东西在**驱动高**（跳线到 PC12、或摄像头 TXD 空闲高）✓
//     PU=0 且 PD=0 ⇒ 被驱动**低**（或短路到 GND）
//   ⚠️ 测完必须把 PD2 还给 UART5_RX（`cam_cfg_uart_open`）。
//-------------------------------------------------------------------------------------------------------------------
static void cam_pd2_tristate (void)
{
    GPIO_InitTypeDef gi = {0};

    gi.Pin   = GPIO_PIN_2;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pull = GPIO_NOPULL;   HAL_GPIO_Init(GPIOD, &gi);  HAL_Delay(1);
    cam_probe_c_np = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

    gi.Pull = GPIO_PULLUP;   HAL_GPIO_Init(GPIOD, &gi);  HAL_Delay(1);
    cam_probe_c_pu = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

    gi.Pull = GPIO_PULLDOWN; HAL_GPIO_Init(GPIOD, &gi);  HAL_Delay(1);
    cam_probe_c_pd = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

    cam_cfg_uart_open(0U);                               // 还原成 UART5_RX
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ 波特率扫描：每档都发一次 SET_COL，听 30ms，记下"收到几个字节 / 线动了几下" ★★★
//   为什么要它：`EDGES=0` 分不清下面两件完全不同的事 ——
//     ① **线没接**（摄像头 TXD 没到 PD2）           ⇒ 每一档都 0
//     ② **接上了但波特率不对**（它把我们的数据当垃圾丢掉）⇒ 也是"没有回话"，但换对档就有
//   一次扫完 7 档（9600/115200/57600/38400/19200/4800/230400）⇒ 一眼看出是哪种 ✓
//   扫描前先做 PD2 三态检测（判断线上到底有没有东西接着）。
//-------------------------------------------------------------------------------------------------------------------
volatile uint32_t cam_scan_rx[CAMCFG_BAUD_N] = {0};   // 每档收到的字节数（>0 ⇒ 这档通了）
volatile uint32_t cam_scan_eg[CAMCFG_BAUD_N] = {0};   // 每档 PD2 上的电平跳变次数

void cam_cfg_scan (void)
{
    uint8_t  i;
    uint32_t t0;
    uint8_t  last;

    cam_pd2_tristate();                                  // ① 先看"线上到底有没有东西"

    for (i = 0U; i < CAMCFG_BAUD_N; i++)
    {
        cam_scan_rx[i] = 0U;
        cam_scan_eg[i] = 0U;

        cam_cfg_uart_open(i);
        HAL_Delay(2);
        while (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; }

        camcfg_send(0x04, 752);                          // 文档里确有的 SET_COL（写了不改变现状）

        last = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;
        t0   = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 30UL)
        {
            uint8_t v = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

            if (v != last) { cam_scan_eg[i] ++; last = v; }
            if (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; cam_scan_rx[i] ++; }
        }
    }

    cam_cfg_uart_open(0U);                               // 回到 9600
}

//=================================================== 软件 I2C / SCCB ===================================================
//   🔥🔥 2026-10-15 的**关键发现**：用户的转接板上写着 **"TXD=SDA / RXD=SCL"**
//      ⇒ 那两根线接的是传感器的 **SCCB（I2C）**，**不是串口**！
//      ⇒ 说明这颗总钻风**没有那颗 51 配置芯片**（就是《拆除51单片机改I2C配置》那种形态）
//      ⇒ **它永远不会回应 UART** —— 所以三颗摄像头、所有波特率都是 0，一个字节都没有 ✓
//   ✅ 正解：**自己用 I2C(SCCB) 直接读写 MT9V034 的寄存器**（数据手册在
//      `Downloads\MT9V03X_Product-master\...\芯片手册\MT9V034-数据手册.PDF`）
//   🔑 接线**不用动**：你原来接的那两根（PD2 / PC12）本来就是 SDA/SCL ✓
//   ⚠️ 丝印的 "TXD=SDA / RXD=SCL" 不一定和我们这边的顺序对得上 ⇒
//      `cam_sccb_test()` 会**两种接法各试一遍**，哪个能读出芯片 ID 就是对的 ✓
//   ⚠️ SCCB 是**开漏**总线，必须有上拉（手册：SDATA 上升时间是按 1.5kΩ 上拉给的）。
//      先用内部 ~40k 上拉 + 很慢的时钟试；不行就外部加 4.7kΩ 到 3.3V。
//   📌 手册关键参数：从设备地址 **0x90(写)/0x91(读)**；寄存器 **16 位地址 + 16 位数据**；
//      SCLK ≤ **400kHz**；芯片版本寄存器 R0x0000 应为 **0x1313**（MT9V034）。
//=====================================================================================================================
static GPIO_TypeDef *sccb_sda_port = GPIOD;
static uint16_t      sccb_sda_pin  = GPIO_PIN_2;
static GPIO_TypeDef *sccb_scl_port = GPIOC;
static uint16_t      sccb_scl_pin  = GPIO_PIN_12;

static void sccb_dly (void)
{
    volatile uint32_t i;

    // 🔥 2026-10-15 改慢：原来 60 次 ≈ **250kHz**，而手册的上升时间参数是按 **1.5kΩ 上拉**给的，
    //   我们现在只有**内部约 40kΩ**上拉 ⇒ 边沿根本来不及拉高 ⇒ 器件地址收不到 ACK（err=1）。
    //   现在半周期 ≈ 600 次空转 ≈ **10µs 以上（≈40kHz 以下）**，给弱上拉足够时间 ✓
    for (i = 0U; i < 600U; i ++) { }
}

// 前置声明（下面两个 helper 用到了这些原语，但它们的定义在后面）
static void    sccb_sda_hi (void);
static void    sccb_sda_lo (void);
static void    sccb_scl_hi (void);
static void    sccb_scl_lo (void);
static uint8_t sccb_sda_rd (void);
static void    sccb_stop   (void);

// 读两根线的**静态电平**：bit1 = SDA，bit0 = SCL（1 = 高）
//   🔑 判读：两根都高 ⇒ 总线空闲（有上拉）✓；
//            某根恒低 ⇒ 被拽住/短路，或者**没有上拉**（此时谁都拉不高）
static uint8_t sccb_levels (void)
{
    return (uint8_t)((sccb_sda_rd() ? 2U : 0U) |
                     ((HAL_GPIO_ReadPin(sccb_scl_port, sccb_scl_pin) == GPIO_PIN_SET) ? 1U : 0U));
}

// 总线复位：SCL 敲 9 个脉冲，把卡在中途的从机放掉（标准 I2C 恢复手法）
static void sccb_bus_recover (void)
{
    uint8_t i;

    sccb_sda_hi();
    for (i = 0U; i < 9U; i++)
    {
        sccb_scl_lo(); sccb_dly();
        sccb_scl_hi(); sccb_dly();
    }
    sccb_stop();
}

static void sccb_sda_hi (void) { HAL_GPIO_WritePin(sccb_sda_port, sccb_sda_pin, GPIO_PIN_SET);   }
static void sccb_sda_lo (void) { HAL_GPIO_WritePin(sccb_sda_port, sccb_sda_pin, GPIO_PIN_RESET); }
static void sccb_scl_hi (void) { HAL_GPIO_WritePin(sccb_scl_port, sccb_scl_pin, GPIO_PIN_SET);   }
static void sccb_scl_lo (void) { HAL_GPIO_WritePin(sccb_scl_port, sccb_scl_pin, GPIO_PIN_RESET); }
static uint8_t sccb_sda_rd (void) { return (sccb_sda_port->IDR & sccb_sda_pin) ? 1U : 0U; }

// swap: 0 ⇒ PD2=SDA / PC12=SCL；1 ⇒ PD2=SCL / PC12=SDA
void cam_sccb_init (uint8_t swap)
{
    GPIO_InitTypeDef gi = {0};

    if (0U == swap)
    {
        sccb_sda_port = GPIOD; sccb_sda_pin = GPIO_PIN_2;
        sccb_scl_port = GPIOC; sccb_scl_pin = GPIO_PIN_12;
    }
    else
    {
        sccb_sda_port = GPIOC; sccb_sda_pin = GPIO_PIN_12;
        sccb_scl_port = GPIOD; sccb_scl_pin = GPIO_PIN_2;
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_OUTPUT_OD;              // ★ 开漏（SCCB/I2C 总线要求）
    gi.Pull  = GPIO_PULLUP;                      // 内部弱上拉（不够就外接 4.7kΩ）
    gi.Speed = GPIO_SPEED_FREQ_HIGH;

    gi.Pin = sccb_sda_pin; HAL_GPIO_Init(sccb_sda_port, &gi);
    gi.Pin = sccb_scl_pin; HAL_GPIO_Init(sccb_scl_port, &gi);

    sccb_sda_hi();
    sccb_scl_hi();
    HAL_Delay(1);
}

static void sccb_start (void)
{
    sccb_sda_hi(); sccb_scl_hi(); sccb_dly();
    sccb_sda_lo(); sccb_dly();
    sccb_scl_lo(); sccb_dly();
}

static void sccb_stop (void)
{
    sccb_sda_lo(); sccb_scl_hi(); sccb_dly();
    sccb_sda_hi(); sccb_dly();
}

static uint8_t sccb_wr_byte (uint8_t b)          // 返回 0 = 收到 ACK（有器件应答）
{
    uint8_t i, ack;

    for (i = 0U; i < 8U; i++)
    {
        if (b & 0x80U) { sccb_sda_hi(); } else { sccb_sda_lo(); }
        b = (uint8_t)(b << 1);
        sccb_dly(); sccb_scl_hi(); sccb_dly(); sccb_scl_lo(); sccb_dly();
    }

    sccb_sda_hi();                                // 释放总线读 ACK
    sccb_dly(); sccb_scl_hi(); sccb_dly();
    ack = sccb_sda_rd();
    sccb_scl_lo(); sccb_dly();

    return ack;
}

static uint8_t sccb_rd_byte (uint8_t nack)        // nack=1 ⇒ 最后一字节，回 NACK
{
    uint8_t i, b = 0U;

    sccb_sda_hi();
    for (i = 0U; i < 8U; i++)
    {
        sccb_dly(); sccb_scl_hi(); sccb_dly();
        b = (uint8_t)((b << 1) | (sccb_sda_rd() ? 1U : 0U));
        sccb_scl_lo();
    }

    if (nack) { sccb_sda_hi(); } else { sccb_sda_lo(); }
    sccb_dly(); sccb_scl_hi(); sccb_dly(); sccb_scl_lo(); sccb_dly();
    sccb_sda_hi();

    return b;
}

//-------------------------------------------------------------------------------------------------------------------
// 🔥🔥🔥 2026-10-15 **读手册后抓到的两处硬错**（这就是"读文档"的价值）：
//   ① **从机地址有四个**，不是两个！手册原文：
//      "The MT9V034 is a serial interface slave with **four possible IDs (0x90, 0x98, 0xB0 and 0xB8)**
//       determined by the S_CTRL_ADR0 and S_CTRL_ADR1 input pins."
//      ⇒ 我之前只试了 0x90 和一个**手册里根本没有的 0xBA** ✗（手册示例图用的正是 **0xB8**）
//   ② **寄存器地址是 8 位（1 个字节）**，数据才是 16 位：
//      "the master then transfers the **8-bit register address** … The MT9V034 uses **16-bit data**
//       for its internal registers, thus requiring two 8-bit transfers"
//      ⇒ 我一直发 **16 位寄存器地址（两个字节）** ✗✗ ⇒ 从机根本不认这个包，连 ACK 都不会给 ✓
//   ✅ 正确序列（手册 Figure 9/10）：
//      写：START + 从机地址(W) + ACK + **寄存器地址(1B)** + ACK + 数据高 + ACK + 数据低 + ACK + STOP
//      读：START + 从机地址(W) + 寄存器地址(1B) + STOP → START + 从机地址(R) + 数据高 + 数据低 + NACK + STOP
//   期望：芯片版本寄存器 **R0x00 = 0x1313**（MT9V034）
//-------------------------------------------------------------------------------------------------------------------
static const uint8_t sccb_ids[4] = {0x90U, 0x98U, 0xB0U, 0xB8U};   // ★ 手册给的四个可能地址
static uint8_t sccb_dev_w = 0x90U;                                 // 当前使用的从机地址（写）

// 读一个 8 位地址的寄存器（返回 0 = 成功）
static uint8_t sccb_read_reg8 (uint8_t dev_w, uint8_t reg, uint16_t *val)
{
    uint8_t hi, lo;

    sccb_start();
    if (sccb_wr_byte(dev_w))                  { sccb_stop(); return 1U; }
    if (sccb_wr_byte(reg))                    { sccb_stop(); return 2U; }   // ★ 1 字节寄存器地址
    sccb_stop();

    sccb_start();
    if (sccb_wr_byte((uint8_t)(dev_w | 1U)))  { sccb_stop(); return 3U; }
    hi = sccb_rd_byte(0U);
    lo = sccb_rd_byte(1U);
    sccb_stop();

    *val = ((uint16_t)hi << 8) | (uint16_t)lo;
    return 0U;
}

// 读一个 16 位寄存器（返回 0 = 成功；非 0 = 在第几步失败）
static uint8_t sccb_read_reg (uint16_t reg, uint16_t *val)
{
    if (reg > 0xFFU) { return 9U; }            // 本协议只支持 8 位寄存器地址
    return sccb_read_reg8(sccb_dev_w, (uint8_t)reg, val);
}

// 对外公开的读（`j <n>` 命令用）
uint8_t cam_sccb_read_reg (uint16_t reg, uint16_t *val)
{
    return sccb_read_reg(reg, val);
}

// 写一个寄存器（返回 0 = 成功）—— 序列：addr(W) + reg(1B) + data(2B)
uint8_t cam_sccb_write_reg (uint16_t reg, uint16_t val)
{
    if (reg > 0xFFU) { return 9U; }

    sccb_start();
    if (sccb_wr_byte(sccb_dev_w))             { sccb_stop(); return 1U; }
    if (sccb_wr_byte((uint8_t)reg))           { sccb_stop(); return 2U; }   // ★ 1 字节
    if (sccb_wr_byte((uint8_t)(val >> 8)))    { sccb_stop(); return 4U; }
    if (sccb_wr_byte((uint8_t)(val & 0xFFU))) { sccb_stop(); return 5U; }
    sccb_stop();
    HAL_Delay(1);
    return 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ SCCB 探活：两种接法各读一次芯片版本寄存器 R0x0000 ★★★
//   返回 0 = 两种接法都没通；1 = 接法0 通；2 = 接法1 通
//   期望值：MT9V034 = **0x1313**（高字节 0x13）
//-------------------------------------------------------------------------------------------------------------------
volatile uint16_t cam_sccb_id0 = 0U;      // 接法0（PD2=SDA / PC12=SCL）+ 地址 0x90 读到的 ID
volatile uint16_t cam_sccb_id1 = 0U;      // 接法1（PD2=SCL / PC12=SDA）+ 地址 0x90 读到的 ID
volatile uint16_t cam_sccb_ba  = 0U;      // 地址换成 0xBA 时读到的（部分模组是这个）
volatile uint8_t  cam_sccb_lv0 = 0U;      // 接法0 下两根线的静态电平：bit1=SDA bit0=SCL
volatile uint8_t  cam_sccb_lv1 = 0U;      // 接法1 下同上
volatile uint8_t  cam_sccb_err = 0U;      // 上一次读寄存器卡在第几步（1 = 器件地址没 ACK）
volatile uint8_t  cam_sccb_addr = 0U;     // ★ 探测成功时用的是哪个从机地址（0x90/0x98/0xB0/0xB8）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ 对 PC12 / PD2 做"三态检测"：**这个脚上到底有没有东西接着？** ★★★
//   做法：把脚临时改成普通输入，先给上拉、再给下拉，各读一次：
//     **PU=1 且 PD=0** ⇒ 只有我们自己的弱上拉能拉高、一下拉就低
//                        ⇒ **这个脚上什么都没接**（线没通 / 脚没引出 / FFC 没送到）✗
//     PU=1 且 PD=1 ⇒ 有**外部上拉或驱动器**把它顶着 ⇒ **确实接着东西** ✓
//     PU=0 且 PD=0 ⇒ 被拽低（接了 GND / 被驱动低）✓
//   🔑 为什么需要它：`LV=3`（读到高）既可能是"空脚被内部上拉拉高"，也可能是"真接着"，
//      光看高电平分不出来；**一给下拉就露馅** ✓
//-------------------------------------------------------------------------------------------------------------------
volatile uint8_t cam_pin_c12_pu = 0U, cam_pin_c12_pd = 0U;   // PC12 上拉/下拉时的读数
volatile uint8_t cam_pin_d2_pu  = 0U, cam_pin_d2_pd  = 0U;   // PD2  上拉/下拉时的读数

static void pin_tristate (GPIO_TypeDef *port, uint16_t pin, volatile uint8_t *pu, volatile uint8_t *pd)
{
    GPIO_InitTypeDef gi = {0};

    gi.Pin   = pin;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pull = GPIO_PULLUP;   HAL_GPIO_Init(port, &gi); HAL_Delay(1);
    *pu = (port->IDR & pin) ? 1U : 0U;

    gi.Pull = GPIO_PULLDOWN; HAL_GPIO_Init(port, &gi); HAL_Delay(1);
    *pd = (port->IDR & pin) ? 1U : 0U;
}

void cam_pin_tristate_all (void)
{
    pin_tristate(GPIOC, GPIO_PIN_12, &cam_pin_c12_pu, &cam_pin_c12_pd);
    pin_tristate(GPIOD, GPIO_PIN_2,  &cam_pin_d2_pu,  &cam_pin_d2_pd);

    cam_sccb_init(0U);                       // 还原成 SCCB（开漏输出 + 上拉）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ 把传感器配成 **188×120**（列 bin4 + 行 bin4）★★★
//   手册依据（P29 寄存器表 / P64「Pixel Binning」）：
//     · **R0x0D[3:2] = Column Bin**：0=正常 / 1=列bin2（宽度÷2）/ **2=列bin4（宽度÷4）**
//     · **R0x0E[1:0] = Row Bin**：0=正常 / 1=行bin2（行数÷2，帧率×2）/ **2=行bin4（行数÷4，帧率×4）**
//     · 原文："MT9V034 also provides the ability to **down-sample the entire image** … 行/列 binning 分别选"
//   ⇒ 752×480 经过 列bin4 + 行bin4 = **188×120** ✓（**不需要动窗口寄存器**）
//   🔑 为什么最想要这个：一帧 188×120 = **22560 字节**，**正好等于我们 DMA 缓冲的大小**
//      ⇒ 缓冲天然就是一整帧 ⇒ **不需要环形缓冲、不需要抽行、不需要那套相位机** ✓✓
//   ⚠️ 用"读-改-写"而不是直接写：保留该寄存器里其它位的出厂设置（板子可能不是全 0 默认）✓
//   ⚠️ 寄存器改动是**易失**的（传感器掉电即恢复）⇒ 复位/掉电后再自动配一次即可 ✓
//-------------------------------------------------------------------------------------------------------------------
volatile uint8_t  cam_bin_ok = 0U;        // 1 = 配置成功且回读一致
volatile uint16_t cam_r0d    = 0U;        // R0x0D 回读值（含列 bin 位）
volatile uint16_t cam_r0e    = 0U;        // R0x0E 回读值（含行 bin 位）

uint8_t cam_sccb_set_bin4 (void)
{
    uint16_t v   = 0U;
    uint8_t  err = 0U;

    // 🔥🔥🔥 2026-10-20 重大更正：**R0x0D 是 Context A 的 Read Mode，R0x0E 是 Context B 的！**
    //   （手册 P29 原文：`0x0E (14)  Read Mode Context B`）
    //   传感器默认跑在 **Context A** ⇒ **行 bin 也必须写在 R0x0D 里**！
    //   我原来把行 bin 写到 R0x0E（Context B）⇒ **根本没生效** ✗
    //   ⇒ 当时传感器实际输出 = 188 列 × **480 行**（列 bin4 生效、行 bin 没生效）✗
    //      —— 这就是"DCMI 看到的行数和固件假设的 120 对不上"的深层原因之一 ✓
    //   ✅ 正确写法：**R0x0D[3:2] = 2（列 bin4）且 R0x0D[1:0] = 2（行 bin4）**，两个都在 R0x0D ✓
    //      （0b1010 = 0x0A；保留位 [9:8]=3 用读-改-写保住 ✓）
    if (0U == cam_sccb_read_reg(0x0DU, &v))
    {
        // ★★ 2026-10-20 实测定论：行 bin4（[1:0]=2）在这颗芯片上**无法成帧**（CAM1：PF 有时钟、RISR=0 永久为 0）✗
        //    而**只做列 bin4**（[3:2]=2，[1:0]=0 → 输出 188×480）曾经出过几千张完整帧 ✓✓
        //    ⇒ 放弃行 bin4，固件改为 4:1 纵向抽行（见 board_cam_set_mode / set_span(480)）✓
        // ★★ 实测定论（图传出图 2400 行全好 ✓）：**列 bin4 + 行 bin4 都要**，
        //    但必须配 **快门 R0x0B ≤ 120 行**（行 bin4 后一帧只有 120 行，
        //    出厂默认曝光 443 行 > 120 → 永远完不成帧 —— 那才是当时“行 bin4 不成帧”的真凶手 ✓）
        //    掩码 ~0x0F：[3:2]=2（列 bin4）且 [1:0]=2（行 bin4）⇒ 0x0A ✓
        v = (uint16_t)((v & (uint16_t)~0x000FU) | 0x000AU);
        (void)cam_sccb_write_reg(0x0DU, v);
    }
    else { err = 1U; }

    // 🔥🔥🔥 2026-10-20 **第二个致命参数：曝光（R0x0B Total Shutter Width）**
    //   R0x0B 的单位是"行"，**出厂默认 443 行**：
    //     · 188×480 时代：443 < 480 ⇒ 能成帧 ⇒（那时有图 ✓）
    //     · **行 bin4 之后：一帧只剩 120 行 ⇒ 曝光 443 行 > 一帧 ⇒ 传感器永远完不成一帧**
    //       ⇒ 症状（CAM1 实测）：**PIXCLK 连续在跑（PF=1134 ✓）但 VSYNC/HREF 一个边沿都没有
    //         （SR=0000 / RIS=0000）** ✗✗ —— DCMI 这边怎么重启都没用，因为根本没帧可同步 ✓
    //   ⇒ 必须把快门压到帧内（120 行以内，取 100 行 ≈ 3.7ms 曝光）✓
    //   （想更亮/更暗：后续可以做成命令，动态改 R0x0B ✓）
    if (0U == cam_sccb_read_reg(0x0BU, &v))
    {
        (void)cam_sccb_write_reg(0x0BU, cam_shutter);   // ★ 阈值之外的亮度由曝光决定；150 ≈ 5.5ms（可 `e <n>` 实时调）
    }
    else { err = 1U; }

    HAL_Delay(10);

    // ★ 回读验证（只看 R0x0D：[3:2] 必须是 2，[1:0] 必须是 2）
    if (0U != cam_sccb_read_reg(0x0DU, &v)) { err = 1U; }
    cam_r0d = v;
    cam_r0e = 0U;                       // （R0x0E 是 Context B，不再使用）
    if (0U != cam_sccb_read_reg(0x0BU, &v)) { err = 1U; }
    cam_r0e = v;                        // ★ 复用 R0E 字段报"快门回读值"（正常应为 100=0x64）

    if ((0U == err) && ((cam_r0d & 0x000CU) == 0x0008U) && ((cam_r0d & 0x0003U) == 0x0002U))
    {
        cam_bin_ok = 1U;
    }
    else
    {
        cam_bin_ok = 0U;
    }

    return cam_bin_ok;
}

volatile uint8_t cam_shutter = 150U;  // ★ 曝光 R0x0B（行数）：出厂 443 在行bin4 下卡死成帧；150≈5.5ms
volatile uint8_t cam_flip = 0U;       // 0=不翻 1=左右镜像(ColumnFlip) 2=上下(RowFlip) 3=两者(180°)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 按 `cam_flip` 重写 R0x0D 的翻转位（bit5=Column Flip 左右镜像 / bit4=Row Flip 上下）★
//   🔑 用"读-改-写"只动 bit4/bit5，**binning 位（[3:2]/[1:0]）与保留位原样保留** ✓
//   传感器在下一帧就生效（这两个位是 shadowed 的 ✓）⇒ 按一次 `n` 最多等一帧就看到方向变化 ✓
//-------------------------------------------------------------------------------------------------------------------
void cam_sccb_apply_flip (void)
{
    uint16_t v = 0U;
    uint16_t f = (uint16_t)(((cam_flip & 1U) ? 0x0020U : 0U) |   // bit5 Column Flip（左右镜像）
                            ((cam_flip & 2U) ? 0x0010U : 0U));   // bit4 Row Flip（上下）

    if (0U == cam_sccb_read_reg(0x0DU, &v))
    {
        v = (uint16_t)((v & (uint16_t)~0x0030U) | f);
        (void)cam_sccb_write_reg(0x0DU, v);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ 全部在单片机内完成：Otsu 自动阈值 → 二值化 → 每行找“亮/暗交界列” ★★★
//   在相位机“采齐一帧”后调用（每帧一次，22560 像素 ≈1ms ✓）：
//     ① 256 级直方图 → Otsu 类间方差最大求阈值（自动适应现场明暗 ✓）
//     ② disp_img 原地二值化（≥阈值=255 白 / 否则 0 黑）—— 发出去的就是二值图 ✓
//     ③ 每行从左向右找第一个跳变列，要求其后连续 2 像素同向（防单点噪声 ✓）
//        ⇒ cam_bound[行] = 交界列号（0xFF = 该行无交界）；上位机只报数据，蓝线由上位机画 ✓
//-------------------------------------------------------------------------------------------------------------------
volatile uint8_t  cam_bound[120];     // 每行的亮/暗交界列（0xFF = 无）
volatile uint8_t  cam_thresh = 0;     // 本帧 Otsu 阈值（诊断用）

//-------------------------------------------------------------------------------------------------------------------
// ★★★ 帧率画进图像（**单片机自己画**，电脑只负责收）★★★
//   5x7 点阵字模（列优先：每字节低 7 位 = 一列的 7 个像素，bit0 在上）
//   只放 0-9 . F P S 和空格 ⇒ 共 15 个字模 = 75 字节，完全不占地方 ✓
//   画法：先铺白底框（二值图里最高对比），再用黑像素画字 ✓
//-------------------------------------------------------------------------------------------------------------------
static const uint8_t fps_font[17][5] =
{
    {0x3E,0x51,0x49,0x45,0x3E},   /* 0 */
    {0x00,0x42,0x7F,0x40,0x00},   /* 1 */
    {0x42,0x61,0x51,0x49,0x46},   /* 2 */
    {0x21,0x41,0x45,0x4B,0x31},   /* 3 */
    {0x18,0x14,0x12,0x7F,0x10},   /* 4 */
    {0x27,0x45,0x45,0x45,0x39},   /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30},   /* 6 */
    {0x01,0x71,0x09,0x05,0x03},   /* 7 */
    {0x36,0x49,0x49,0x49,0x36},   /* 8 */
    {0x06,0x49,0x49,0x29,0x1E},   /* 9 */
    {0x00,0x60,0x60,0x00,0x00},   /* . */
    {0x7F,0x09,0x09,0x09,0x01},   /* F */
    {0x7F,0x09,0x09,0x09,0x06},   /* P */
    {0x26,0x49,0x49,0x49,0x32},   /* S */
    {0x00,0x00,0x00,0x00,0x00},   /* 空格 */
    {0x7F,0x09,0x19,0x29,0x46},   /* R ★路线识别用 */
    {0x00,0x00,0x08,0x00,0x00},   /* - ★「目前-未来」分隔 */
};

#define FPS_TXT_SCALE   1U      /* 1 = 小字（不挡视野）；想更大改成 2 */

static uint8_t fps_glyph_idx (char c)
{
    if ((c >= '0') && (c <= '9')) { return (uint8_t)(c - '0'); }
    if (c == '.')                 { return 10U; }
    if (c == 'F')                 { return 11U; }
    if (c == 'P')                 { return 12U; }
    if (c == 'S')                 { return 13U; }
    if (c == 'R')                 { return 15U; }
    if (c == '-')                 { return 16U; }
    return 14U;
}

static void board_cam_draw_fps_text (uint16_t fps10)
{
    char     txt[12];
    uint8_t  i, n;
    uint16_t x0 = 2U, y0 = 2U, sc = FPS_TXT_SCALE;
    uint16_t tw, th, gx, gy, cx, cy;

    (void)sprintf(txt, "FPS %u.%u", (unsigned)(fps10 / 10U), (unsigned)(fps10 % 10U));
    n  = (uint8_t)strlen(txt);
    tw = (uint16_t)((uint16_t)n * 6U * sc);
    th = (uint16_t)(7U * sc);

    for (gy = 0U; gy < (uint16_t)(th + 2U * sc); gy++)                /* 白底框 */
    {
        for (gx = 0U; gx < (uint16_t)(tw + 2U * sc); gx++)
        {
            if (((y0 + gy) < (uint16_t)CAM_H) && ((x0 + gx) < (uint16_t)CAM_TXW))
            {
                disp_img[y0 + gy][x0 + gx] = 255U;
            }
        }
    }

    for (i = 0U; i < n; i++)                                          /* 黑字 */
    {
        uint8_t gi = fps_glyph_idx(txt[i]);
        for (cx = 0U; cx < 5U; cx++)
        {
            uint8_t col = fps_font[gi][cx];
            for (cy = 0U; cy < 7U; cy++)
            {
                if (0U != (col & (uint8_t)(1U << cy)))
                {
                    uint16_t sy, sx;
                    for (sy = 0U; sy < sc; sy++)
                    {
                        for (sx = 0U; sx < sc; sx++)
                        {
                            uint16_t px = (uint16_t)(x0 + sc + ((uint16_t)i * 6U + cx) * sc + sx);
                            uint16_t py = (uint16_t)(y0 + sc + cy * sc + sy);
                            if ((px < (uint16_t)CAM_TXW) && (py < (uint16_t)CAM_H))
                            {
                                disp_img[py][px] = 0U;
                            }
                        }
                    }
                }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// ★★★ 路线识别：用左边界在近/中/远三段行上的位置趋势判弯向 ★★★
//   0 = 直线   1 = 左转   2 = 右转   3 = 丢线/十字
//   原理：远段（图上部）对应“车前方远处”，边界往左偏 ⇒ 轨道向左弯 ✓
//-------------------------------------------------------------------------------------------------------------------
volatile uint8_t cam_now   = 0U;   /* 目前走哪 0/1/2/3 */
volatile uint8_t cam_ahead = 0U;   /* 未来走哪 0/1/2/3 */
volatile uint8_t cam_route = 0;          // ★ 当前路线类型（0..3）

/* ★ 消失点所在行（地平线）—— 透视基准用，现场可微调 */
#define LF_HORIZON_ROW   56

/* ★★★ 带透视补偿的弯向判定 ★★★
   ⚠️ 以前直接用"远段 x − 近段 x"，但**直道上这个差值也不为 0**（所有平行线都向消失点收敛）⇒ 直道也会被判成弯道 ✗
   ✅ 正确：先算"如果是直道，远段应在哪"（近段指向消失点的直线），
           实际值减去这个预期值，才是纯粹的弯曲量 ✓ */
static uint8_t lf_route_of (int32_t r_near, int32_t r_far)
{
    int32_t r, sN = 0, sF = 0;
    uint8_t cN = 0U, cF = 0U;
    int32_t x1, x2, x_exp, dev;

    for (r = r_near - 5; r <= r_near + 4; r++) { if (0xFFU != cam_bound[r]) { sN += (int32_t)cam_bound[r]; cN ++; } }
    for (r = r_far  - 5; r <= r_far  + 4; r++) { if (0xFFU != cam_bound[r]) { sF += (int32_t)cam_bound[r]; cF ++; } }

    if ((cN < 3U) || (cF < 3U)) { return 3U; }

    x1 = sN / (int32_t)cN;
    x2 = sF / (int32_t)cF;

    /* 直道基准：x(r) = x1 + (CENTER - x1) * (r_near - r) / (r_near - LF_HORIZON_ROW) */
    {
        int32_t den = r_near - LF_HORIZON_ROW;
        if (den < 8) { den = 8; }
        x_exp = x1 + (((int32_t)(CAM_TXW / 2) - x1) * (r_near - r_far)) / den;
    }

    dev = x2 - x_exp;                    /* 往左偏 ⇒ 负 ⇒ 左转 ✓ */

    if (dev < -6) { return 1U; }
    if (dev >  6) { return 2U; }
    return 0U;
}

/* ★★★ v3：「目前」看近→中，「未来」看中→远（都带透视补偿）★★★ */
void board_cam_classify_route (void)
{
    cam_now   = lf_route_of(110,  92);
    cam_ahead = lf_route_of( 92,  67);
    cam_route = cam_ahead;
}

/* ★★★ v2：「目前走哪」+「未来走哪」 ★★★
   三个行带：近 106~115 / 中 88~97 / 远 62~71
   目前 = 中−近：未来 = 远−中
   → 两者**共用中带** ⇒ 目前的“远”恰好是未来的“近”
     ⇒ 车往前开时，拐弯从「未来」自然滑进「目前」，**不存在切换时机** ✓ */

// 在左上角画 "R<code>"（紧跟在 FPS 下面一行）
static void board_cam_draw_route_text (void)
{
    char     txt[8];
    uint8_t  i, n;
    uint16_t x0 = 2U, y0 = 12U, sc = FPS_TXT_SCALE;
    uint16_t gx, gy, cx, cy;

    (void)sprintf(txt, "R%u-%u", (unsigned)cam_now, (unsigned)cam_ahead);
    n = (uint8_t)strlen(txt);

    for (gy = 0U; gy < (uint16_t)(7U * sc + 2U * sc); gy++)
        for (gx = 0U; gx < (uint16_t)((uint16_t)n * 6U * sc + 2U * sc); gx++)
            if (((y0 + gy) < (uint16_t)CAM_H) && ((x0 + gx) < (uint16_t)CAM_TXW))
                disp_img[y0 + gy][x0 + gx] = 255U;

    for (i = 0U; i < n; i++)
    {
        uint8_t gi = fps_glyph_idx(txt[i]);
        for (cx = 0U; cx < 5U; cx++)
        {
            uint8_t col = fps_font[gi][cx];
            for (cy = 0U; cy < 7U; cy++)
                if (0U != (col & (uint8_t)(1U << cy)))
                {
                    uint16_t sy, sx;
                    for (sy = 0U; sy < sc; sy++)
                        for (sx = 0U; sx < sc; sx++)
                        {
                            uint16_t px = (uint16_t)(x0 + 1U + ((uint16_t)i * 6U + cx) * sc + sx);
                            uint16_t py = (uint16_t)(y0 + 1U + cy * sc + sy);
                            if ((px < (uint16_t)CAM_TXW) && (py < (uint16_t)CAM_H))
                                disp_img[py][px] = 0U;
                        }
                }
        }
    }
}

void board_cam_analyze (void)
{
    static uint32_t hist[256];
    uint8_t *base = &disp_img[0][0];
    uint32_t total = (uint32_t)CAM_H * (uint32_t)CAM_TXW;
    uint32_t i, k, r, c;
    uint32_t sum_all = 0, sum_b = 0, w_b = 0;
    float    best = -1.0f;
    uint8_t  T = 128;

    for (i = 0; i < 256U; i++) { hist[i] = 0U; }
    for (i = 0; i < total; i++)      { hist[base[i]] ++; }
    for (i = 0; i < 256U; i++)       { sum_all += i * hist[i]; }

    for (k = 0; k < 256U; k++)       // ★ Otsu：类间方差最大者胜
    {
        uint32_t w_f;
        float mu_b, mu_f, d;
        w_b += hist[k];
        if (0U == w_b)   { continue; }
        w_f = total - w_b;
        if (0U == w_f)   { break; }
        sum_b += k * hist[k];
        mu_b = (float)sum_b / (float)w_b;
        mu_f = (float)(sum_all - sum_b) / (float)w_f;
        d    = (float)w_b * (float)w_f * (mu_b - mu_f) * (mu_b - mu_f);
        if (d > best) { best = d; T = (uint8_t)k; }
    }
    cam_thresh = T;

    /* ② 先整帧二值化（与行序无关，单独一遍） */
    for (r = 0U; r < (uint32_t)CAM_H; r++)
    {
        uint8_t *row = &disp_img[r][0];
        for (c = 0U; c < (uint32_t)CAM_TXW; c++) { row[c] = (row[c] >= T) ? 255U : 0U; }
    }

    /* ③ 从下往上"跟线"找左边界 ★★★
       ⚠️ 不能每行独立"从最左边扫"：背景里只要有一块白（墙/纸/反光），
          就会把那个白块边缘当成赛道边界（实测：赛道在右下、左上有一块白条 ⇒ 误判 R2 ✗）
       ✅ 正确做法：从最底行（离车最近、最可靠）开始，之后每行只在上一行边界 ±12 内找，
             这样会**一直跟住同一条边**，不会被背景里的白块骗走 ✓ */
    {
        int32_t prev = -1;                        /* 上一行边界列，-1 = 还没有 */
        uint8_t lost = 0U;                        /* 连续丢行计数，超过 8 就重新捕获 */
        int32_t rr;

        for (rr = (int32_t)CAM_H - 1; rr >= 0; rr--)
        {
            uint8_t *row = &disp_img[rr][0];
            int32_t  c_hi, c_lo, cc;
            uint8_t  b = 0xFFU;

            if (prev < 0)
            {
                c_hi = (int32_t)CAM_TXW / 2;      /* 首行：从画面中间往左扫 */
                c_lo = 2;
            }
            else
            {
                c_hi = prev + 20;                 /* 之后：只在上行边界±20 内找 */
                c_lo = prev - 20;
                if (c_hi > (int32_t)CAM_TXW - 3) { c_hi = (int32_t)CAM_TXW - 3; }
                if (c_lo < 2)                    { c_lo = 2; }
            }

            for (cc = c_hi; cc >= c_lo; cc--)     /* 找"白→黑" = 白色赛道的左边缘 */
            {
                if ((row[cc] == 255U) && (row[cc - 1] == 0U)) { b = (uint8_t)cc; break; }
            }

            cam_bound[rr] = b;
            if (0xFFU != b) { prev = (int32_t)b; lost = 0U; }
            else if (lost < 8U) { lost ++; }
            else { prev = -1; lost = 0U; }        /* 掉线太久 ⇒ 下一行重新从中间找（重新捕获远处赛道）*/
        }
    }

    /* ★★ 帧率画进图像（单片机自己画，电脑只收）★★
       用本函数两次调用的间隔反推出帧率（= 实际出帧率），再画在左上角 ✓ */
    {
        static uint32_t t_prev = 0U;
        static uint16_t fps10  = 0U;
        uint32_t now2 = HAL_GetTick();

        if (0U != t_prev)
        {
            uint32_t dt = now2 - t_prev;
            if (dt > 0U)
            {
                uint32_t f10 = 10000U / dt;
                if (f10 > 999U) { f10 = 999U; }
                fps10 = (uint16_t)f10;
            }
        }
        t_prev = now2;
        board_cam_draw_fps_text(fps10);
    }

    /* ★ 路线识别 + 在图上画 "R<code>"（紧跟在 FPS 下面一行） */
    board_cam_classify_route();
    board_cam_draw_route_text();
}

uint8_t cam_sccb_test (void)
{
    uint8_t  i, j;
    uint16_t id = 0U;

    // ★★ 两种 SDA/SCL 顺序 × **手册给的四个从机地址** = 8 种组合全部试一遍 ★★
    //   （手册：0x90 / 0x98 / 0xB0 / 0xB8，由 S_CTRL_ADR0/ADR1 决定；示例图用的是 0xB8）
    for (j = 0U; j < 2U; j++)
    {
        cam_sccb_init(j);
        sccb_bus_recover();

        if (0U == j) { cam_sccb_lv0 = sccb_levels(); }
        else         { cam_sccb_lv1 = sccb_levels(); }

        for (i = 0U; i < 4U; i++)
        {
            sccb_dev_w   = sccb_ids[i];
            cam_sccb_err = sccb_read_reg8(sccb_dev_w, 0x00U, &id);      // R0x00 = 芯片版本

            if ((0U == cam_sccb_err) && ((id >> 8) == 0x13U))           // MT9V034 ⇒ 0x1313
            {
                cam_sccb_addr = sccb_dev_w;                             // 记住哪个地址通了
                if (0U == j) { cam_sccb_id0 = id; } else { cam_sccb_id1 = id; }
                return (uint8_t)(j + 1U);                               // 1 = 接法0 通；2 = 接法1 通
            }
        }
    }

    cam_sccb_init(0U);
    sccb_dev_w = 0x90U;
    return 0U;
}

void cam_cfg_probe (uint8_t baud_idx)
{
    uint32_t t0, i;
    uint8_t  last;

    cam_cfg_uart_open(baud_idx);
    HAL_Delay(5);

    while (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; }      // 清干净
    (void)UART5->SR;  (void)UART5->DR;                          // 读 SR+DR 清 FE/NE/ORE/PE

    // ---------------- 阶段 A：发一次"文档里确有的"命令，然后静听 60ms（看摄像头回不回）----------------
    camcfg_send(0x04, 752);                                     // SET_COL=752（写了也不改变现状）

    cam_probe_edges = 0; cam_probe_low = 0; cam_probe_n = 0;
    cam_probe_rx = 0;    cam_probe_err = 0;
    last = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 60UL)
    {
        uint8_t v = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

        cam_probe_n ++;
        if (0U == v) { cam_probe_low ++; }
        if (v != last) { cam_probe_edges ++; last = v; }

        if (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; cam_probe_rx ++; }
        if (UART5->SR & USART_SR_FE)   { cam_probe_err |= 1U; }
        if (UART5->SR & USART_SR_NE)   { cam_probe_err |= 2U; }
        if (UART5->SR & USART_SR_ORE)  { cam_probe_err |= 4U; }
    }

    // ---------------- 阶段 B：**我们自己持续发 0x55**，同时采样（用来做 PC12↔PD2 短接自检）----------------
    //   🔑 同时数**两个脚**的边沿：
    //     TX_EDGES（PC12）——我们自己发的信号，**必须 > 0**（不然连发都没发出去）
    //     EDGES   （PD2 ）——短接/接过摄像头时应同步出现；**TX>0 而 RX=0 ⇒ 两个脚没真正连上**
    cam_probe_b_edges = 0; cam_probe_b_rx = 0; cam_probe_b_n = 0; cam_probe_b_tx_edges = 0;

    {
        uint8_t last_rx = (GPIOD->IDR & GPIO_PIN_2)  ? 1U : 0U;
        uint8_t last_tx = (GPIOC->IDR & GPIO_PIN_12) ? 1U : 0U;

        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 60UL)
        {
            uint8_t v;
            uint16_t k;

            camcfg_putc(0x55);                                  // 发一个字节（0x55 ⇒ 线上是方波）
            for (k = 0U; k < 40U; k ++)
            {
                v = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;
                cam_probe_b_n ++;
                if (v != last_rx) { cam_probe_b_edges ++; last_rx = v; }

                v = (GPIOC->IDR & GPIO_PIN_12) ? 1U : 0U;        // ★ 同一时刻看我们自己那根 TX
                if (v != last_tx) { cam_probe_b_tx_edges ++; last_tx = v; }

                if (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; cam_probe_b_rx ++; }
            }
        }
    }

    // ---------------- 阶段 C：PD2 "三态检测" —— PD2 上到底有没有东西接着？----------------
    //   ⚠️ 要临时把 PD2 从"UART 复用"改成"普通输入"才能改上下拉；测完必须还原（`cam_cfg_uart_open`）。
    {
        GPIO_InitTypeDef gi = {0};

        gi.Pin   = GPIO_PIN_2;
        gi.Mode  = GPIO_MODE_INPUT;
        gi.Speed = GPIO_SPEED_FREQ_LOW;

        gi.Pull = GPIO_NOPULL;    HAL_GPIO_Init(GPIOD, &gi);  HAL_Delay(1);
        cam_probe_c_np = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

        gi.Pull = GPIO_PULLUP;    HAL_GPIO_Init(GPIOD, &gi);  HAL_Delay(1);
        cam_probe_c_pu = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

        gi.Pull = GPIO_PULLDOWN;  HAL_GPIO_Init(GPIOD, &gi);  HAL_Delay(1);
        cam_probe_c_pd = (GPIOD->IDR & GPIO_PIN_2) ? 1U : 0U;

        cam_cfg_uart_open(baud_idx);                          // ★ 把 PD2 还给 UART5_RX
    }

    for (i = 0; i < 1U; i ++) { (void)i; }                      // （防未用变量）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★★★ 按手册把摄像头配成 **188×120 + PCLK消隐=1 + 指定帧率** ★★★
//   手册依据（5.1 / 5.2 / 5.3）：
//     · 分辨率只有 **752×480 / 376×240 / 188×120** 三档是**同视野**的（其余都是裁剪）；
//       **FPS 可设 1~500**，手册建议 50 或 100 以避开工频干扰
//     · 🔥 **"PCLK 消隐信号"用 STM32 的 DCMI 采集时"必须设置为 1"**（仅 MT9V034 **V2.1+** 支持）
//     · 协议：4 字节/包 `0xA5 | cmd | 数据高8 | 数据低8`，**包间隔 ≥2ms**；
//       **所有参数发完后再发一次 INIT**，摄像头重初始化完成后回 **3 字节 `A5 FF FF`**
//   ⚠️ 不把"没应答"判成失败（有些固件不回），但会把**收到几个字节**报出来供判断。
//   ⚠️ 配完这一套，摄像头就出 **188×120** 了 ⇒ 固件必须跟着切到 `board_cam_set_mode(1)`
//      （缓冲天然就是一整帧，不再需要抽行）—— 所以命令 `K` 里会**自动切**。
//-------------------------------------------------------------------------------------------------------------------
uint32_t cam_cfg_apply_188 (uint16_t fps)
{
    static const uint8_t cmd[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00};
    uint16_t val[10];
    uint8_t  i;
    uint32_t got = 0;
    int      c;

    if (0U == fps) { fps = 50U; }

    val[0] = 0U;          // AUTO_EXP：0 = 关闭自动曝光
    val[1] = 512U;        // EXP_TIME
    val[2] = fps;         // FPS
    val[3] = 188U;        // SET_COL
    val[4] = 120U;        // SET_ROW
    val[5] = 0U;          // LR_OFFSET（列 = 188/376/752 时不允许设偏移）
    val[6] = 0U;          // UD_OFFSET（行 = 120/240/480 时不允许设偏移）
    val[7] = 32U;         // GAIN
    val[8] = 1U;          // ★★ PCLK 消隐 = 1（**STM32 的 DCMI 必须要这一位**）
    val[9] = 0U;          // INIT（最后发 ⇒ 前面那些参数才真正生效）

    cam_cfg_uart_open(0U);                       // 9600
    HAL_Delay(5);
    while (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; }      // 清陈货

    for (i = 0U; i < 10U; i++)
    {
        camcfg_send(cmd[i], val[i]);
        HAL_Delay(2);                            // 手册：一个包发完最好延时 2ms
    }

    // 等应答（3 字节 A5 FF FF）；等不到也照样往下走，只把收到的字节数报出来
    for (i = 0U; i < 100U; i++)
    {
        c = camcfg_getc(20);
        if (c < 0) { break; }
        got ++;
        cam_cfg_ver = (cam_cfg_ver << 8) | (uint32_t)(uint8_t)c;
    }

    cam_cfg_ver   = cam_cfg_ver & 0x00FFFFFFUL;
    cam_cfg_state = got ? 1U : 2U;
    return got;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给总钻风下发完整配置（配成 188×120 + PCLK消隐=1）
// 返回参数     0 = 成功（收到 0xA5 0xFF 0xFF）；1 = 没收到应答
// 备注信息     命令顺序照抄逐飞官方驱动 mt9v03x_set_confing_buffer[]，只有最后一行的值改了：
//                 INIT(0x00)=0  自动曝光(0x01)=0  曝光(0x02)=512  帧率(0x03)=50
//                 列(0x04)=188  行(0x05)=120     左右偏移(0x06)=0  上下偏移(0x07)=0
//                 增益(0x08)=32 **PCLK模式(0x09)=1** ← ★ 就是这一位
//             🔑 两个额外动作：
//               1) 开头先发一次 GET_VERSION(0xF2)，把**收到的原始字节**存进 cam_cfg_ver
//                  （心跳里显示成 RV=xxxxxxxx）—— 用来判定"摄像头根本没说话"还是"我们没解析对"。
//               2) **结尾再补发一次 INIT**：手册原文是
//                  "对摄像头配置…所有配置数据发送完成**然后**发送初始化命令"，
//                  只发开头那次的话，参数可能只是存进去、并没有被应用。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cam_cfg_camera (void)
{
    static const uint8_t  cmd[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    static const uint16_t val[10] = {   0,    0,  512,   50,  188,  120,    0,    0,   32,    1};
    // 波特率候选：手册写 9600；但这一路上"手册没写全"已经踩过好几次，所以顺手扫一遍
    static const uint32_t bauds[5] = {9600U, 115200U, 57600U, 38400U, 19200U};
    uint8_t  i, ok = 0, bi, found = 0;
    uint32_t rx = 0;
    int      c;

    camcfg_uart5_init();
    HAL_Delay(50);

    // ---- 1. 先扫波特率：每档发一次 GET_VERSION(0xF2)，收到任何字节就认定这档 ----
    for (bi = 0; bi < 5U; bi++)
    {
        uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
        uint32_t d = (pclk1 * 25U) / (4U * bauds[bi]);
        uint32_t m = d / 100U;
        uint32_t f = ((d - m * 100U) * 16U + 50U) / 100U;

        UART5->CR1 = 0;
        UART5->BRR = (m << 4) | (f & 0x0FU);
        UART5->CR1 = USART_CR1_UE;
        UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

        HAL_Delay(20);
        while (UART5->SR & USART_SR_RXNE) { (void)UART5->DR; }   // 清干净

        camcfg_send(0xF2, 0x0000);                               // GET_VERSION
        rx = 0;
        for (i = 0; i < 30U; i++)
        {
            c = camcfg_getc(10);
            if (c < 0) { break; }
            rx = (rx << 8) | (uint32_t)(uint8_t)c;
        }
        if (0 != rx) { found = 1; break; }
    }

    // RV 里：高 8 位 = 哪一档波特率通了(1=9600 2=115200 3=57600 4=38400 5=19200，0=都不通)
    //          低 24 位 = 收到的原始字节
    cam_cfg_ver = (found ? ((uint32_t)(bi + 1U) << 24) : 0UL) | (rx & 0x00FFFFFFUL);

    if (0 == found)
    {
        // 5 档都不通 —— 顺手量一下两根线的**静态电平**（UART 空闲时两根本来都该是高），
        // 把"到底哪一侧不对"分开：
        //   TX = PC12 是"我们发出去"的脚（UART 使能且空闲 ⇒ 应该是高）
        //   RX = PD2  是"摄像头答回来"的脚（摄像头 TX 接好且空闲 ⇒ 应该是高）
        uint8_t tx_hi = (GPIOC->IDR & GPIO_PIN_12) ? 1U : 0U;
        uint8_t rx_hi = (GPIOD->IDR & GPIO_PIN_2)  ? 1U : 0U;

        if (0U == tx_hi)      { cam_cfg_state = 4U; }   // 我们这侧的 TX 就是低的 ⇒ 先查 PC12
        else if (0U == rx_hi) { cam_cfg_state = 5U; }   // 摄像头那根 TX 是低的 ⇒ 查 PD2 / 接线
        else                  { cam_cfg_state = 2U; }   // 两极高，但它一声不响
        cam_cfg_ver = ((uint32_t)tx_hi << 1) | (uint32_t)rx_hi;
        return 1U;
    }

    // ---- 2. 用找到的波特率逐个发配置包，包间隔 2ms ----
    for (i = 0; i < 10U; i++)
    {
        camcfg_send(cmd[i], val[i]);
        HAL_Delay(2);
    }
    camcfg_send(0x00, 0x0000);                      // ★ 结尾补发 INIT，让参数真正生效
    HAL_Delay(5);

    // ---- 3. 等应答：3 字节 0xA5 0xFF 0xFF（说明摄像头已经用新参数重新初始化完成）----
    for (i = 0; i < 64U; i++)
    {
        c = camcfg_getc(20);

        if (c < 0) { break; }
        if (0xA5 == (uint8_t)c)
        {
            int a = camcfg_getc(30);
            int b = camcfg_getc(30);

            if ((0xFF == a) && (0xFF == b)) { ok = 1; break; }
        }
    }

    cam_cfg_state = ok ? 1U : 3U;      // 3 = 波特率通了、但配置没应答
    return ok ? 0U : 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头初始化（DMA + 两个中断 + 启动第一帧）
// 返回参数     uint8   0 = 成功
// 备注信息     ⚠️ 必须在 MX_DCMI_Init() 之后调用（DCMI 外设本身由 CubeMX 初始化好了）
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// ★★ 摄像头串口配置的总开关 ★★
//   🔥 2026-10-14 实测：**暂时必须为 0**。
//      摄像头从头到尾不回 ACK（上位机看到 CST=2 / RV=00000000），
//      可我们照样把 INIT + 各参数（还补发了一次 INIT）灌过去
//      ⇒ **反而把它"配死"了**：现象是 PCLK 还在(21MHz) 但 **VSYNC 完全没有边沿**(ec≈1)
//      ⇒ DCMI 一帧都采不到（f=0 / ND=5640）。
//      等把"摄像头为什么不答"查清楚、确认它能正常应答之后，再把它改成 1 打开。
#define CAM_CFG_ENABLE      0
//-------------------------------------------------------------------------------------------------------------------
uint8_t board_cam_init (void)
{
#if (CAM_CFG_ENABLE)
    // ★ 第 1 步：先用串口把摄像头配成 188×120（顺便把 PCLK消隐打开 —— DCMI 必须要它）
    cam_cfg_camera();
    HAL_Delay(200);                     // 给摄像头一点时间稳定输出
#endif

    // ★ 第 2 步：用 TIM3 量一下 PA6 上那根线的频率（量完就把 PA6 还给 DCMI）
    board_pclk_measure();

    // ==================== ★ 自己把这 11 根 DCMI 引脚配成 AF13 ★ ====================
    //   为什么不靠 HAL_DCMI_MspInit()：
    //     1) 之前只验证过 PB7 一根，其余 10 根完全没验过；
    //     2) 实测 DCMI->SR 的 bit0(HSYNC) 恒为 0，而引脚探测显示 HREF 有 80% 时间是高的
    //        ⇒ DCMI 读不到 HREF，最大嫌疑就是这几根引脚没真的进复用模式。
    //   所以干脆自己再强制配一遍（幂等，重复配没坏处），并把速度设成 VERY_HIGH
    //   （CubeMX 生成的是 LOW —— 对输入脚无所谓，但按规范该给高速）。
    {
        GPIO_InitTypeDef gi = {0};

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_DCMI_CLK_ENABLE();

        gi.Mode      = GPIO_MODE_AF_PP;
        gi.Pull      = GPIO_NOPULL;
        gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gi.Alternate = GPIO_AF13_DCMI;

        gi.Pin = GPIO_PIN_4 | GPIO_PIN_6;                                   /* PA4=HSYNC  PA6=PIXCLK */
        HAL_GPIO_Init(GPIOA, &gi);
        gi.Pin = GPIO_PIN_6 | GPIO_PIN_7;                                   /* PB6=D5     PB7=VSYNC */
        HAL_GPIO_Init(GPIOB, &gi);
        gi.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;         /* PC6..PC9 = D0..D3 */
        HAL_GPIO_Init(GPIOC, &gi);
        gi.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;                      /* PE4=D4 PE5=D6 PE6=D7 */
        HAL_GPIO_Init(GPIOE, &gi);

        // ---- 🔥 把板载 W25Q128 按住，别让它来抢 PA6（本板 U5 是焊了的！）----
        //   这块板子的 SPI FLASH 占的就是 PA4/PA5/PA6/PA7 —— 和 DCMI **硬冲突**：
        //     CS = PA4 （现在被 DCMI_HSYNC 占用）
        //     SCK= PA5
        //     SO = PA6 （现在被 DCMI_PIXCLK 占用 ← 就是摄像头的 PCLK 那根！）
        //     DI = PA7
        //   ⚠️ 危险点：HREF 走 PA4，而 PA4 正是 FLASH 的 /CS
        //      ⇒ **HREF 每次变低（约 20% 时间）都会把 FLASH"选中"一次**。
        //      此时若 SCK(PA5) 悬空，线上噪声沿就会被 FLASH 当成时钟，
        //      ⇒ FLASH 可能**主动去驱动它的 SO 脚(PA6)**，把摄像头的 PCLK 顶掉
        //      ⇒ DCMI 收到一堆废边沿 / 压根收不到干净时钟 ⇒ 一帧都采不到。
        //
        //   ✅ 对策（不用拆焊）：把 PA5 / PA7 这两个"MCU→FLASH"的输入脚
        //      **压成低电平推挽输出** ⇒ FLASH 永远等不到时钟沿 ⇒ SO 保持高阻
        //      ⇒ PA6 完全归摄像头的 PCLK。
        //   ★ 最彻底的办法还是把 **RN3（0Ω 排阻）或 U5(W25Q128) 吹掉**，那是物理隔离；
        //      这段代码是为了"不拆焊也能先跑起来"。
        {
            GPIO_InitTypeDef gs = {0};

            gs.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
            gs.Mode  = GPIO_MODE_OUTPUT_PP;
            gs.Pull  = GPIO_NOPULL;
            gs.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIOA, &gs);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);
        }
    }
    // ============================================================================

    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_dcmi.Instance                 = DMA2_Stream1;
    hdma_dcmi.Init.Channel             = DMA_CHANNEL_1;
    hdma_dcmi.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode                = DMA_CIRCULAR;   // ★ 循环：DCMI 连续采集，DMA 转满一圈 = 一帧
    hdma_dcmi.Init.Priority            = DMA_PRIORITY_HIGH;
    // ⚠️ 改成**直接模式（FIFO 关掉）**：DCMI 这种"来一个字节请求一次、不支持突发"的外设，
    //   直接模式最稳；FIFO 模式（阈值=FULL，得先攒满 4 个字才往内存写）在个别场合会把请求挂住，
    //   现象就是"什么寄存器都对、ND 却纹丝不动"。
    hdma_dcmi.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_dcmi.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;   // 直接模式下这个参数被忽略
    hdma_dcmi.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_dcmi.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK) return 1;

    __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);

    cam_frame_cnt = 0;
    cam_ready     = 0;

    // ==================== ★ 修正 DCMI 的三个极性位（按参考手册 RM0090 §15.8.1）★ ====================
    //   🔥 2026-10-14 纠正：上一版我按"HAL 宏名和手册是反的"把这三个位**全改反了** —— 那是我错。
    //      参考手册（这份 PDF 就在「第04章 文档手册」里）第 468 页原文是：
    //        Bit 5 PCKPOL :  0 = Falling edge active . 1 = Rising edge active
    //        Bit 6 HSPOL  :  0 = HSYNC active low    . 1 = HSYNC active high
    //        Bit 7 VSPOL  :  0 = VSYNC active low    . 1 = VSYNC active high
    //      ⇒ **CubeMX 生成的那三个值（RISING / HIGH / LOW）本来就是对的了。**
    //      总钻风实测：VSYNC 高电平占 91% ⇒ 同步脉冲是**负脉冲** ⇒ VSYNC **低有效** ⇒ VSPOL = 0
    //                  HREF  高电平占 80% ⇒ 高电平 = 数据有效     ⇒ HSPOL = 1
    //                  DVP 惯例：数据在 PCLK 上升沿有效            ⇒ PCKPOL = 1
    //      ⇒ 最后 CR 应该是 **0x4061**（ENABLE + PCKPOL + HSPOL + CAPTURE）。
    hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;    // 位=1 → 上升沿采集 ✓
    hdcmi.Init.HSPolarity  = DCMI_HSPOLARITY_HIGH;       // 位=1 → HSYNC 高有效 ✓
    hdcmi.Init.VSPolarity  = DCMI_VSPOLARITY_LOW;        // 位=0 → VSYNC 低有效 ✓
    // ★★ 真正的极性从"RTC 备份寄存器里的组号"来 —— 而且必须在**启动采集之前**配好 ★★
    //   🔥 为什么不能像以前那样"启动后主循环里再改"：
    //      DCMI 一旦在跑，中途改 PCKPOL/HSPOL/VSPOL **是没法重新同步的**，
    //      改动"迟一拍"才生效 ⇒ 你按"第 N 组"去判断，看到的其实是**上一组**的画面
    //      （我就是照这个错位观察把极性锁成第 1 组 ⇒ **直接黑屏**）。
    //   ⇒ 现在：开机就按备份寄存器里的组配好 ⇒ DCMI 从第一帧就同步正确 ✓
    //      换组 = 按一下 RST（`board_cam_pol_search()` 会把"下一组"提前存好）。
    //   ⚠️ 顺序要紧：`HAL_DCMI_Init()`（CubeMX 里调的）会用 `hdcmi.Init.*` **重推**这三个位，
    //      而 `HAL_DCMI_Start_DMA()` 只动 CM/CAPTURE ⇒ 在这里改 CR 是**有效且不会被覆盖**的。
    //   ★ 启动阶段先用**打底组**（实测"直接设就能同步上"的那一组）；
    //     主循环里的 `board_cam_pol_search()` 之后会**切换**到目标组
    //     —— 那个"变化"才是真正让 DCMI 按目标组重新同步的动作 ✓
    dbg_pol_idx = CAM_POL_PRIMER;
    board_cam_apply_polarity(dbg_pol_idx);
    // ============================================================================================

    // ⚠️ 2026-10-14 18:28：**CROP 已撤销** —— 实测它反而更糟（只采到 1 行、其余全黑），
    //   说明"摄像头是 376×480、缓冲不够需要裁一半"这个推断不成立。
    //   （`DCMI->CWSIZER = (CAM_H << 16) | CAM_W; DCMI->CR |= DCMI_CR_CROP;` 那两行已删除。）
    //   注意 CMSIS 里寄存器成员名是 **CWSTRTR / CWSIZER**（带尾巴 R），不是 CWSTRT/CWSIZE。
    // ======================================================================================

    // ★★ 启动采集，**并且在启动之后再写一次极性** ★★
    //   🔥🔥 2026-10-14 19:05 关键实测（三个版本对照得出来的）：
    //      · 旧版（能出图）：极性是在 `Start` **之后**（主循环里）写的 ⇒ 有图 ✓
    //      · 只把极性写在 `Start` **之前**（上一版）：`SR=0000 RIS=0000 er=0`、
    //        **缓冲全是 0**、DCMI 一个事件都不产生 ⇒ "所有极性都没图" ✓
    //   ⇒ **DCMI 必须靠"采集已经启动之后再写一次极性"才真正按它去同步。**
    //      跟极性的**值**无关，是**写的时机**。
    //   ⇒ 所以这里 `board_cam_start()` 之后**立刻再写一次**（不用等主循环/WiFi 初始化那 60 秒）。
    {
        uint8_t r = board_cam_start();

        board_cam_apply_polarity(dbg_pol_idx);
        return r;
    }
}
//===================================================摄像头===================================================


//===================================================板载 LED（PB2）===================================================

#define LED_PORT    GPIOB
#define LED_PIN     GPIO_PIN_2

static uint8_t  led_code = LED_CODE_RETRY;
static uint32_t led_t0   = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化板载 LED
// 备注信息     ⚠️ 空板子上的绿灯接在 **PB2**（就是 BOOT1 那个脚，板上有下拉，启动后可以做普通 IO）。
//              约定是 **高电平点亮**。如果你发现逻辑反了（常亮/常灭对不上），
//              把下面所有 GPIO_PIN_SET / GPIO_PIN_RESET 对调即可。
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
