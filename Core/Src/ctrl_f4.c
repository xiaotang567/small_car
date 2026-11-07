/*********************************************************************************************************************
* 文件名称          ctrl_f4.c
* 适用平台          STM32F407
* 说明              车模控制层（从 MSPM0G3507 工程移植）：电机 / 编码器 / 舵机 / MPU9250 / 速度 PID
*                   全部寄存器级驱动，理由见 ctrl_f4.h 顶部
*
* 修改记录
* 日期              作者                备注
* 2026-11-05        织命                从 MSPM0G3507 工程移植（motor/encoder/speed_ctrl/hx_servo/mpu9250/yaw）
********************************************************************************************************************/

#include "ctrl_f4.h"
#include "wifi_f4.h"          /* OLED 状态页要显示 WiFi 连接状态 */
#include <stdio.h>            /* sprintf */
#include <string.h>

#define CTRL_APB1_HZ_LOCAL  42000000UL      /* APB1 = 42MHz（板级时钟树，见 board_f4.c） */
#define CTRL_TIM_APB1_HZ    84000000UL      /* APB1 上的定时器 = 84MHz */

//===================================================★ 微秒延时（DWT）===================================================
static void ctrl_dwt_init (void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ctrl_delay_us (uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);

    while ((DWT->CYCCNT - start) < ticks) { }
}

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
static void encoder_tick_us (uint32_t dt_us)
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

//===================================================★ 幻尔总线舵机（UART4，1Mbps，同步轮询）===================================================
//   ★★★ 2026-11-07（FW=2103）：**按 MSPM0 原工程 user/src/hx_servo.c 的写法整体重写** ★★★
//
//   为什么换掉原来那版（中断 + 环形缓冲 + 主循环状态机）：
//     把 PC10(TX) 和 PC11(RX) 一并起来（收发短接）后，板子会**把自己发的每一帧都收回来**
//     ⇒ RXNE 中断不断；而 2098 以前 SysTick 是最低优先级(15)、UART4 是 6
//     ⇒ 时间基准被饿死 ⇒ 所有 `while (HAL_GetTick() - t0 < N)` 永不超时 ⇒ 板子卡死。
//   原工程的做法**根本不用 UART 中断**：
//     · 发一帧 → **立刻轮询**把应答收回来（非阻塞查 RXNE）
//     · 超时用**循环次数**数出来（SERVO_RX_WAIT_LOOPS），和 HAL_GetTick 无关
//     · 收到的帧和"刚发出去那帧"一模一样 ⇒ 判定为回显 ⇒ 丢掉，继续等真正的应答
//   ⇒ "中断风暴"这条路从结构上消失；并把 PC10/PC11 短接，最多只是读不到舵机，不会卡板子。
//
//   帧格式： FF FF ID LEN INSTR P1..Pn CHK
//            LEN = 参数个数 + 2
//            CHK = ~(ID + LEN + INSTR + P1..Pn) & 0xFF
//   写位置： INSTR=0x03  addr=0x2A  2 字节小端（0~4095）
//   读位置： INSTR=0x02  addr=0x38  2 字节
//   速度  ： addr=0x2E，符号-幅值（负数 = 最高位 BIT15 置 1，不是补码）
//===================================================

#define SERVO_TX_BUF_MAX        (32U)       /* 发送缓冲上限 */
#define SERVO_RX_BUF_MAX        (32U)       /* 接收缓冲上限（LEN 最大 32）*/
#define SERVO_PARAM_MAX         (24U)       /* 单帧最多几个参数 */
#define SERVO_RX_GUARD_MAX      (64U)       /* 找包头时最多空转几轮，防死等 */

/* ★ 等一个字节要空转多少圈（**不用 HAL_GetTick**，和时间基准无关）
   168MHz 下 40000 圈约 1~2ms；1Mbps 一个字节只要 10µs，绰绰有余。
   没应答时也就是白等这一两毫秒，绝不卡死。*/
#define SERVO_RX_WAIT_LOOPS     (40000U)
/* TX 等 TXE 的圈数上限（防止 UART 没使能时死等）*/
#define SERVO_TX_WAIT_LOOPS     (100000U)

/* ---- 幻尔 HX 系列：指令码 / 寄存器地址 ---- */
#define SERVO_INST_PING         (0x01U)
#define SERVO_INST_READ         (0x02U)
#define SERVO_INST_WRITE        (0x03U)
#define SERVO_REG_MODE          (0x21U)
#define SERVO_REG_TORQUE        (0x28U)
#define SERVO_REG_GOAL_POS      (0x2AU)
#define SERVO_REG_GOAL_SPEED    (0x2EU)
#define SERVO_REG_CURPOS        (0x38U)

static uint8_t  g_servo_id       = CTRL_SERVO_ID;
static uint8_t  g_servo_err      = 0U;
static uint16_t g_servo_pos      = 0xFFFFU;         /* 最近一次**读到**的位置（0xFFFF=还没读到）*/
static uint16_t g_servo_target   = CTRL_SERVO_POS_CENTER;
static uint16_t g_servo_last_set = 0xFFFFU;          /* 0xFFFF = 还没下发过 ⇒ 开机第一次节拍会摆到中位 */
static uint8_t  g_servo_boot     = 0U;
static uint8_t  g_servo_speed_ok = 0U;
static uint8_t  g_servo_half     = 0U;              /* 兼容旧接口；同步轮询下不再切引脚 */
static uint32_t g_servo_baud     = CTRL_SERVO_BAUD;

static uint32_t g_srv_tx_bytes = 0U;
static uint32_t g_srv_rx_bytes = 0U;
static uint32_t g_srv_rx_bad   = 0U;
static uint32_t g_srv_tx_cnt   = 0U;
static uint32_t g_srv_rx_cnt   = 0U;

/* 刚发出去的那一帧（用来识别总线回显）*/
static uint8_t  g_srv_last_tx[SERVO_TX_BUF_MAX];
static uint8_t  g_srv_last_tx_len = 0U;

static uint8_t servo_checksum (const uint8_t *p, uint8_t n)
{
    uint32_t sum = 0U;
    uint8_t  i;

    for (i = 0U; i < n; i ++) { sum += (uint32_t)p[i]; }
    return (uint8_t)((~sum) & 0xFFU);
}

/* 改波特率 —— ★★★ 2026-11-07（FW=2108）：**这里原来算错了 16 倍，舵机一直没通过话** ★★★
   错的写法（沿用了很久）：
       div = fCK / bps;              // 42MHz / 1M = 42，以为这是"整数分频"
       UART4->BRR = div << 4;        // → 672
   实际上 OVER8=0 时：
       波特率 = fCK / (16 × USARTDIV)     ⇒   USARTDIV = fCK/(16×bps)
       而 BRR  = USARTDIV × 16           ⇒   **BRR = fCK / bps**（整数就是完整的 BRR，不用再移 4 位！）
   ⇒ 旧写法把 BRR 写成 672 ⇒ USARTDIV = 42 ⇒ 实际波特率 = 42e6/(16×42) = **62500**。
      1Mbps 的舵机收到的是一串 62.5k 的乱码，当然一辈子不应答；
      而 PC10/PC11 回环测试照样通（收发两边一样错）。
   正确：BRR = round(fCK / bps)。1M → 42 (=0x2A)，115200 → 365。两者都 ≤ 65535，
   所以根本用不着 OVER8（BRR 最大 65535 ⇒ 最低约 641bps）。*/
static void servo_set_baud (uint32_t bps)
{
    uint32_t brr;

    if (0U == bps) { return; }

    brr = (CTRL_APB1_HZ_LOCAL + (bps / 2U)) / bps;   /* ★ BRR 本身 = fCK/baud（OVER8=0）*/
    if (0U == brr) { brr = 1U; }

    UART4->CR1 &= (uint16_t)(~USART_CR1_UE);
    UART4->CR1 &= (uint16_t)(~USART_CR1_OVER8);      /* 16 倍过采样 */
    UART4->BRR  = (uint16_t)(brr & 0xFFFFU);         /* ★ 不再 << 4 */
    UART4->CR1 |= USART_CR1_UE;
    UART4->CR1 |= (uint16_t)(USART_CR1_TE | USART_CR1_RE);
    g_servo_baud = bps;
}

/* 非阻塞取一个字节：有数据返回 0，没有返回 1（只碰硬件寄存器） */
static uint8_t servo_hw_byte (uint8_t *d)
{
    if (0U != (UART4->SR & USART_SR_RXNE))
    {
        *d = (uint8_t)(UART4->DR & 0xFFU);       /* 读 DR：同时清 ORE/FE/NE/PE */
        g_srv_rx_bytes ++;
        return 0U;
    }
    return 1U;
}

/* ★★★ 2026-11-07（FW=2105）：**软件 FIFO** ★★★
   现场：把 PC10(TX) 和 PC11(RX) 短接做回环，结果是"发 1644B / 只收 549B"。
   原因不是引脚坏，而是 **F4 的 UART 只有 1 个字节的接收寄存器**：
   1Mbps 下字节间隔只有 **10µs**，而同步驱动在发送时 CPU 一直盯着 TXE 位、
   没空读 RX ⇒ 回显在发送过程中就一个个 **溢出（ORE）丢掉了**。
   （这跟 MSPM0 原工程注释里说的"要手动打开 FIFO，否则 1 个字节缓冲接不住"是同一件事。）
   ⇒ 这里补一个 64 字节软件 FIFO，并规定：**任何等待循环里都要顺手把硬件 RDR 搬进 FIFO**。
      这样"等 TXE 的那 10µs"也在收字节，回环就能发多少收多少。
   ⚠️ 真正的舵机是"收完整帧再回话"，本来不会和发送重叠 ⇒ 这条对真机没有副作用。*/
#define SERVO_FIFO_SIZE     (64U)

static uint8_t  g_srv_fifo[SERVO_FIFO_SIZE];
static uint16_t g_srv_fifo_head = 0U;
static uint16_t g_srv_fifo_tail = 0U;

/* 把硬件 RDR 里现有的字节全部搬进 FIFO（不等待、不阻塞）*/
static void servo_rx_pump (void)
{
    uint8_t  d;
    uint16_t next;

    while (0U == servo_hw_byte(&d))
    {
        next = (uint16_t)((g_srv_fifo_head + 1U) % SERVO_FIFO_SIZE);
        if (next == g_srv_fifo_tail) { return; }         /* FIFO 满：丢（和硬件溢出等效）*/
        g_srv_fifo[g_srv_fifo_head] = d;
        g_srv_fifo_head = next;
    }
}

/* 从 FIFO 里取一个字节：有返回 0，没有返回 1 */
static uint8_t servo_fifo_get (uint8_t *d)
{
    if (g_srv_fifo_tail == g_srv_fifo_head) { return 1U; }
    *d = g_srv_fifo[g_srv_fifo_tail];
    g_srv_fifo_tail = (uint16_t)((g_srv_fifo_tail + 1U) % SERVO_FIFO_SIZE);
    return 0U;
}

/* 读一个字节，**自己数圈数超时**（原工程 hx_rx_byte 的写法）
   ★ 等待期间必须不停 servo_rx_pump()，否则 10µs 就溢出了 */
static uint8_t servo_rx_byte (uint8_t *d)
{
    uint32_t guard = SERVO_RX_WAIT_LOOPS;

    while (guard --)
    {
        servo_rx_pump();                        /* ★ 每圈都把硬件 RDR 搬进 FIFO */
        if (0U == servo_fifo_get(d)) { return 0U; }
    }
    return 1U;
}

/* 把 RX 里残留的字节清掉（发之前清一下，别把上一帧的尾巴当成应答）*/
static void servo_rx_flush (void)
{
    uint8_t d;
    uint8_t n = 0U;

    servo_rx_pump();                            /* 先把硬件里的搬进 FIFO */
    while ((n < 200U) && (0U == servo_fifo_get(&d))) { n ++; }
    g_srv_fifo_head = 0U;
    g_srv_fifo_tail = 0U;
}

/* ★ 总线方向控制（单线半双工；对应老版本里 TC 中断里那段"放开 PC10"）
   PC10 是 UART4_TX，MODER 的 [21:20] 两位：10 = 复用推挽，00 = 输入(高阻)。
   推挽的时候总线由板子驱动；放开以后靠外部上拉（和 PC11 的内部上拉）维持空闲高，
   舵机才有机会把线拉低回话。*/
static void servo_bus_drive (void)
{
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << 20)) | (2UL << 20);   /* 推挽 AF */
}

static void servo_bus_release (void)
{
    GPIOC->MODER &= ~(3UL << 20);                                  /* 高阻输入 */
}

/* 组一帧发出去（原工程 hx_tx_frame）*/
static void servo_tx_frame (uint8_t id, uint8_t inst, const uint8_t *params, uint8_t n)
{
    uint8_t buf[SERVO_TX_BUF_MAX];
    uint8_t i;

    buf[0] = 0xFFU;
    buf[1] = 0xFFU;
    buf[2] = id;
    buf[3] = (uint8_t)(n + 2U);
    buf[4] = inst;

    for (i = 0U; i < n; i ++) { buf[5U + i] = params[i]; }

    buf[5U + n] = servo_checksum(&buf[2], (uint8_t)(n + 3U));

    /* 存一份，供接收端识别回显 */
    g_srv_last_tx_len = (uint8_t)(6U + n);
    for (i = 0U; i < g_srv_last_tx_len; i ++) { g_srv_last_tx[i] = buf[i]; }

    servo_rx_flush();

    /* ★★★ 2026-11-07（FW=2106）：**单线总线：说话之前先把 PC10 切回推挽 AF** ★★★
       官方的 F103 例程能直接推挽通，是因为中间有幻尔的**调试板**（半双工转换在那里做）。
       我们是把舵机直接并在 PC10/PC11 上：如果不主动释放，PC10 会一直推挽拉高，
       舵机想回话时就被顶住 ⇒ 永远收不到应答。*/
    if (0U != g_servo_half) { servo_bus_drive(); }

    for (i = 0U; i < g_srv_last_tx_len; i ++)
    {
        uint32_t guard = SERVO_TX_WAIT_LOOPS;

        /* ★★★ 2026-11-07（FW=2105）：**等 TXE 的空隙里也要搬 RX** ★★★
           1Mbps 下一个字节只有 10µs，而 F4 的接收寄存器只有 1 个字节。
           如果发送期间只顾着盯 TXE，回显就会一个个溢出丢掉 ——
           现场"发 1644B / 只收 549B"就是这个。这里顺手搬进软件 FIFO，
           回环就能做到"发多少收多少"。*/
        while ((0U == (UART4->SR & USART_SR_TXE)) && (0U != (-- guard)))
        {
            servo_rx_pump();
        }
        if (0U == guard) { break; }
        UART4->DR = buf[i];
        g_srv_tx_bytes ++;
    }

    /* ★ 发完**等最后一个位真的移出去（TC）**再放开总线 —— 早放会截断最后一个字节 */
    if (0U != g_servo_half)
    {
        uint32_t guard = SERVO_TX_WAIT_LOOPS;

        while ((0U == (UART4->SR & USART_SR_TC)) && (0U != (-- guard)))
        {
            servo_rx_pump();
        }
        servo_bus_release();            /* PC10 → 高阻，总线交给舵机 */
    }
    g_srv_tx_cnt ++;
}

/* 在字节流里找包头 FF FF（带空转上限，绝不死等）*/
static uint8_t servo_find_header (void)
{
    uint8_t b;
    uint8_t guard = 0U;

    for (;;)
    {
        if (0U != servo_rx_byte(&b)) { return 1U; }

        if (0xFFU == b)
        {
            if (0U != servo_rx_byte(&b)) { return 1U; }
            if (0xFFU == b) { return 0U; }              /* 连续两个 0xFF，包头确认 */
        }

        if (++ guard > SERVO_RX_GUARD_MAX) { return 1U; }
    }
}

/* 收一帧应答（最多试 4 次：前面可能是自己发出去的回显）*/
static uint8_t servo_recv_frame (uint8_t *rid, uint8_t *rerr, uint8_t *rdata, uint8_t *rlen)
{
    uint8_t  id, len, i, attempt;
    uint8_t  tmp[SERVO_RX_BUF_MAX];
    uint8_t  raw[SERVO_RX_BUF_MAX + 4U];                /* 整帧原样，用来和刚发那帧比对 */
    uint32_t sum;

    for (attempt = 0U; attempt < 4U; attempt ++)
    {
        if (0U != servo_find_header()) { return 1U; }

        raw[0] = 0xFFU;
        raw[1] = 0xFFU;

        if (0U != servo_rx_byte(&id))  { return 1U; }
        if (0U != servo_rx_byte(&len)) { return 1U; }
        raw[2] = id;
        raw[3] = len;

        if ((0U == len) || (len > SERVO_RX_BUF_MAX)) { continue; }   /* 长度不合理，重找 */

        for (i = 0U; i < len; i ++)
        {
            if (0U != servo_rx_byte(&tmp[i])) { return 1U; }
            raw[4U + i] = tmp[i];
        }

        /* ★★★ 回显过滤：和刚发出去那帧一模一样 ⇒ 是我们自己的回声，丢掉继续等 */
        if ((uint8_t)(len + 4U) == g_srv_last_tx_len)
        {
            uint8_t same = 1U;

            for (i = 0U; i < (uint8_t)(len + 4U); i ++)
            {
                if (raw[i] != g_srv_last_tx[i]) { same = 0U; break; }
            }
            if (0U != same) { continue; }
        }

        /* 校验：~(ID + LEN + tmp[0..len-2]) == tmp[len-1] */
        sum = (uint32_t)id + (uint32_t)len;
        for (i = 0U; i < (uint8_t)(len - 1U); i ++) { sum += (uint32_t)tmp[i]; }

        if ((uint8_t)((~sum) & 0xFFU) != tmp[len - 1U])
        {
            g_srv_rx_bad ++;
            continue;
        }

        if (NULL != rid)  { *rid  = id; }
        if (NULL != rerr) { *rerr = tmp[0]; }                    /* ERROR 是第一个字节 */
        if (NULL != rlen) { *rlen = (uint8_t)(len - 2U); }       /* 数据 = 去掉 ERROR 和校验 */

        if ((NULL != rdata) && (len > 2U))
        {
            for (i = 1U; i < (uint8_t)(len - 1U); i ++) { rdata[i - 1U] = tmp[i]; }
        }

        g_srv_rx_cnt ++;
        return 0U;
    }

    return 1U;
}

/* 写若干字节到舵机内存表 */
static uint8_t servo_write (uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t p[SERVO_PARAM_MAX];
    uint8_t err = 0xFFU;
    uint8_t i;

    if ((0U == n) || (n > (SERVO_PARAM_MAX - 1U))) { return 1U; }

    p[0] = addr;
    for (i = 0U; i < n; i ++) { p[1U + i] = data[i]; }

    servo_tx_frame(id, SERVO_INST_WRITE, p, (uint8_t)(n + 1U));

    if (0U != servo_recv_frame(NULL, &err, NULL, NULL)) { return 1U; }

    g_servo_err = err;
    return 0U;
}

/* 从舵机内存表读若干字节 */
static uint8_t servo_read (uint8_t id, uint8_t addr, uint8_t n, uint8_t *out)
{
    uint8_t p[2];
    uint8_t d[SERVO_RX_BUF_MAX];
    uint8_t dlen = 0U;
    uint8_t err  = 0xFFU;
    uint8_t i;

    if ((NULL == out) || (0U == n) || (n > SERVO_RX_BUF_MAX)) { return 1U; }

    p[0] = addr;
    p[1] = n;

    servo_tx_frame(id, SERVO_INST_READ, p, 2U);

    if (0U != servo_recv_frame(NULL, &err, d, &dlen)) { return 1U; }
    if (dlen < n) { return 1U; }

    for (i = 0U; i < n; i ++) { out[i] = d[i]; }

    g_servo_err = err;
    return 0U;
}

/* PING 一下（只读不写，验证"接线 + 波特率 + ID"） */
static uint8_t servo_ping (uint8_t id, uint8_t *err)
{
    uint8_t e   = 0xFFU;
    uint8_t rid = 0U;

    servo_tx_frame(id, SERVO_INST_PING, NULL, 0U);

    /* ★ 发完**立刻**开始收，中间不能加延时（1Mbps 下 6 字节约 60µs）*/
    if (0U != servo_recv_frame(&rid, &e, NULL, NULL)) { return 1U; }

    if (NULL != err) { *err = e; }
    return (0U != e) ? 2U : 0U;                 /* 2 = 应答了但舵机自己报故障 */
}

/* 读当前位置（同步，成功才更新 g_servo_pos）*/
static void servo_read_pos (void)
{
    uint8_t d[2];

    if (0U == servo_read(g_servo_id, SERVO_REG_CURPOS, 2U, d))
    {
        g_servo_pos = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));   /* 小端 */
    }
}

/* 设目标速度（0x2E，符号-幅值：负数最高位置 1）*/
static uint8_t servo_set_speed (int16_t speed)
{
    uint8_t  d[2];
    uint16_t w;

    if (speed >  (int16_t)CTRL_SERVO_SPEED) { speed =  (int16_t)CTRL_SERVO_SPEED; }
    if (speed < -(int16_t)CTRL_SERVO_SPEED) { speed = -(int16_t)CTRL_SERVO_SPEED; }

    w = (0 > speed) ? (uint16_t)((uint16_t)(-speed) | 0x8000U) : (uint16_t)speed;
    d[0] = (uint8_t)(w & 0xFFU);
    d[1] = (uint8_t)((w >> 8) & 0xFFU);

    return servo_write(g_servo_id, SERVO_REG_GOAL_SPEED, d, 2U);
}

//===================================================对外接口===================================================

uint32_t ctrl_servo_rx_bytes (void) { return g_srv_rx_bytes; }
uint32_t ctrl_servo_tx_bytes (void) { return g_srv_tx_bytes; }
uint32_t ctrl_servo_rx_bad   (void) { return g_srv_rx_bad; }
uint32_t ctrl_servo_baud     (void) { return g_servo_baud; }
uint32_t ctrl_servo_tx_cnt   (void) { return g_srv_tx_cnt; }
uint32_t ctrl_servo_rx_cnt   (void) { return g_srv_rx_cnt; }
uint8_t  ctrl_servo_id       (void) { return g_servo_id; }
uint8_t  ctrl_servo_get_err  (void) { return g_servo_err; }
uint16_t ctrl_servo_get_pos  (void) { return g_servo_pos; }
uint8_t  ctrl_servo_is_half  (void) { return g_servo_half; }

/* PC11(RX) 现在的电平：1 = 空闲高（正常），0 = 被拉低 */
uint8_t ctrl_servo_rx_pin (void)
{
    return (0U != (GPIOC->IDR & GPIO_PIN_11)) ? 1U : 0U;
}

/* 运行时切换"单线/全双工"
   单线（half=1）：PC10 发完立刻放成高阻，总线和 PC11 一起交给舵机（默认）；
   全双工（half=0）：PC10 一直是推挽 TX、PC11 是 RX（中间有调试板时才这么接）。*/
void ctrl_servo_set_half (uint8_t half)
{
    g_servo_half = (0U != half) ? 1U : 0U;

    if (0U == g_servo_half) { servo_bus_drive(); }     /* 全双工：一直由板子驱动 */
}

void ctrl_servo_init (void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC10 = UART4_TX (AF8)，PC11 = UART4_RX (AF8)，上拉 */
    gi.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gi);

    UART4->CR1 = 0U;
    UART4->CR1 = USART_CR1_UE;
    servo_set_baud(CTRL_SERVO_BAUD);
    UART4->CR1 |= (uint16_t)(USART_CR1_TE | USART_CR1_RE);

    /* ★★★ 关键改动：**不打开 RXNE 中断、不让 NVIC 管 UART4** ★★★
       整个驱动改成同步轮询（和 MSPM0 原工程一样）⇒ 没有任何中断，
       "中断风暴把时间基准饿死"这条路从结构上消失。*/
    UART4->CR1 &= (uint16_t)(~USART_CR1_RXNEIE);
    UART4->CR3 &= (uint16_t)(~USART_CR3_EIE);
    HAL_NVIC_DisableIRQ(UART4_IRQn);

    g_srv_tx_bytes = 0U;
    g_srv_rx_bytes = 0U;
    g_srv_rx_bad   = 0U;
    g_srv_tx_cnt   = 0U;
    g_srv_rx_cnt   = 0U;
    g_servo_pos      = 0xFFFFU;
    g_servo_err      = 0U;
    g_servo_boot     = 0U;
    g_servo_speed_ok = 0U;

    /* ★ 默认走"单线半双工"（发完放开 PC10），和现场把 PC10/PC11 并到一根线上的接法一致 */
    ctrl_servo_set_half((uint8_t)CTRL_SERVO_HALF_DUPLEX_DEFAULT);

    /* 等舵机上电初始化（原工程 system_delay_ms(300)；用 DWT 延时，不依赖 SysTick）*/
    ctrl_delay_us(300000UL);

    ctrl_servo_scan();
}

/* ★★★ 2026-11-07（FW=2107）：**ID 固定 1，波特率自动试两档** ★★★
   舵机型号 HX-10HM。它的手册前后不一致（第 1 页写 115200、规格表和协议文档写 1Mbps）——
   其实 115200 是**调试板到电脑**那一路，**舵机总线那一路是 1000000**。
   为了不再靠猜：开机把 1M → 115200 各 PING 一次（每次最多 1~2ms，总共几毫秒），
   谁应答就用谁；两个都不应答就停在 1M，运行中还会继续换档重试（见 board_ctrl_tick）。*/
void ctrl_servo_scan (void)
{
    static const uint32_t bauds[] = { CTRL_SERVO_BAUD, CTRL_SERVO_BAUD_ALT };
    uint8_t  bi;
    uint8_t  err = 0xFFU;

    ctrl_oled_boot(9U);

    g_servo_id = CTRL_SERVO_ID;                 /* ★ 固定 = 1 */

    for (bi = 0U; bi < (uint8_t)(sizeof(bauds) / sizeof(bauds[0])); bi ++)
    {
        servo_set_baud(bauds[bi]);

        if (0U == servo_ping(g_servo_id, &err))     /* 0 = 应答正常；2 = 应答但报故障 */
        {
            g_servo_err = err;
            return;                                 /* 这一档能通 ⇒ 定下来 */
        }
    }

    servo_set_baud(CTRL_SERVO_BAUD);            /* 都不应答：回默认档 */
    g_servo_err = 0xFFU;                        /* 留着给上位机显示 */
}

/* 走到目标位置（写 0x2A 两个字节，同步等应答）*/
uint8_t ctrl_servo_set_pos (uint16_t pos)
{
    uint8_t d[2];

    if (pos < CTRL_SERVO_POS_MIN) { pos = CTRL_SERVO_POS_MIN; }
    if (pos > CTRL_SERVO_POS_MAX) { pos = CTRL_SERVO_POS_MAX; }

    g_servo_target = pos;

    d[0] = (uint8_t)(pos & 0xFFU);
    d[1] = (uint8_t)((pos >> 8) & 0xFFU);

    if (0U != servo_write(g_servo_id, SERVO_REG_GOAL_POS, d, 2U)) { return 1U; }

    g_servo_pos      = pos;
    g_servo_last_set = pos;         /* 记下来 ⇒ 主循环节拍不会重复下发同一目标 */
    return 0U;
}

/* UART4 中断：同步轮询驱动已经把它关掉了，这里只保留符号（stm32f4xx_it.c 还在调）*/
void ctrl_uart4_irq (void)
{
    UART4->CR1 &= (uint16_t)(~USART_CR1_RXNEIE);
        (void)UART4->DR;
}

//===================================================★ MPU9250（软 I2C：PB8=SCL PB9=SDA）===================================================
//   原工程用的也是纯软件 I2C（逐飞 soft_iic），这里照着实现：
//     SCL 推挽、SDA **开漏**（读的时候直接读 IDR，和原工程 SOFT_IIC_SDA_IO_SWITCH=0 一样）
#define I2C_SCL_H()   (GPIOB->BSRR = GPIO_PIN_8)
#define I2C_SCL_L()   (GPIOB->BSRR = (uint32_t)GPIO_PIN_8 << 16)
#define I2C_SDA_H()   (GPIOB->BSRR = GPIO_PIN_9)
#define I2C_SDA_L()   (GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16)
#define I2C_SDA_RD()  ((GPIOB->IDR & GPIO_PIN_9) ? 1U : 0U)

#define MPU_REG_SMPLRT  0x19U
#define MPU_REG_CONFIG  0x1AU
#define MPU_REG_GYROCFG 0x1BU
#define MPU_REG_ACCCFG  0x1CU
#define MPU_REG_ACCEL   0x3BU
#define MPU_REG_PWR     0x6BU
#define MPU_REG_WHOAMI  0x75U

static uint8_t  g_imu_who  = 0U;
static uint8_t  g_imu_ok   = 0U;
static int16_t  g_acc[3]   = {0, 0, 0};
static int16_t  g_gyr[3]   = {0, 0, 0};

static void i2c_delay (void) { ctrl_delay_us(CTRL_IMU_I2C_DELAY_US); }

static void i2c_start (void)
{
    I2C_SDA_H(); I2C_SCL_H(); i2c_delay();
    I2C_SDA_L(); i2c_delay();
    I2C_SCL_L(); i2c_delay();
}

static void i2c_stop (void)
{
    I2C_SDA_L(); i2c_delay();
    I2C_SCL_H(); i2c_delay();
    I2C_SDA_H(); i2c_delay();
}

static uint8_t i2c_write_byte (uint8_t b)
{
    uint8_t i, ack;

    for (i = 0U; i < 8U; i++)
    {
        if (0U != (b & 0x80U)) { I2C_SDA_H(); } else { I2C_SDA_L(); }
        b = (uint8_t)(b << 1);
        i2c_delay();
        I2C_SCL_H(); i2c_delay();
        I2C_SCL_L(); i2c_delay();
    }

    I2C_SDA_H();                 /* 释放 SDA，等从机拉低 */
    i2c_delay();
    I2C_SCL_H(); i2c_delay();
    ack = (0U == I2C_SDA_RD()) ? 0U : 1U;      /* 0 = ACK */
    I2C_SCL_L(); i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte (uint8_t nack)
{
    uint8_t i, b = 0U;

    I2C_SDA_H();
    for (i = 0U; i < 8U; i++)
    {
        b = (uint8_t)(b << 1);
        i2c_delay();
        I2C_SCL_H(); i2c_delay();
        if (0U != I2C_SDA_RD()) { b |= 1U; }
        I2C_SCL_L(); i2c_delay();
    }

    if (0U != nack) { I2C_SDA_H(); } else { I2C_SDA_L(); }
    i2c_delay();
    I2C_SCL_H(); i2c_delay();
    I2C_SCL_L(); i2c_delay();
    I2C_SDA_H();
    return b;
}

static uint8_t mpu_write_reg (uint8_t reg, uint8_t val)
{
    i2c_start();
    if (0U != i2c_write_byte((uint8_t)(CTRL_IMU_ADDR << 1))) { i2c_stop(); return 1U; }
    (void)i2c_write_byte(reg);
    (void)i2c_write_byte(val);
    i2c_stop();
    return 0U;
}

static uint8_t mpu_read_regs (uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t i;

    i2c_start();
    if (0U != i2c_write_byte((uint8_t)(CTRL_IMU_ADDR << 1))) { i2c_stop(); return 1U; }
    (void)i2c_write_byte(reg);
    i2c_start();                                     /* 重复起始 */
    if (0U != i2c_write_byte((uint8_t)((CTRL_IMU_ADDR << 1) | 1U))) { i2c_stop(); return 1U; }

    for (i = 0U; i < n; i++)
    {
        buf[i] = i2c_read_byte((uint8_t)((i + 1U == n) ? 1U : 0U));
    }
    i2c_stop();
    return 0U;
}

void ctrl_imu_init (void)
{
    GPIO_InitTypeDef gi;
    uint8_t id = 0U;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gi.Mode      = GPIO_MODE_OUTPUT_OD;              /* 开漏，靠外部上拉（模块板一般自带） */
    gi.Pull      = GPIO_PULLUP;                      /* 没外部上拉时内部弱上拉兜底 */
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = 0;
    HAL_GPIO_Init(GPIOB, &gi);
    I2C_SCL_H(); I2C_SDA_H();

    ctrl_delay_us(5000U);                            /* 上电稳定（原工程 50ms，这里不阻塞太久） */

    if (0U != mpu_read_regs(MPU_REG_WHOAMI, &id, 1U)) { g_imu_who = 0U; g_imu_ok = 0U; return; }
    g_imu_who = id;

    (void)mpu_write_reg(MPU_REG_PWR,    0x00U);      /* 解除休眠（不做软复位，同原工程） */
    ctrl_delay_us(50000U);
    (void)mpu_write_reg(MPU_REG_ACCCFG, 0x00U);      /* ±2g */
    (void)mpu_write_reg(MPU_REG_GYROCFG,0x00U);      /* ±250dps */
    (void)mpu_write_reg(MPU_REG_CONFIG, 0x03U);      /* DLPF ≈44Hz */
    (void)mpu_write_reg(MPU_REG_SMPLRT, 0x04U);      /* 200Hz 采样 */

    /* 兼容片：0x68/0x70/0x71/0x73/0x75/0x98 都认（和原工程同一张表） */
    switch (id)
    {
        case 0x68U: case 0x70U: case 0x71U: case 0x73U: case 0x75U: case 0x98U:
            g_imu_ok = 1U; break;
        default:
            g_imu_ok = 0U; break;
    }
}

uint8_t ctrl_imu_whoami (void) { return g_imu_who; }

void ctrl_imu_accel (int16_t *ax, int16_t *ay, int16_t *az)
{
    if (NULL != ax) { *ax = g_acc[0]; }
    if (NULL != ay) { *ay = g_acc[1]; }
    if (NULL != az) { *az = g_acc[2]; }
}

void ctrl_imu_gyro (int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (NULL != gx) { *gx = g_gyr[0]; }
    if (NULL != gy) { *gy = g_gyr[1]; }
    if (NULL != gz) { *gz = g_gyr[2]; }
}

/* 读 14 字节：ax ay az temp gx gy gz（大端） */
static void imu_tick (void)
{
    uint8_t b[14];

    if (0U == g_imu_ok) { return; }
    if (0U != mpu_read_regs(MPU_REG_ACCEL, b, 14U)) { return; }

    g_acc[0] = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    g_acc[1] = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    g_acc[2] = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
    g_gyr[0] = (int16_t)(((uint16_t)b[8] << 8) | b[9]);
    g_gyr[1] = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
    g_gyr[2] = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
}

//===================================================★ yaw（照抄原工程 yaw.c 的算法）===================================================
//   4 点滑动平均 → 零偏标定/扣除 → 80LSB 死区 → 20ms 积分。
//   K = SIGN × (TICK/1000) × SCALE / 131  ⇒ 每秒的度数。
static float    g_yaw_deg  = 0.0f;
static float    g_yaw_rate = 0.0f;
static int32_t  g_yaw_bias = 0;
static int32_t  g_cal_sum  = 0;
static uint16_t g_cal_n    = 0U;
static uint8_t  g_yaw_ready = 0U;
static int32_t  g_ybuf[CTRL_YAW_FILTER_N];
static uint8_t  g_yidx = 0U, g_yfill = 0U;

#define YAW_K   ((float)CTRL_YAW_SIGN * ((float)CTRL_YAW_TICK_MS / 1000.0f) \
                 * CTRL_YAW_SCALE / CTRL_YAW_LSB_PER_DPS)

static void yaw_feed (int16_t gz)
{
    int32_t s = 0;
    uint8_t i;

    g_ybuf[g_yidx] = gz;
    g_yidx = (uint8_t)((g_yidx + 1U) % CTRL_YAW_FILTER_N);
    if (g_yfill < CTRL_YAW_FILTER_N) { g_yfill ++; }

    for (i = 0U; i < g_yfill; i++) { s += g_ybuf[i]; }
    s /= (int32_t)g_yfill;

    if (0U == g_yaw_ready)
    {
        g_cal_sum += s;
        g_cal_n ++;
        if (g_cal_n >= CTRL_YAW_CALIB_N)
        {
            g_yaw_bias  = g_cal_sum / (int32_t)g_cal_n;
            g_yaw_ready = 1U;
        }
        g_yaw_deg  = 0.0f;
        g_yaw_rate = 0.0f;
        return;
    }

    s -= g_yaw_bias;
    if ((s < CTRL_YAW_DEADBAND_LSB) && (s > -CTRL_YAW_DEADBAND_LSB)) { s = 0; }
    g_yaw_rate = (float)s;
}

static void yaw_tick (void)
{
    if (0U == g_yaw_ready) { return; }
    g_yaw_deg += g_yaw_rate * YAW_K;
}

float   ctrl_yaw_deg   (void) { return g_yaw_deg; }
float   ctrl_yaw_rate  (void) { return g_yaw_rate / CTRL_YAW_LSB_PER_DPS * (float)CTRL_YAW_SIGN; }
uint8_t ctrl_yaw_ready (void) { return g_yaw_ready; }

void ctrl_yaw_reset (void)
{
    __disable_irq();
    g_yaw_deg = 0.0f;
    __enable_irq();
}

//===================================================★ 速度环（PI + 比例前馈 + 斜坡，参数照抄原工程）===================================================
#define SC_RAMP_STEP_X10    ((int32_t)CTRL_SPEED_RAMP_RPM_S * CTRL_PERIOD_MS / 100)   /* 3000*20/100 = 600 */

static int32_t g_sc_target_x10 = 0;
static int32_t g_sc_ramp_x10   = 0;
static int32_t g_sc_i_acc      = 0;
static int16_t g_sc_duty       = 0;
static int16_t g_sc_err_rpm    = 0;
static uint8_t g_sc_state      = 0U;
static int8_t  g_sc_sign       = 1;
static uint8_t g_sc_sign_locked = 0U;
static uint8_t g_sc_was_active  = 0U;      /* 上一拍速度环是不是在出力（用来只清一次电机） */

void ctrl_speed_init (void)
{
    g_sc_target_x10  = 0;
    g_sc_ramp_x10    = 0;
    g_sc_i_acc       = 0;
    g_sc_duty        = 0;
    g_sc_err_rpm     = 0;
    g_sc_state       = 0U;
    g_sc_sign        = 1;
    g_sc_sign_locked = 0U;
    ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, 0);
}

void ctrl_speed_set_target (int16_t rpm)
{
    g_sc_target_x10 = (int32_t)rpm * 10;
    g_sc_i_acc      = 0;                  /* 换目标就清积分（原工程同） */
}

int16_t ctrl_speed_get_target (void) { return (int16_t)(g_sc_target_x10 / 10); }
int16_t ctrl_speed_get_duty   (void) { return g_sc_duty; }
int16_t ctrl_speed_get_error  (void) { return g_sc_err_rpm; }
int16_t ctrl_speed_get_rpm    (void) { return (int16_t)(g_enc_rpm_x10 / 10); }
uint8_t ctrl_speed_get_state  (void) { return g_sc_state; }

const char * ctrl_speed_state_name (void)
{
    switch (g_sc_state)
    {
        case 0U: return "STOP";
        case 1U: return "RAMP";
        case 2U: return "RUN ";
        default: return "FAULT";
    }
}

/* ★★★ 2026-11-07（FW=2119）：**开环 / 闭环切换** ★★★
   现象：上位机用"占空比模式"发 `w 300 0`，电机没反应。
   原因：固件上电就自动闭环跑 500RPM，`speed_tick()` **每 20ms 都会写一次占空比**
         ⇒ 你发的值下一个节拍就被覆盖了。
   做法：收到 `w` ⇒ 进开环，速度环直接让位（什么都不写），电机只认你给的占空比；
        收到 `q`（含 `q 0` 停止）⇒ 回闭环。
   顺便：开环时堵转保护也不参与（没目标转速可比）。*/
static uint8_t g_open_loop = 0U;

void ctrl_open_loop (uint8_t on)
{
    g_open_loop = (0U != on) ? 1U : 0U;
}

uint8_t ctrl_is_open_loop (void) { return g_open_loop; }

static void speed_tick (void)
{
    int32_t meas_x10, err_x10, p_term, i_step, ff_term, out;

    if (0U != g_open_loop) { return; }      /* ★ 开环模式：速度环完全让位 */

    /* ★★★ 安全：**断链强制停车** ★★★
       上位机一关（或 WiFi 掉线），板子不知道，会按最后一次目标转速一直跑 —— 很危险。
       `wifi_f4_is_ready()` 在链路断开/发送失败时会被驱动清 0（遥测 100ms 一次，
       所以最多几百 ms 就能发现）⇒ 这里直接把目标清 0，走下面的停机分支。*/
    if (0U == wifi_f4_is_ready())
    {
        g_sc_target_x10 = 0;
    }

    /* 目标 0 ⇒ 停机（原工程同）。
       ⚠️ 只在"从出力变回停机"的那一拍写一次 0：否则每 20ms 都写一遍，
          会把 `w` 命令开环给的占空比立刻冲掉，没法做开环测试。 */
    if (0 == g_sc_target_x10)
    {
        g_sc_ramp_x10 = 0;
        g_sc_i_acc    = 0;
        g_sc_duty     = 0;
        g_sc_err_rpm  = 0;
        g_sc_state    = 0U;
        if (0U != g_sc_was_active)
        {
            g_sc_was_active = 0U;
            ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, 0);
        }
        return;
    }
    g_sc_was_active = 1U;

    /* 设定值斜坡 */
    if (g_sc_ramp_x10 < g_sc_target_x10)
    {
        g_sc_ramp_x10 += SC_RAMP_STEP_X10;
        if (g_sc_ramp_x10 > g_sc_target_x10) { g_sc_ramp_x10 = g_sc_target_x10; }
    }
    else if (g_sc_ramp_x10 > g_sc_target_x10)
    {
        g_sc_ramp_x10 -= SC_RAMP_STEP_X10;
        if (g_sc_ramp_x10 < g_sc_target_x10) { g_sc_ramp_x10 = g_sc_target_x10; }
    }

    meas_x10 = g_enc_rpm_x10;

    /* ★★★ 2026-11-07（FW=2117）：**测速一阶低通** ★★★
       日志实测：占空比很稳（2824~3212），但 rpm 在 466~526 之间慢慢摆（周期约 2s）。
       20ms 窗口里编码器量化的 ±1 个计数 ≈ ±7RPM，会被 P 项放大成可见的摆动。
       这里做 y += (x-y)/4（时间常数约 4 拍 = 80ms）：对真实加减速几乎无延迟，
       但把量化噪声压掉大半，速度环就不会再"追噪声"。*/
    {
        static int32_t meas_f_x10 = 0;
        meas_f_x10 += (meas_x10 - meas_f_x10) / 4;
        meas_x10 = meas_f_x10;
    }

#if CTRL_SPEED_SIGN_AUTO
    /* 反馈极性自动识别：上一拍给了明显正占空比，实测却是明显负转速 ⇒ 反了 */
    if ((0U == g_sc_sign_locked) && (g_sc_duty > 800))
    {
        if (meas_x10 < -2000)      { g_sc_sign = -1; g_sc_sign_locked = 1U; }
        else if (meas_x10 > 2000)  { g_sc_sign =  1; g_sc_sign_locked = 1U; }
    }
#endif
    meas_x10 *= (int32_t)g_sc_sign;

    err_x10    = g_sc_ramp_x10 - meas_x10;
    g_sc_err_rpm = (int16_t)(err_x10 / 10);

    p_term = (err_x10 * CTRL_SPEED_KP_X1000) / 10000;
    i_step = (err_x10 * CTRL_SPEED_KI_X1000) / 10000;

    g_sc_i_acc += i_step;
    if (g_sc_i_acc >  CTRL_SPEED_I_LIMIT) { g_sc_i_acc =  CTRL_SPEED_I_LIMIT; }
    if (g_sc_i_acc < -CTRL_SPEED_I_LIMIT) { g_sc_i_acc = -CTRL_SPEED_I_LIMIT; }

    ff_term = (g_sc_ramp_x10 * CTRL_SPEED_FF_PER_1000) / 10000;

    out = ff_term + p_term + g_sc_i_acc;
    if (out > CTRL_SPEED_DUTY_MAX) { out = CTRL_SPEED_DUTY_MAX; }
    if (out < 0)                   { out = 0; }

    g_sc_duty = (int16_t)out;
    ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, g_sc_duty);

    g_sc_state = (g_sc_ramp_x10 < g_sc_target_x10) ? 1U : 2U;
}

//===================================================★ OLED（SSD1306 128×64，软 I2C：PB10=SCL PB11=SDA）===================================================
//   · 4 针模块（VCC/GND/SCL/SDA），I2C 地址一般是 **0x3C**（有的批次 0x3D，初始化时两个都试）
//   · 和 MPU9250 一样走软件 I2C（本工程没开 HAL 的 I2C 模块），但**另一对引脚**（I2C2：PB10/PB11）
//   · **不占 RAM 显存**：直接按"页(page)+列"写屏，所以没有 1KB framebuffer
//     （这个工程 RAM 只剩 2.4KB，能省就省）
//   · 全屏清一次 = 8 页 × 128 字节 ≈ 37ms（250kHz 软 I2C）⇒ **只在开机做一次**，主循环里不刷全屏
#define OLED_SCL_H()   (GPIOB->BSRR = GPIO_PIN_10)
#define OLED_SCL_L()   (GPIOB->BSRR = (uint32_t)GPIO_PIN_10 << 16)
#define OLED_SDA_H()   (GPIOB->BSRR = GPIO_PIN_11)
#define OLED_SDA_L()   (GPIOB->BSRR = (uint32_t)GPIO_PIN_11 << 16)
#define OLED_ADDR_A    0x3CU
#define OLED_ADDR_B    0x3DU

static uint8_t g_oled_addr = OLED_ADDR_A;
static uint8_t g_oled_ok   = 0U;        /* 1 = 总线上真有 OLED（初始化时探测的应答）*/

/* 5×7 ASCII 字体（列优先，bit0 = 最上面一行），只放 32~126 共 95 个字符 */
static const uint8_t oled_font5x7[95][5] =
{
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40}, {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00}, {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00}, {0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00}, {0x08,0x08,0x2A,0x1C,0x08}
};

static void oled_delay (void) { ctrl_delay_us(CTRL_OLED_I2C_DELAY_US); }

static void i2c2_start (void)
{
    OLED_SDA_H(); OLED_SCL_H(); oled_delay();
    OLED_SDA_L(); oled_delay();
    OLED_SCL_L(); oled_delay();
}

static void i2c2_stop (void)
{
    OLED_SDA_L(); oled_delay();
    OLED_SCL_H(); oled_delay();
    OLED_SDA_H(); oled_delay();
}

static uint8_t i2c2_write (uint8_t b)
{
    uint8_t i, ack;

    for (i = 0U; i < 8U; i++)
    {
        if (0U != (b & 0x80U)) { OLED_SDA_H(); } else { OLED_SDA_L(); }
        b = (uint8_t)(b << 1);
        oled_delay();
        OLED_SCL_H(); oled_delay();
        OLED_SCL_L(); oled_delay();
    }
    OLED_SDA_H(); oled_delay();
    OLED_SCL_H(); oled_delay();
    ack = (0U == (GPIOB->IDR & GPIO_PIN_11)) ? 0U : 1U;
    OLED_SCL_L(); oled_delay();
    return ack;
}

/* 探测某个地址有没有应答（0 = 有） */
static uint8_t i2c2_probe (uint8_t addr)
{
    uint8_t ack;

    i2c2_start();
    ack = i2c2_write((uint8_t)(addr << 1));
    i2c2_stop();
    return ack;
}

/* 发控制字节 + 一串命令/数据 */
static void oled_send (uint8_t ctrl, const uint8_t *p, uint16_t n)
{
    uint16_t i;

    i2c2_start();
    if (0U != i2c2_write((uint8_t)(g_oled_addr << 1))) { i2c2_stop(); return; }
    (void)i2c2_write(ctrl);
    for (i = 0U; i < n; i++) { (void)i2c2_write(p[i]); }
    i2c2_stop();
}

static void oled_cmds (const uint8_t *c, uint8_t n) { oled_send(0x00U, c, n); }

/* 定位于某页某列（页 = 8 行；列 0..127） */
static void oled_set_pos (uint8_t x, uint8_t page)
{
    uint8_t c[3];

    c[0] = (uint8_t)(0xB0U | (page & 0x07U));       /* 页地址 */
    c[1] = (uint8_t)(x & 0x0FU);                    /* 列低 4 位 */
    c[2] = (uint8_t)(0x10U | ((x >> 4) & 0x0FU));   /* 列高 4 位 */
    oled_cmds(c, 3U);
}

void ctrl_oled_clear (void)
{
    static const uint8_t zero[128] = {0};
    uint8_t page;

    if (0U == g_oled_ok) { return; }

    for (page = 0U; page < 8U; page++)
    {
        oled_set_pos(0U, page);
        oled_send(0x40U, zero, 128U);
    }
}

/* 在 (x, page) 用 5×7 字体写字符串；scale = 1（6×8）或 2（12×16，占 2 页） */
void ctrl_oled_text (uint8_t x, uint8_t page, const char *s, uint8_t scale)
{
    uint8_t  col[24];          /* 最多 12 列 × 2 页 = 24 字节 */
    uint8_t  n;
    uint8_t  ch;
    uint16_t bits;
    uint8_t  r;

    if (0U == scale) { scale = 1U; }

    while ((0 != *s) && (x < 128U))
    {
        ch = (uint8_t)((uint8_t)*s - 32U);
        if (ch > 94U) { ch = 0U; }

        /* 展开成 scale 倍宽、scale 倍高（最多 12 列 × 16 行）*/
        for (n = 0U; n < (uint8_t)(6U * scale); n++)
        {
            uint8_t g = (uint8_t)(n / scale);
            uint8_t v = (g < 5U) ? oled_font5x7[ch][g] : 0U;

            bits = 0U;
            for (r = 0U; r < 8U; r++)
            {
                if (0U != (v & (uint8_t)(1U << r)))
                {
                    bits |= (uint16_t)(3U << (2U * r));
                }
            }
            if (1U == scale)
            {
                col[n] = (uint8_t)(v);
            }
            else
            {
                col[n] = (uint8_t)(bits & 0xFFU);
                if (page < 7U) { col[(uint8_t)(12U + n)] = (uint8_t)((bits >> 8) & 0xFFU); }
            }
        }

        if (1U == scale)
        {
            oled_set_pos(x, page);
            oled_send(0x40U, col, 6U);
            x = (uint8_t)(x + 6U);
        }
        else
        {
            oled_set_pos(x, page);
            oled_send(0x40U, col, 12U);
            if (page < 7U)
            {
                oled_set_pos(x, (uint8_t)(page + 1U));
                oled_send(0x40U, &col[12], 12U);
            }
            x = (uint8_t)(x + 12U);
        }

        s ++;
        if (x > 120U) { break; }
    }
}

void ctrl_oled_init (void)
{
    static const uint8_t init_seq[] =
    {
        0xAE,            /* 关显示 */
        0xD5, 0x80,      /* 时钟分频 */
        0xA8, 0x3F,      /* 复用率 1/64 */
        0xD3, 0x00,      /* 显示偏移 0 */
        0x40,            /* 起始行 0 */
        0x8D, 0x14,      /* ★ 电荷泵开（4 针模块必须开，否则不亮） */
        0x20, 0x00,      /* 水平寻址模式 */
        0xA1,            /* 左右不反 */
        0xC8,            /* 上下不反 */
        0xDA, 0x12,      /* COM 引脚 */
        0x81, 0xCF,      /* 对比度 */
        0xD9, 0xF1,      /* 预充电 */
        0xDB, 0x40,      /* VCOMH */
        0xA4,            /* 显示跟随 RAM */
        0xA6,            /* 正常显示 */
        0xAF             /* 开显示 */
    };
    GPIO_InitTypeDef gi;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gi.Mode      = GPIO_MODE_OUTPUT_OD;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = 0;
    HAL_GPIO_Init(GPIOB, &gi);
    OLED_SCL_H(); OLED_SDA_H();
    ctrl_delay_us(50000U);          /* SSD1306 手册要求上电后等一会儿再发命令（开机只做一次） */

    /* 先试 0x3C；不应答再试 0x3D；两个都不应答 ⇒ **没有 OLED**，后面一律不刷（省时间）*/
    g_oled_addr = OLED_ADDR_A;
    if (0U != i2c2_probe(g_oled_addr))
    {
        g_oled_addr = OLED_ADDR_B;
        if (0U != i2c2_probe(g_oled_addr))
        {
            g_oled_ok = 0U;
            return;
        }
    }
    g_oled_ok = 1U;

    oled_cmds(init_seq, (uint8_t)sizeof(init_seq));
}

uint8_t ctrl_oled_ok (void) { return g_oled_ok; }

void ctrl_oled_hello (void)
{
    /* 2 倍字：hello 一行、world 一行，各 5 字符 × 12 = 60 像素，居中 */
    if (0U == g_oled_ok) { return; }
    ctrl_oled_text((uint8_t)((128U - 60U) / 2U), 2U, "hello", 2U);
    ctrl_oled_text((uint8_t)((128U - 60U) / 2U), 4U, "world", 2U);
}

//===================================================★ OLED 状态页（WiFi / MPU / 编码器 / 转速）===================================================
//   布局（2 倍字，每行占两页，共 4 行 = 8 页）：
//       page 0-1  WiFi OK / E<n>    page 2-3  MPU OK / <whoami>
//       page 4-5  ENC 01234         page 6-7  RPM  123
//   ★ 只在**内容变了**的时候重画，而且**每个 20ms 控制拍只重画一行**：
//     一行 2 页 ≈ 8.6ms（250kHz 软 I2C）⇒ 摊开后主循环最多被按 8.6ms，
//     摄像头环形缓冲能存 80ms，不会丢帧。
//   ⚠️ 2026-11-05 踩坑：第一版让"hello world"先显示 1 秒再由主循环切到状态页，
//      但 `ctrl_all_init()` 是在 `wifi_f4_init()` **之前**跑的，而后者最长会阻塞几十秒
//      ⇒ 这段时间里主循环根本没转 ⇒ 屏幕一直停在 hello。
//      ⇒ 现在**开机就直接画状态页**（`ctrl_oled_first_page()`，只在开机画一次），不再等主循环。
static char     g_oled_line[4][20];
static uint8_t  g_oled_dirty  = 0U;
static uint32_t g_oled_t_txt  = 0U;

static void oled_line_set (uint8_t i, const char *s)
{
    if (0 != strcmp(s, g_oled_line[i]))
    {
        strncpy(g_oled_line[i], s, 19U);
        g_oled_line[i][19] = 0;
        g_oled_dirty = (uint8_t)(g_oled_dirty | (uint8_t)(1U << i));
    }
}

/* 把一行补空格到 10 个字符（120 像素）：
   ① 2 倍字 10 字符正好 120 < 128，放得下；
   ② 长度恒定 ⇒ 上一次长一点的残留（比如 "ENC 12345" 变 "ENC 00042"）
      一定会被空格覆盖掉，不会留鬼影。 */
static void oled_pad (char *b)
{
    uint8_t n = (uint8_t)strlen(b);

    while (n < 10U) { b[n] = ' '; n ++; }
    b[n] = 0;
}

/* ★ 电机巡航/堵转状态（定义在这里、给下面的 OLED 和 motor_supervisor 共用）*/
static uint8_t  s_motor_started = 0U;      /* 0 = 还没起过转 */
static uint8_t  s_motor_stalled = 0U;      /* 1 = 堵转停机，等 PC2 */
#if CTRL_STALL_ENABLE
static uint16_t s_below_cnt     = 0U;      /* 连续低于阈值拍数（保护关掉时不需要）*/
#endif
static uint8_t  s_btn_armed     = 0U;      /* 进堵转时已记住按键电平 */
static uint8_t  s_key_prev      = 0U;      /* 进堵转那一刻的按键电平（PC2/PC3）*/

/* 重新生成 4 行文字（内容变了才标 dirty） */
static void oled_text_update (void)
{
    static const char spin[4] = { '-', '\\', '|', '/' };
    char     b[20];
    char     w[4];
    char     m[4];
    uint32_t now = HAL_GetTick();

    /* ---- 第 0 行：WiFi / MPU 一键看（"W:OK M:OK" 正好 10 个字符）---- */
    if (0U != wifi_f4_is_ready())      { strcpy(w, "OK"); }
    else if (0U == wifi_f4_last_err()) { strcpy(w, "--"); }      /* 还在初始化 */
    else                               { sprintf(w, "E%u", (unsigned)wifi_f4_last_err()); }

    if (0U != g_imu_ok) { strcpy(m, "OK"); }
    else                { sprintf(m, "%02X", (unsigned)g_imu_who); }

    sprintf(b, "W:%s M:%s", w, m);
    oled_pad(b); oled_line_set(0U, b);

    /* ---- 第 1 行：★ 电机状态（2026-11-07 FW=2112 新增）----
         RUN 500   = 闭环巡航在 500RPM
         STALL PC2 = 堵转停机，**按 PC2 重新起转**
         STOP      = 没在跑（目标是 0）*/
    {
        int16_t tgt = ctrl_speed_get_target();
        int32_t at  = (tgt < 0) ? -(int32_t)tgt : (int32_t)tgt;

        if (0U != s_motor_stalled)                { sprintf(b, "STALL PC2"); }
        else if ((0U != s_motor_started) && (at > 0)) { sprintf(b, "RUN %4ld", (long)at); }
        else                                      { sprintf(b, "STOP"); }
    }
    oled_pad(b); oled_line_set(1U, b);

    sprintf(b, "ENC %05ld", (long)(g_enc_total % 100000L));
    oled_pad(b); oled_line_set(2U, b);

    /* ★★ 2026-11-07（FW=2100）：行尾这个会**转的小符号** = "主循环还活着"的铁证 ★★
       以前 RPM 一直是 0、ENC 不动的时候，屏幕看着就像"卡死"，根本分不清是
       "主循环停了"还是"只是数值没变所以没重画"。现在它每 250ms 变一次
       （一行重画 ≈4ms ⇒ 约 1.6% CPU），**停了就是真停了**。*/
    sprintf(b, "RPM %4d %c", (int)(g_enc_rpm_x10 / 10), spin[(now / 250U) & 3U]);
    oled_pad(b); oled_line_set(3U, b);
}

/* ★ 开机立刻把 4 行全画出来（约 34ms，只在 `ctrl_all_init()` 里调一次）
   —— 不能等主循环：`wifi_f4_init()` 可能阻塞几十秒 */
void ctrl_oled_first_page (void)
{
    uint8_t i;

    if (0U == g_oled_ok) { return; }

    oled_text_update();
    for (i = 0U; i < 4U; i++)
    {
        ctrl_oled_text(0U, (uint8_t)(i * 2U), g_oled_line[i], 2U);
    }
    g_oled_dirty  = 0U;
    g_oled_t_txt  = HAL_GetTick();
}

/* ★★★ 2026-11-07（FW=2101）：**启动阶段显示** ★★★
   开机到主循环之间有两段长时间的阻塞：`ctrl_servo_scan()`（1~8 秒）和
   `wifi_f4_init()`（最长 60 秒）。这两段里屏幕**本来就是静止的** ⇒
   "卡在这里"和"真的死机"看起来完全一样。现在每过一个阶段就在第 0 行写 `BOOT n`，
   卡住时屏幕上就停在那一步的编号 —— 不用连电脑也能看。
     1 = 进 ctrl_all_init（OLED 刚点亮）
     2 = ctrl_all_init 完成（舵机扫描也做完了）
     3 = 正在 wifi_f4_init（连热点/连上位机，最长 60 秒）
     4 = wifi_f4_init 返回，准备进主循环
     9 = 正在扫舵机 ID（ctrl_servo_scan 内部）
   主循环一起来，第 0 行会被状态页重新盖掉（这里故意把缓存清空，保证一定重画）。*/
void ctrl_oled_boot (uint8_t stage)
{
    char b[20];

    if (0U == g_oled_ok) { return; }

    sprintf(b, "BOOT %u", (unsigned)stage);
    oled_pad(b);
    ctrl_oled_text(0U, 0U, b, 2U);
    g_oled_line[0][0] = '\0';                /* 让状态页一定把它重画掉 */
    g_oled_dirty = (uint8_t)(g_oled_dirty & (uint8_t)~1U);
}

void ctrl_oled_status (void)
{
    uint32_t now = HAL_GetTick();

    if (0U == g_oled_ok) { return; }
    if ((now - g_oled_t_txt) >= CTRL_OLED_TEXT_MS)           /* 10Hz 更新文字 */
    {
        g_oled_t_txt = now;
        oled_text_update();
    }

    if (0U != g_oled_dirty)                                  /* 有脏行就一次补完（最多 4 行 ≈ 16ms）*/
    {
        uint8_t i;

        for (i = 0U; i < 4U; i++)
        {
            if (0U != (g_oled_dirty & (uint8_t)(1U << i)))
            {
                g_oled_dirty = (uint8_t)(g_oled_dirty & (uint8_t)~(uint8_t)(1U << i));
                ctrl_oled_text(0U, (uint8_t)(i * 2U), g_oled_line[i], 2U);
            }
        }
    }
}

//===================================================★ 按键 / 蜂鸣器===================================================
void ctrl_io_init (void)
{
    GPIO_InitTypeDef gi;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_2 | GPIO_PIN_3;         /* 按键：输入上拉，按下 = 低 */
    gi.Mode      = GPIO_MODE_INPUT;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = 0;
    HAL_GPIO_Init(GPIOC, &gi);

    gi.Pin  = GPIO_PIN_1;                           /* 蜂鸣器 */
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gi);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

uint8_t ctrl_key (uint8_t idx)
{
    if (1U == idx) { return (0U == (GPIOC->IDR & GPIO_PIN_2)) ? 1U : 0U; }
    if (2U == idx) { return (0U == (GPIOC->IDR & GPIO_PIN_3)) ? 1U : 0U; }
    return 0U;
}

void ctrl_buzzer (uint8_t on)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (0U != on) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//===================================================★ 初始化 / 控制节拍===================================================

/* ★★★ 2026-11-07（FW=2112）：**电机闭环巡航 + 防堵转 + PC2 重启** ★★★
   用户要求：
     · 原来的"开环占空比"改成 **闭环 500 RPM**；
     · 转速连续 **2 秒**低于目标（这里取目标的 80%）⇒ 判堵转，**停机**；
     · 停机后 **PC2 低电平** 重新起转；
     · OLED 显示 RUN / STALL PC2 / STOP。
   为什么用 80% 而不是"低于目标"：PI 稳态本来就允许一点误差（比如稳在 495），
   严格按"低于目标"会把正常工作误判成堵转。真堵转时转速基本是 0，用 80% 足够灵敏。*/

static void motor_supervisor (void)
{
    /* ① 自动起转（仅闭环、只做一次）*/
    if ((0U == g_open_loop) && (0U == s_motor_started))
    {
        s_motor_started = 1U;
#if CTRL_RUN_AUTO_START
        ctrl_speed_set_target((int16_t)CTRL_RUN_RPM);
#endif
    }

    /* ② 堵转停机后：**任意按键（PC2 / PC3）新按下** ⇒ 回闭环重新起转
       ★ 2026-11-07（FW=2121）：实测遥测 `k1` **242 条全是 0** —— PC2 一直是低电平
         （按键卡住 / 常闭 / 板上有下拉电阻），"先松开再按下"的边沿永远等不到
         ⇒ 按 PC2 毫无反应。改成：
           · 进堵转时记住当时的电平；
           · 之后只要有任意一次电平变化、并且变成"有键按下"就重启；
           · PC2 卡死不动也没关系，按 PC3 一样能重启。*/
    if (0U != s_motor_stalled)
    {
        uint8_t k = (uint8_t)(((0U != ctrl_key(1U)) ? 1U : 0U) |
                              ((0U != ctrl_key(2U)) ? 2U : 0U));

        if (0U == s_btn_armed)
        {
            s_key_prev  = k;          /* 刚进堵转：记住当前电平（卡住的键不会误触发）*/
            s_btn_armed = 1U;
        }
        else if (k != s_key_prev)
        {
            s_key_prev = k;
            if (0U != k)              /* 变成"有键按下" ⇒ 重启 */
            {
                s_motor_stalled = 0U;
                s_btn_armed     = 0U;
#if CTRL_STALL_ENABLE
                s_below_cnt     = 0U;
#endif
                ctrl_open_loop(0U);                                /* 回闭环巡航 */
                ctrl_speed_set_target((int16_t)CTRL_RUN_RPM);
            }
        }
        return;
    }
    s_btn_armed = 0U;                 /* 没堵转时复位，下次堵转重新记住电平 */

    /* ③ ★★★ 全局堵转判据（FW=2120，用户要求，**不再分模式/不再看目标**）★★★
          规则只有一条：**实测转速 < CTRL_STALL_RPM(400) 持续 CTRL_STALL_MS(2 秒) ⇒ 关电机**
          · 闭环 500、开环占空比、甚至上位机没连 —— 全都一样判；
          · 唯一例外：电机本来就没驱动（占空比 0 且目标 0）⇒ 那就是"停着的"，不算堵转；
          · 关掉电机后进入 STALL 状态，等 PC2（先松开再按下）重新起转。*/
#if CTRL_STALL_ENABLE
    {
        int16_t duty  = ctrl_motor_get((uint8_t)CTRL_SPEED_MOTOR);
        int16_t tgt   = ctrl_speed_get_target();
        int16_t rpm   = ctrl_speed_get_rpm();
        int32_t a_rpm = (rpm < 0) ? -(int32_t)rpm : (int32_t)rpm;

        /* 根本没在驱动 ⇒ 不算堵转（这就是"已经停了"）*/
        if ((0 == duty) && (0 == tgt))
        {
            s_below_cnt = 0U;
            return;
        }

        if (a_rpm < (int32_t)CTRL_STALL_RPM)
        {
            if (s_below_cnt < 0xFFFFU) { s_below_cnt ++; }

            if (s_below_cnt >= (uint16_t)CTRL_STALL_TICKS)
            {
                s_motor_stalled = 1U;
                s_below_cnt     = 0U;
                ctrl_open_loop(0U);                             /* 退回闭环，便于 PC2 重启 */
                ctrl_speed_set_target(0);                       /* ★ 关电机 */
                ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, 0);   /* 开环时也确保真的关 */
            }
        }
        else
        {
            s_below_cnt = 0U;
        }
    }
#endif  /* CTRL_STALL_ENABLE */
}

/* ★ 2026-11-07（FW=2121）：上位机发 `q <非零>` 时清掉"堵转停机"状态 ——
   这样即使按键坏了/没接，也能从面板上把电机重新起转。*/
void ctrl_motor_clear_stall (void)
{
    s_motor_stalled = 0U;
    s_btn_armed     = 0U;
#if CTRL_STALL_ENABLE
    s_below_cnt     = 0U;
#endif
}

void ctrl_all_init (void)
{
    ctrl_dwt_init();

    ctrl_io_init();          /* 蜂鸣器/按键最先，上电不叫 */
    ctrl_motor_init();       /* 电机 PWM=0、DIR=0，不会突然转 */
    ctrl_encoder_init();
    ctrl_imu_init();

    /* OLED：清屏 + **立刻画状态页**（约 40ms，只在开机做一次）
       ⚠️ 必须在这里画，不能等主循环：`ctrl_all_init()` 之后紧接着就是
          `wifi_f4_init()`，它最长会阻塞几十秒，那段时间主循环根本不在转。 */
    ctrl_oled_init();
    ctrl_oled_clear();
    ctrl_oled_first_page();
    ctrl_oled_boot(1U);      /* ★ 开机阶段 1：OLED 已点亮 */

    /* ★ 舵机放最后：它上电要 ~1 秒才肯说话，`ctrl_servo_init()` 里会等 1 秒再扫 ID
       ⇒ 先把屏幕点亮，别让这一秒黑着。 */
    ctrl_servo_init();
    ctrl_oled_boot(2U);      /* ★ 开机阶段 2：车模控制全部初始化完成 */

    ctrl_speed_init();
}

void board_ctrl_tick (void)
{
    static uint32_t t_last     = 0U;
    static uint32_t t_last_cyc = 0U;    /* ★ DWT 周期计数：给测速算 µs 级的真实间隔 */
    static uint8_t  srv_div = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - t_last) < CTRL_PERIOD_MS) { return; }

    /* ★★★ 2026-11-07（FW=2116）：**用实际经过的时间算转速** ★★★
       现场数据（上位机记录文件）：占空比稳在 2700 左右，但转速读数 405~519 乱跳，
       平均值只有 430 —— 不是控制环在振荡，是**测速本身不准**：
       原来 RPM 固定按 20ms 算，可主循环会被图传发送、舵机同步读（各 1~3ms）拖慢，
       实际周期常常是 22~30ms ⇒ 同一个增量被当成 20ms 的 ⇒ 读数忽高忽低。
       ⇒ 这里把真实间隔 dt 传进 encoder_tick()，用它算 RPM。*/
    {
        uint32_t cyc   = DWT->CYCCNT;
        uint32_t dt_us = (uint32_t)((cyc - t_last_cyc) / (SystemCoreClock / 1000000UL));

        t_last_cyc = cyc;
        t_last     = now;

        if (0U == dt_us)         { dt_us = (uint32_t)CTRL_PERIOD_MS * 1000U; }
        if (dt_us > 200000UL)    { dt_us = 200000UL; }      /* 卡太久就夹住 */

        /* 1) 编码器：取增量 + 用**DWT 实测的 µs 间隔**算转速 */
        encoder_tick_us(dt_us);
    }

    /* 2) 速度环（只在目标非 0 时才会有输出） */
    speed_tick();

    /* 2.5) ★ 闭环巡航 + 防堵转 + PC2 重启（见 motor_supervisor 的说明） */
    motor_supervisor();

    /* 3) IMU：每拍读一次（50Hz），读出陀螺就喂 yaw */
    imu_tick();
    if (0U != g_imu_ok)
    {
        yaw_feed(g_gyr[2]);
        yaw_tick();
    }

    /* 4) 舵机（★ 2026-11-07 改成和 MSPM0 原工程一样的**同步轮询**，没有中断）
          · 上电先等 50 拍（1 秒）—— 总线舵机自己也要初始化，太早说话它不理
          · 然后写一次目标速度（0x2E 是保持型寄存器），再摆到目标位置
          · 之后：目标变了才发（发完立刻等应答，约 1~2ms）；空闲时每 10 拍（200ms）读一次位置
          ⚠️ 一处失败（没应答）不会重试 —— 与原来的行为一致；下一次目标变化还会再发。*/
    /* ★★★ 2026-11-07（FW=2107）：一次合法应答都没收到过 ⇒ 每 3 秒换一档波特率再 PING ★★★
       HX-10HM 的波特率在手册里有两种说法（1M / 115200），与其猜，不如两档都试。
       一旦收到应答（g_srv_rx_cnt > 0）就停，不再打扰总线。
       每次 PING 最多 ~2ms，靠同步轮询，不用中断，不会卡。*/
    if ((0U == g_srv_rx_cnt) && (g_servo_boot >= 50U))
    {
        static uint32_t t_baud = 0U;
        static uint8_t  alt    = 0U;

        if ((HAL_GetTick() - t_baud) > 3000U)
        {
            uint8_t err = 0xFFU;

            t_baud = HAL_GetTick();
            alt    = (uint8_t)(alt ^ 1U);
            servo_set_baud((0U != alt) ? CTRL_SERVO_BAUD_ALT : CTRL_SERVO_BAUD);
            (void)servo_ping(g_servo_id, &err);
            g_servo_err = err;
        }
    }

    if (g_servo_boot < 50U)
    {
        g_servo_boot ++;
    }
    else if (0U == g_servo_speed_ok)
    {
        if (0U == servo_set_speed((int16_t)CTRL_SERVO_SPEED)) { g_servo_speed_ok = 1U; }
    }
    else if (g_servo_target != g_servo_last_set)
    {
        (void)ctrl_servo_set_pos(g_servo_target);
        g_servo_last_set = g_servo_target;      /* 无论成功与否都记下来，别每拍重发 */
    }
    else
    {
        srv_div ++;
        if (srv_div >= 10U) { srv_div = 0U; servo_read_pos(); }
    }

    /* 5) OLED 状态页（没有 OLED 就整个跳过） */
    ctrl_oled_status();
}
