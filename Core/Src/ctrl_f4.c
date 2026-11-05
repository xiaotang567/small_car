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
static void encoder_tick (void)
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

    /* RPM×10 = ΔN × 600000 / (CPR × 周期ms)   —— 必须 int64，原工程踩过 int32 溢出 */
    g_enc_rpm_x10 = (int16_t)(((int64_t)d * 600000LL) /
                              ((int64_t)CTRL_ENC_CPR * (int64_t)CTRL_PERIOD_MS));
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

//===================================================★ 幻尔总线舵机（UART4，1Mbps，非阻塞）===================================================
//   原工程是**轮询死等回包**（一次可能几十 ms）—— 搬到这里会把摄像头主循环按住 ⇒ 全套改非阻塞：
//     · 发送：9 字节指令塞进 64 字节环形缓冲，TXE 中断自己往外吐（一帧约 90µs，主循环不等）
//     · 接收：RXNE 中断塞缓冲，20ms 节拍里解析（找不到合法帧就当没有，绝不死等）
//   帧格式（幻尔）：FF FF ID LEN INSTR P.. CS，CS = ~(ID+LEN+INSTR+P..) & 0xFF
#define SERVO_TX_SIZE       64U
#define SERVO_RX_SIZE       64U
#define SERVO_INSTR_PING    0x01U
#define SERVO_INSTR_READ    0x02U
#define SERVO_INSTR_WRITE   0x03U
#define SERVO_REG_POS       0x2AU       /* 目标位置（2 字节小端） */
#define SERVO_REG_SPEED     0x2EU       /* 目标速度（2 字节，符号-幅值） */
#define SERVO_REG_CURPOS    0x38U       /* 当前位置（2 字节小端） */

static uint8_t  g_stx[SERVO_TX_SIZE];
static volatile uint16_t g_stx_head = 0U, g_stx_tail = 0U;
static uint8_t  g_srx[SERVO_RX_SIZE];
static volatile uint16_t g_srx_head = 0U, g_srx_tail = 0U;

static uint16_t g_servo_pos     = 0xFFFFU;
static uint8_t  g_servo_err     = 0U;
static uint32_t g_servo_tx_cnt  = 0U;
static uint32_t g_servo_rx_cnt  = 0U;
static uint16_t g_servo_target  = CTRL_SERVO_POS_CENTER;
static uint16_t g_servo_last_set = 0xFFFFU;     /* 0xFFFF = 还没发过 ⇒ 第一拍就把舵机摆到中位 */
static uint8_t  g_servo_speed_ok = 0U;          /* 目标速度（0x2E）是保持型寄存器，上电写一次 */
static uint8_t  g_servo_boot     = 0U;          /* 上电等 1 秒再跟舵机说话（它自己要初始化） */
static uint8_t  g_last_tx[16];          /* 最近一次发出去的帧（用来丢"自己的回显"）*/
static uint8_t  g_last_tx_len = 0U;

static uint8_t servo_checksum (const uint8_t *p, uint8_t n)
{
    uint8_t s = 0U, i;

    for (i = 0U; i < n; i++) { s = (uint8_t)(s + p[i]); }
    return (uint8_t)(~s);
}

/* 把一帧塞进发送环（满了直接丢弃，绝不死等） */
static uint8_t servo_queue (const uint8_t *frame, uint8_t n)
{
    uint16_t next;
    uint8_t  i;

    for (i = 0U; i < n; i++)
    {
        next = (uint16_t)((g_stx_head + 1U) % SERVO_TX_SIZE);
        if (next == g_stx_tail) { return 1U; }        /* 环满 */
        g_stx[g_stx_head] = frame[i];
        g_stx_head = next;
    }

    memcpy(g_last_tx, frame, n);
    g_last_tx_len = n;

    UART4->CR1 |= USART_CR1_TXEIE;
    g_servo_tx_cnt ++;
    return 0U;
}

void ctrl_servo_init (void)
{
    GPIO_InitTypeDef gi;

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gi.Pin       = GPIO_PIN_10 | GPIO_PIN_11;       /* PC10=TX PC11=RX，AF8 */
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gi);

    UART4->CR1 = 0U;
    /* 1Mbps：APB1=42MHz，42e6/1e6 = 42 ⇒ mantissa=42, fraction=0 ⇒ BRR=0x2A0（正好整除）*/
    UART4->BRR = (uint16_t)((CTRL_APB1_HZ_LOCAL / CTRL_SERVO_BAUD) << 4);
    UART4->CR1 = USART_CR1_UE;
    UART4->CR1 = (uint16_t)(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
    UART4->CR1 |= USART_CR1_RXNEIE;

    HAL_NVIC_SetPriority(UART4_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    g_stx_head = g_stx_tail = 0U;
    g_srx_head = g_srx_tail = 0U;
    g_servo_pos = 0xFFFFU;
}

uint8_t ctrl_servo_set_pos (uint16_t pos)
{
    uint8_t f[9];

    if (pos < CTRL_SERVO_POS_MIN) { pos = CTRL_SERVO_POS_MIN; }
    if (pos > CTRL_SERVO_POS_MAX) { pos = CTRL_SERVO_POS_MAX; }

    f[0] = 0xFFU; f[1] = 0xFFU;
    f[2] = (uint8_t)CTRL_SERVO_ID;
    f[3] = 5U;                                      /* Length = 参数 2 + 3 */
    f[4] = SERVO_INSTR_WRITE;
    f[5] = SERVO_REG_POS;
    f[6] = (uint8_t)(pos & 0xFFU);
    f[7] = (uint8_t)(pos >> 8);
    f[8] = servo_checksum(&f[2], 6U);               /* ID..D1 共 6 字节 */

    g_servo_target = pos;
    if (0U == servo_queue(f, 9U))
    {
        g_servo_last_set = pos;
        return 0U;
    }
    return 1U;                                      /* 发送环满 ⇒ 下一拍再发 */
}

/* 写目标速度（0x2E，符号-幅值：BIT15=方向）。它是**保持型寄存器**，上电写一次就够 */
static uint8_t servo_set_speed (int16_t speed)
{
    uint8_t  f[9];
    uint16_t v;

    if (speed >  3400) { speed =  3400; }
    if (speed < -3400) { speed = -3400; }

    v = (speed < 0) ? (uint16_t)((uint16_t)(-speed) | 0x8000U) : (uint16_t)speed;

    f[0] = 0xFFU; f[1] = 0xFFU;
    f[2] = (uint8_t)CTRL_SERVO_ID;
    f[3] = 5U;
    f[4] = SERVO_INSTR_WRITE;
    f[5] = SERVO_REG_SPEED;
    f[6] = (uint8_t)(v & 0xFFU);
    f[7] = (uint8_t)(v >> 8);
    f[8] = servo_checksum(&f[2], 6U);
    return servo_queue(f, 9U);
}

uint16_t ctrl_servo_get_pos (void) { return g_servo_pos; }
uint8_t  ctrl_servo_get_err (void) { return g_servo_err; }
uint32_t ctrl_servo_tx_cnt  (void) { return g_servo_tx_cnt; }
uint32_t ctrl_servo_rx_cnt  (void) { return g_servo_rx_cnt; }

/* 请求读当前位置（5Hz 调一次就够，原工程约 300ms 一次） */
static void servo_read_pos (void)
{
    uint8_t f[8];

    f[0] = 0xFFU; f[1] = 0xFFU;
    f[2] = (uint8_t)CTRL_SERVO_ID;
    f[3] = 4U;
    f[4] = SERVO_INSTR_READ;
    f[5] = SERVO_REG_CURPOS;
    f[6] = 2U;
    f[7] = servo_checksum(&f[2], 5U);
    (void)servo_queue(f, 8U);
}

/* 收包解析：在 20ms 节拍里跑，**绝不 while 等** */
static void servo_poll (void)
{
    static uint8_t buf[40];
    static uint8_t n = 0U;
    uint8_t len, cs;

    while (g_srx_tail != g_srx_head)
    {
        if (n < sizeof(buf)) { buf[n++] = g_srx[g_srx_tail]; }
        g_srx_tail = (uint16_t)((g_srx_tail + 1U) % SERVO_RX_SIZE);

        /* 找帧头 FF FF */
        while ((n >= 2U) && !((0xFFU == buf[0]) && (0xFFU == buf[1])))
        {
            memmove(buf, buf + 1, (size_t)(n - 1U));
            n --;
        }
        if (n < 4U) { continue; }

        len = buf[3];
        if ((len < 2U) || (len > 32U)) { memmove(buf, buf + 1, (size_t)(n - 1U)); n --; continue; }
        if (n < (uint8_t)(len + 4U)) { continue; }   /* 还没收全 */

        cs = servo_checksum(&buf[2], (uint8_t)(len + 1U));   /* ID..最后一个数据字节 */
        if (cs != buf[len + 3U]) { memmove(buf, buf + 1, (size_t)(n - 1U)); n --; continue; }

        /* ★ 丢掉自己的回显：和刚发出去的帧一模一样就跳过 */
        if ((n >= g_last_tx_len) && (0 == memcmp(buf, g_last_tx, g_last_tx_len)))
        {
            memmove(buf, buf + (len + 4U), (size_t)(n - (len + 4U)));
            n = (uint8_t)(n - (len + 4U));
            continue;
        }

        g_servo_err = buf[4];
        /* 应答 = FF FF ID LEN ERROR P1..PN CS，**不带寄存器地址**（幻尔协议）
           我们只读 0x38 当前位置 2 字节 ⇒ LEN=4，P1/P2 = 位置低/高 */
        if ((buf[4] == 0U) && (len >= 4U))
        {
            g_servo_pos = (uint16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
            g_servo_rx_cnt ++;
        }
        else if (buf[4] == 0U)
        {
            g_servo_rx_cnt ++;
        }

        memmove(buf, buf + (len + 4U), (size_t)(n - (len + 4U)));
        n = (uint8_t)(n - (len + 4U));
    }
}

/* stm32f4xx_it.c 的 UART4_IRQHandler 转发到这里 */
void ctrl_uart4_irq (void)
{
    uint32_t sr = UART4->SR;

    if (0U != (sr & USART_SR_RXNE))
    {
        uint8_t  b    = (uint8_t)(UART4->DR & 0xFFU);
        uint16_t next = (uint16_t)((g_srx_head + 1U) % SERVO_RX_SIZE);

        if (next != g_srx_tail) { g_srx[g_srx_head] = b; g_srx_head = next; }
    }

    if (0U != (sr & USART_SR_TXE))
    {
        if (g_stx_tail != g_stx_head)
        {
            UART4->DR = g_stx[g_stx_tail];
            g_stx_tail = (uint16_t)((g_stx_tail + 1U) % SERVO_TX_SIZE);
        }
        else
        {
            UART4->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
        }
    }
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

static void speed_tick (void)
{
    int32_t meas_x10, err_x10, p_term, i_step, ff_term, out;

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

/* 重新生成 4 行文字（内容变了才标 dirty） */
static void oled_text_update (void)
{
    char b[20];

    if (0U != wifi_f4_is_ready()) { sprintf(b, "WiFi OK"); }
    else if (0U == wifi_f4_last_err()) { sprintf(b, "WiFi --"); }   /* 还在初始化（模块没报错）*/
    else                          { sprintf(b, "WiFi E%u", (unsigned)wifi_f4_last_err()); }
    oled_pad(b); oled_line_set(0U, b);

    if (0U != g_imu_ok) { sprintf(b, "MPU OK"); }
    else                { sprintf(b, "MPU %02X", (unsigned)g_imu_who); }
    oled_pad(b); oled_line_set(1U, b);

    sprintf(b, "ENC %05ld", (long)(g_enc_total % 100000L));
    oled_pad(b); oled_line_set(2U, b);

    sprintf(b, "RPM %4d", (int)(g_enc_rpm_x10 / 10));
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
void ctrl_all_init (void)
{
    ctrl_dwt_init();

    ctrl_io_init();          /* 蜂鸣器/按键最先，上电不叫 */
    ctrl_motor_init();       /* 电机 PWM=0、DIR=0，不会突然转 */
    ctrl_encoder_init();
    ctrl_servo_init();
    ctrl_imu_init();

    /* OLED：清屏 + **立刻画状态页**（约 40ms，只在开机做一次）
       ⚠️ 必须在这里画，不能等主循环：`ctrl_all_init()` 之后紧接着就是
          `wifi_f4_init()`，它最长会阻塞几十秒，那段时间主循环根本不在转。 */
    ctrl_oled_init();
    ctrl_oled_clear();
    ctrl_oled_first_page();

    ctrl_speed_init();
}

void board_ctrl_tick (void)
{
    static uint32_t t_last = 0U;
    static uint8_t  srv_div = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - t_last) < CTRL_PERIOD_MS) { return; }
    t_last = now;

    /* 1) 编码器：取增量 + 算转速 */
    encoder_tick();

    /* 2) 速度环（只在目标非 0 时才会有输出） */
    speed_tick();

    /* 3) IMU：每拍读一次（50Hz），读出陀螺就喂 yaw */
    imu_tick();
    if (0U != g_imu_ok)
    {
        yaw_feed(g_gyr[2]);
        yaw_tick();
    }

    /* 4) 舵机
          · 上电先等 1 秒（50 拍）—— 总线舵机自己也要初始化，太早说话它不理
          · 然后先写一次目标速度（0x2E 是保持型寄存器），再把舵机摆到中位
          · 之后：目标变了才发；空闲时每 10 拍（200ms）读一次位置
          ⚠️ 读写不要挤在同一拍：总线是半双工的，一问一答挨太近会打架 */
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
    }
    else
    {
        srv_div ++;
        if (srv_div >= 10U) { srv_div = 0U; servo_read_pos(); }
    }

    servo_poll();

    /* 5) OLED 状态页（没有 OLED 就整个跳过） */
    ctrl_oled_status();
}
