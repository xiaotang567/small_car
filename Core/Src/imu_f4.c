/*********************************************************************************************************************
* 文件名              imu_f4.c
* 适用平台            STM32F407
* 说明                MPU9250（软 I2C：PB8=SCL / PB9=SDA）+ yaw 积分
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"


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

/* ★ 2026-11-07：把"读 IMU + 喂 yaw + 积分"合成一个节拍，供 board_ctrl_tick() 调用 */
void ctrl_imu_tick (void)
{
    imu_tick();
    if (0U != g_imu_ok)
    {
        yaw_feed(g_gyr[2]);
        yaw_tick();
    }
}

uint8_t ctrl_imu_ok (void) { return g_imu_ok; }
