/*********************************************************************************************************************
* 文件名称          ctrl_f4.h
* 适用平台          STM32F407
* 说明              车模控制层（从 MSPM0G3507 工程移植）：电机 / 编码器 / 幻尔总线舵机 / MPU9250 陀螺仪 / 速度 PID
*
* ★ 引脚（和嘉立创打的那块板一致，别改）：
*     电机1  PWM PA2(TIM2_CH3)  DIR PB0        电机2  PWM PA3(TIM2_CH4)  DIR PB1
*     编码器 A  PA0(TIM5_CH1)   B  PA1(TIM5_CH2)  Z  PC0(EXTI0)
*     舵机   TX PC10(UART4_TX)  RX PC11(UART4_RX)   —— 1Mbps，幻尔总线协议
*     MPU9250 SCL PB8  SDA PB9      （软件 I2C，本工程的硬件 I2C 模块没开）
*     蜂鸣器 PC1     按键 PC2 / PC3
*
* ★ 为什么全用**寄存器级**驱动：
*   本工程的 `stm32f4xx_hal_conf.h` 只开了 DCMI/SPI/DMA 三个 HAL 模块，
*   TIM/UART/I2C 的 HAL 模块是**关掉的**。⇒ 电机/编码器/舵机/IMU 全部直接写寄存器，
*   和现有的 UART5/TIM6 写法保持一致，也不占 flash。
*
* ★ 和摄像头的共存约定（非常重要）：
*   1) 这里所有初始化都不碰 DCMI / DMA2 / PB7 / PA4 / PA6 / PC6~PC9 / PE4~PE6；
*   2) 控制任务必须**非阻塞**（舵机发送走 TXE、IMU 读一次约 0.4ms），
*      绝不能出现"死等模块"的循环 —— 主循环被按住就会漏摄像头行；
*   3) 控制节拍统一由 `board_ctrl_tick()` 从主循环里调，内部按 20ms 自己节流。
*
* ★ 从 MSPM0 工程过来的东西（对应的原文件）：
*     motor.c/motor.h        -> ctrl_motor_*（占空比 ±10000、DIR 电平同原工程、17kHz）
*     encoder.c/encoder.h    -> ctrl_encoder_*（原工程用"脉冲+方向"，新板是 A/B 正交 ⇒ 4 倍频）
*     speed_ctrl.c/.h        -> ctrl_speed_*（PI + 比例前馈 + 斜坡 + 极性自动识别，参数照抄）
*     hx_servo.c/h           -> ctrl_servo_*（1Mbps 幻尔协议；**改成非阻塞**，不再死等回包）
*     mpu9250.c/h + yaw.c/h  -> ctrl_imu_* / ctrl_yaw_*（软 I2C + 4 点滑动平均 + 20ms 积分）
*
* 修改记录
* 日期              作者                备注
* 2026-11-05        织命                从 MSPM0G3507 工程移植
********************************************************************************************************************/

#ifndef _CTRL_F4_H_
#define _CTRL_F4_H_

#include "main.h"

//===================================================参数（按实车标定）===================================================

/* ---- 电机（原工程：17kHz、占空比 ±10000、DIR 高=正转）---- */
#define CTRL_PWM_FREQ_HZ        17000U
#define CTRL_PWM_ARR            ((uint16_t)(84000000UL / CTRL_PWM_FREQ_HZ - 1UL))   /* = 4940 → 17.0kHz */
#define CTRL_PWM_FULL           10000        /* 对外约定：±10000 = 满占空（和原工程一致） */
#define CTRL_M1_DIR_INV         0            /* 1 = 电机1 方向反过来（接线/装反时改这里） */
#define CTRL_M2_DIR_INV         0

/* ---- 编码器 ----
   原工程用的是"脉冲+方向"型编码器（每圈 1024 计数）。新板子是 A/B/Z 三线正交接 TIM5，
   硬件 4 倍频 ⇒ 每圈计数 = **线数 × 4**。
   ⚠️ 这个数**必须实测**：手动把轮子转整一圈，看 `ctrl_encoder_total()` 涨了多少就填多少。*/
#define CTRL_ENC_CPR            4096         /* 1024 线 × 4 倍频（先按这个，实测后改） */
#define CTRL_ENC_DIR_INV        1            /* ★ 2026-11-06：实测"轮子往前转、速度显示负数" ⇒ 置 1 取反
                                                （A/B 相接反了，改线也行，但软件一行更省事） */

/* ---- 控制周期 ---- */
#define CTRL_PERIOD_MS          20           /* 速度环 / 姿态积分周期（原工程 ENC_SAMPLE_PERIOD_MS = 20） */

/* ---- 速度环（数值全部照抄原工程 speed_ctrl.h，只把电机的编号放松成可配）---- */
#define CTRL_SPEED_MOTOR        1            /* 速度环驱动哪个电机：1 / 2（原工程是 MOTOR_1）*/
#define CTRL_SPEED_FF_PER_1000  5000         /* 前馈：目标每 1000RPM 给多少占空比 */
#define CTRL_SPEED_RAMP_RPM_S   3000         /* 设定值斜坡：每秒允许升多少 RPM */
#define CTRL_SPEED_KP_X1000     1500         /* Kp = 1.500 */
#define CTRL_SPEED_KI_X1000     150          /* Ki = 0.150（每个控制周期） */
#define CTRL_SPEED_I_LIMIT      2500         /* 积分限幅（占空比单位） */
#define CTRL_SPEED_DUTY_MAX     5000         /* 输出上限 50%（防烧电机，别乱改大） */
#define CTRL_SPEED_SIGN_AUTO    1            /* 1 = 反馈极性自动识别 */

/* ---- 舵机（幻尔 HX 总线舵机）----
   ★ 2026-11-06：用户这块舵机是 **1 Mbps**（幻尔 HX 出厂默认）。
     开机还会自动在 11 档波特率里找一遍（10000 也在列表里，用户一度以为是 10000），
     所以这里的默认值只是"第一个试的"。
   ⚠️ 顺带修的一个坑：低速时 USARTDIV 会超过 4095（BRR 装不下）
      ⇒ `servo_set_baud()` 会自动切 OVER8=1（8 倍过采样），10000bps 以下才需要。*/
#define CTRL_SERVO_BAUD         1000000UL
/* ★ 2026-11-06：**开机默认走单线半双工**（用户要求）
   1 = 默认单线：数据只走 **PC10** 一根线（UART4 HDSEL + PC10 开漏+上拉）
   0 = 默认全双工：PC10=TX / PC11=RX
   运行时仍可用 `r -1`（单线）/ `r -2`（全双工）切换，不用重烧。*/
#define CTRL_SERVO_HALF_DUPLEX_DEFAULT  0
#define CTRL_SERVO_ID           1            /* 总线上那颗舵机的 ID */
#define CTRL_SERVO_POS_MIN      1048         /* 安全限位（原工程 2048 ± 1000） */
#define CTRL_SERVO_POS_MAX      3048
#define CTRL_SERVO_POS_CENTER   2048         /* 4096 步 = 360° ⇒ 0.0879°/步 */
#define CTRL_SERVO_SPEED        3400         /* 目标速度（步/秒，原工程 SERVO_RUN_SPEED） */

/* ---- MPU9250 ---- */
#define CTRL_IMU_ADDR           0x68         /* AD0 接地 = 0x68（原工程硬编码） */
#define CTRL_IMU_PERIOD_MS      20           /* 多久读一次（原工程约 100ms 读一次，这里 20ms 更好） */
#define CTRL_IMU_I2C_DELAY_US   2            /* 软 I2C 半周期（µs）：2 ⇒ 约 250kHz。
                                                ⚠️ 模块板上没有上拉电阻、或杜邦线很长时，
                                                   读不到 WHO_AM_I（who=00）就把这个改大：4 ⇒ 125kHz，5 ⇒ 100kHz */

/* ---- yaw（照抄原工程 yaw.h 的算法与参数）---- */
#define CTRL_YAW_TICK_MS        20
#define CTRL_YAW_CALIB_N        50           /* 前 N 拍做零偏标定（50 × 20ms = **1 秒**；原工程 2 秒）*/
#define CTRL_YAW_FILTER_N       4            /* 4 点滑动平均 */
#define CTRL_YAW_DEADBAND_LSB   80           /* 死区 ≈ 0.61°/s */
#define CTRL_YAW_LSB_PER_DPS    131.0f       /* ±250dps 量程 ⇒ 131 LSB/(°/s) */
#define CTRL_YAW_SIGN           (+1)
#define CTRL_YAW_SCALE          1.0f

//===================================================对外接口===================================================

/* ---------- 初始化（全部很快，不阻塞；在 board_cam_init() 之后调）---------- */
void     ctrl_all_init (void);

/* ---------- 电机 ---------- */
void     ctrl_motor_init (void);
void     ctrl_motor_set  (uint8_t id, int16_t duty);   /* id=1/2，duty = -10000..+10000，正=前进 */
void     ctrl_motor_stop (void);
int16_t  ctrl_motor_get  (uint8_t id);

/* ---------- 编码器（TIM5，32 位）---------- */
void     ctrl_encoder_init (void);
int32_t  ctrl_encoder_total (void);        /* 上电以来的累计计数（不清零） */
int32_t  ctrl_encoder_delta (void);        /* 上一个 20ms 窗口的计数增量 */
void     ctrl_encoder_clear (void);        /* 累计值清零（Z 相中断也会把 CNT 清 0，但不影响累计值） */
int32_t  ctrl_encoder_rpm_x10 (void);      /* 转速，单位 0.1RPM（M 法：ΔN×600000/(CPR×周期ms)） */
uint8_t  ctrl_encoder_z_cnt (void);

/* ---------- 速度环（PI + 前馈 + 斜坡）---------- */
void     ctrl_speed_init (void);
void     ctrl_speed_set_target (int16_t rpm);      /* 目标转速；0 = 停机 */
int16_t  ctrl_speed_get_target (void);
int16_t  ctrl_speed_get_duty   (void);
int16_t  ctrl_speed_get_error  (void);
int16_t  ctrl_speed_get_rpm    (void);             /* 实测转速（RPM，整数） */
uint8_t  ctrl_speed_get_state  (void);             /* 0=停 1=爬坡 2=稳速 3=故障 */
const char * ctrl_speed_state_name (void);

/* ---------- 舵机（非阻塞：只把帧塞进发送缓冲，TXE 中断自己发）---------- */
void     ctrl_servo_init (void);
void     ctrl_servo_scan (void);           /* 扫 0~20 号，谁应答就用谁（init 里自动调）*/
/* 运行时切换：half=1 单线半双工（只接 PC10）；half=0 全双工（PC10=TX / PC11=RX）*/
void     ctrl_servo_set_half (uint8_t half);
uint8_t  ctrl_servo_is_half (void);
/* pos = 0..4095（4096 步 = 360°）。返回 0 = 已排进发送队列 */
uint8_t  ctrl_servo_set_pos (uint16_t pos);
uint16_t ctrl_servo_get_pos (void);        /* 最近一次**读到**的位置（没读到过 = 0xFFFF） */
uint8_t  ctrl_servo_get_err (void);        /* 最近一次应答里的 ERROR 字节（0 = 正常） */
uint8_t  ctrl_servo_id     (void);         /* 开机扫描出来的舵机 ID */
uint32_t ctrl_servo_rx_bytes (void);       /* 一共收到多少字节（0 = 舵机从没回过话）*/
uint32_t ctrl_servo_tx_bytes (void);       /* 一共**真正发出去**多少字节（0 = TX 没工作）*/
uint32_t ctrl_servo_rx_bad (void);         /* 校验失败的帧数（>0 = 波特率/格式不对）*/
uint32_t ctrl_servo_baud   (void);         /* 当前用的波特率 */
uint8_t  ctrl_servo_rx_pin (void);         /* PC11(RX) 现在的电平：1 = 空闲高（正常），0 = 被拉低 */
uint32_t ctrl_servo_tx_cnt  (void);        /* 发出去几帧（诊断） */
uint32_t ctrl_servo_rx_cnt  (void);        /* 收到几帧合法应答（诊断） */

/* ---------- MPU9250 + yaw ---------- */
void     ctrl_imu_init (void);
uint8_t  ctrl_imu_whoami (void);           /* 0x68/0x70/0x71/0x73/0x75/0x98 = 正常 */
void     ctrl_imu_accel (int16_t *ax, int16_t *ay, int16_t *az);
void     ctrl_imu_gyro  (int16_t *gx, int16_t *gy, int16_t *gz);
float    ctrl_yaw_deg   (void);            /* 偏航角（度，相对开机，无限累加） */
float    ctrl_yaw_rate  (void);            /* 去零偏后的角速度（°/s） */
uint8_t  ctrl_yaw_ready (void);            /* 1 = 零偏标定完成 */
void     ctrl_yaw_reset (void);

/* ---------- OLED（SSD1306 128×64，软 I2C：PB10=SCL PB11=SDA）---------- */
/* 软 I2C 半周期（µs）。1 ⇒ 约 400~500kHz（SSD1306 上限 400kHz，实测多数模块能吃）；
   OLED 只用来刷状态页，不涉及时序敏感的外设，快一点肉眼差别很大。
   ⚠️ 屏幕出现花屏/丢像素就把它改回 2 */
#define CTRL_OLED_I2C_DELAY_US  1
/* 状态页文字刷新周期（ms）：100 ⇒ **10Hz**（原来 250ms = 4Hz，看着慢）
   一行 2 页 ≈ 4ms（400kHz）⇒ 10Hz 下最多占 8% CPU，且每次只堵主循环 4ms */
#define CTRL_OLED_TEXT_MS       100
void     ctrl_oled_init (void);
void     ctrl_oled_clear (void);
/* 在 (x, page) 用 5×7 字体写字；scale=1 → 6×8 / scale=2 → 12×16（占两页） */
void     ctrl_oled_text (uint8_t x, uint8_t page, const char *s, uint8_t scale);
void     ctrl_oled_hello (void);           /* 开机画面：两行 2 倍字的 hello / world */
uint8_t  ctrl_oled_ok (void);              /* 1 = 总线上真的有 OLED（初始化时探测到应答）*/
/* 状态页（4 行 2 倍字）：WiFi 状态 / MPU 状态 / 编码器累计 / 转速
   ★ 4Hz 更新文字、每 20ms 最多重画一行 ⇒ 不会长时间按住主循环 */
void     ctrl_oled_status (void);
/* ★ 开机立刻整页刷一次（在 ctrl_all_init() 里调）：
   主循环要等 wifi_f4_init() 返回（最长几十秒）才开始转，
   不先画一次的话屏幕会一直停在开机画面上。约 34ms。 */
void     ctrl_oled_first_page (void);
/* ★★★ 2026-11-07（FW=2101）：**启动阶段显示** ★★★
   开机到主循环之间有 `ctrl_servo_scan()`（1~8 秒）和 `wifi_f4_init()`（最长 60 秒），
   这段时间屏幕**本来是静止的** ⇒ 卡在这两段里和"真死机"看起来一模一样。
   现在每一步都在第 0 行写 `BOOT n`，卡住时屏幕上就停在那一步的编号上。 */
void     ctrl_oled_boot (uint8_t stage);

/* ---------- 按键 / 蜂鸣器 ---------- */
void     ctrl_io_init (void);
uint8_t  ctrl_key (uint8_t idx);           /* idx = 1/2，返回 1 = 按下（低电平有效） */
void     ctrl_buzzer (uint8_t on);

/* ---------- 控制节拍（主循环每轮调一次，内部 20ms 节流）---------- */
void     board_ctrl_tick (void);

/* ---------- 中断入口（stm32f4xx_it.c 转发）---------- */
void     ctrl_uart4_irq (void);
void     ctrl_exti0_irq (void);

#endif /* _CTRL_F4_H_ */
