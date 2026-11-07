/*********************************************************************************************************************
* 文件名              oled_f4.c
* 适用平台            STM32F407
* 说明                OLED（SSD1306 128x64，软 I2C：PB10=SCL / PB11=SDA）+ 状态页
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"
#include "wifi_f4.h"
#include <stdio.h>
#include <string.h>


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
    static const char spin[4] = { '-', '\\', '|', '/' };
    char     b[20];
    char     w[4];
    char     m[4];
    uint32_t now = HAL_GetTick();

    /* ---- 第 0 行：WiFi / MPU 一键看（"W:OK M:OK" 正好 10 个字符）---- */
    if (0U != wifi_f4_is_ready())      { strcpy(w, "OK"); }
    else if (0U == wifi_f4_last_err()) { strcpy(w, "--"); }      /* 还在初始化 */
    else                               { sprintf(w, "E%u", (unsigned)wifi_f4_last_err()); }

    if (0U != ctrl_imu_ok()) { strcpy(m, "OK"); }
    else                { sprintf(m, "%02X", (unsigned)ctrl_imu_whoami()); }

    sprintf(b, "W:%s M:%s", w, m);
    oled_pad(b); oled_line_set(0U, b);

    /* ---- 第 1 行：★ 电机状态（2026-11-07 FW=2112 新增）----
         RUN 500   = 闭环巡航在 500RPM
         STALL PC2 = 堵转停机，**按 PC2 重新起转**
         STOP      = 没在跑（目标是 0）*/
    {
        int16_t tgt = ctrl_speed_get_target();
        int32_t at  = (tgt < 0) ? -(int32_t)tgt : (int32_t)tgt;

        if (0U != ctrl_motor_stalled())                { sprintf(b, "STALL PC2"); }
        else if ((0U != ctrl_motor_started()) && (at > 0)) { sprintf(b, "RUN %4ld", (long)at); }
        else                                      { sprintf(b, "STOP"); }
    }
    oled_pad(b); oled_line_set(1U, b);

    sprintf(b, "ENC %05ld", (long)(ctrl_encoder_total() % 100000L));
    oled_pad(b); oled_line_set(2U, b);

    /* ★★ 2026-11-07（FW=2100）：行尾这个会**转的小符号** = "主循环还活着"的铁证 ★★
       以前 RPM 一直是 0、ENC 不动的时候，屏幕看着就像"卡死"，根本分不清是
       "主循环停了"还是"只是数值没变所以没重画"。现在它每 250ms 变一次
       （一行重画 ≈4ms ⇒ 约 1.6% CPU），**停了就是真停了**。*/
    sprintf(b, "RPM %4d %c", (int)(ctrl_encoder_rpm_x10() / 10), spin[(now / 250U) & 3U]);
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
