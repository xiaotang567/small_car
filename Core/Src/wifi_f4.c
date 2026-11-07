/*#endif
********************************************************************************************************************
* 文件名称          wifi_f4
* 适用平台          STM32F407（HAL 库）
* 说明              逐飞 高速WIFI6B21-SPI V2.0 模块驱动 —— STM32F4 精简移植版。见 wifi_f4.h 的说明。
*
* 修改记录
* 日期              作者                备注
* 2026-10-13        织命                first version（F407/HAL 移植）
********************************************************************************************************************/

#include "wifi_f4.h"
#include <string.h>
#include <stdio.h>

//===================================================协议常量===================================================
#define WF_CMD_RESET            0x01        // 复位（length 固定写 0xA5A5）
#define WF_CMD_DATA             0x02        // 透传数据
#define WF_CMD_SET_WIFI         0x10        // 设置 WIFI 信息
#define WF_CMD_SET_SOCKET       0x11        // 设置 SOCKET 信息
#define WF_CMD_GET_VERSION      0x20        // 取固件版本
#define WF_CMD_GET_MAC          0x21        // 取 MAC 地址
#define WF_CMD_GET_IP           0x22        // 取模块 IP/端口

#define WF_REPLY_OK             0x80        // 从机应答"正确"

#define WF_RECV_SIZE            32          // 每次传输的固定载荷长度（官方驱动写死 32）
#define WF_TX_CHUNK             4088        // 一次 SPI 事务最大字节数（官方驱动写死 4088）

// 超时：单位是"100 微秒"（和官方驱动的 wait_idle 口径一致）
#define WF_TO_OTHER             1000        // 1 秒    普通操作
#define WF_TO_WIFI              10000       // 10 秒   连热点
#define WF_TO_SOCKET            50000       // 50 秒   连 TCP 服务器
//===================================================协议常量===================================================


//===================================================内部状态===================================================
typedef struct { uint8_t command; uint8_t reserve; uint16_t length; } wf_head_t;
typedef struct { wf_head_t head; uint8_t buffer[WF_RECV_SIZE]; }      wf_pkt_t;

static SPI_HandleTypeDef    hwifi;
static volatile uint8_t     wf_ready = 0;

volatile uint8_t wifi_f4_progress = 0;      // 0=没开始 1=拿到版本 2=拿到IP 3=就绪
char             wifi_f4_ip_text[25]  = "-";
char             wifi_f4_ver_text[12] = "-";
static char      wf_mac_text[20]      = "-";
//===================================================内部状态===================================================


//===================================================底层：延时 / GPIO / SPI===================================================

// ---- DWT 微秒延时（HAL 只有毫秒，协议里要 10us 级延时）----
static void wf_dwt_init (void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void wf_delay_us (uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

#define WF_CS_LOW()       HAL_GPIO_WritePin(WIFI_CS_PORT,  WIFI_CS_PIN,  GPIO_PIN_RESET)
#define WF_CS_HIGH()      HAL_GPIO_WritePin(WIFI_CS_PORT,  WIFI_CS_PIN,  GPIO_PIN_SET)

// ---- SPI 收发（全用 HAL 阻塞模式，超时给足）----
static void wf_spi_tx (const uint8_t *d, uint16_t n)
{
    if (n) HAL_SPI_Transmit(&hwifi, (uint8_t *)d, n, 1000);
}

static void wf_spi_tr (const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (0 == n) return;
    if      (NULL == tx) HAL_SPI_Receive       (&hwifi, rx,           n, 1000);
    else if (NULL == rx) HAL_SPI_Transmit      (&hwifi, (uint8_t *)tx, n, 1000);
    else                 HAL_SPI_TransmitReceive(&hwifi, (uint8_t *)tx, rx, n, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     硬件初始化（SPI2 + 4 个 GPIO）
// 备注信息     ⚠️ 这里自己配，不依赖 CubeMX —— 所以 CubeMX 里不用管 SPI2
//-------------------------------------------------------------------------------------------------------------------
static void wf_hw_init (void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    // SCK / MISO / MOSI -> AF5 (SPI2)
    gi.Pin       = WIFI_SCK_PIN | WIFI_MISO_PIN | WIFI_MOSI_PIN;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gi);

    // CS / RST -> 推挽输出，上电先给高
    HAL_GPIO_WritePin(WIFI_CS_PORT,  WIFI_CS_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_SET);

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin   = WIFI_CS_PIN;
    HAL_GPIO_Init(WIFI_CS_PORT, &gi);
    gi.Pin   = WIFI_RST_PIN;
    HAL_GPIO_Init(WIFI_RST_PORT, &gi);

    // INT -> 输入（内部上拉，空闲时是高）
    gi.Pin  = WIFI_INT_PIN;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(WIFI_INT_PORT, &gi);

    hwifi.Instance               = WIFI_SPI_INSTANCE;
    hwifi.Init.Mode              = SPI_MODE_MASTER;
    hwifi.Init.Direction         = SPI_DIRECTION_2LINES;
    hwifi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hwifi.Init.CLKPolarity       = SPI_POLARITY_LOW;        // ↓ Mode 0
    hwifi.Init.CLKPhase          = SPI_PHASE_1EDGE;         // ↑
    hwifi.Init.NSS               = SPI_NSS_SOFT;
    hwifi.Init.BaudRatePrescaler = WIFI_SPI_BAUD;
    hwifi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hwifi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hwifi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hwifi);
}
//===================================================底层===================================================


//===================================================协议层（对照官方驱动逐条移植）===================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等模块空闲（INT 变高）
// 返回参数     uint8   0 = 空闲（可以发）  1 = 超时
// 备注信息     ⚠️ INT 是"低 = 忙"，所以这里等它变高
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_wait_idle (uint32_t wait_time)
{
    uint32_t time = 0;

    wait_time = wait_time * 100;                        // 和官方一致：单位 100us
    while (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) == GPIO_PIN_RESET)
    {
        wf_delay_us(10);
        time++;
        if (wait_time <= time) break;
    }
    return (wait_time <= time);
}

// 只写不读：CS 低 -> 写 b1 -> 写 b2 -> CS 高
static void wf_write (const uint8_t *b1, uint16_t l1, const uint8_t *b2, uint16_t l2)
{
    WF_CS_LOW();
    wf_spi_tx(b1, l1);
    wf_spi_tx(b2, l2);
    WF_CS_HIGH();
}

// 帧头全双工收发（4 字节）；datalen > 0 时接着把 buffer 也全双工收发
static void wf_xfer_command (wf_pkt_t *p, uint16_t datalen)
{
    uint8_t tx[4], rx[4];

    tx[0] = p->head.command;
    tx[1] = p->head.reserve;
    tx[2] = (uint8_t)(p->head.length >> 8);
    tx[3] = (uint8_t)(p->head.length & 0xFF);

    WF_CS_LOW();
    wf_spi_tr(tx, rx, 4);

    if (datalen)
    {
        uint8_t tmp[WF_RECV_SIZE];
        memset(tmp, 0, sizeof(tmp));
        wf_spi_tr(p->buffer, tmp, datalen);
        memcpy(p->buffer, tmp, datalen);
    }
    WF_CS_HIGH();

    p->head.command = rx[0];
    p->head.reserve = rx[1];
    p->head.length  = ((uint16_t)rx[2] << 8) | rx[3];
}

// 数据收发：发 write_data，收进 read_data->buffer
static void wf_xfer_data (const uint8_t *write_data, wf_pkt_t *read_data, uint16_t length)
{
    uint8_t tx[4], rx[4];

    read_data->head.command = WF_CMD_DATA;
    read_data->head.reserve = 0;
    read_data->head.length  = length;

    tx[0] = read_data->head.command;
    tx[1] = read_data->head.reserve;
    tx[2] = (uint8_t)(length >> 8);
    tx[3] = (uint8_t)(length & 0xFF);

    WF_CS_LOW();
    wf_spi_tr(tx, rx, 4);
    read_data->head.command = rx[0];
    read_data->head.reserve = rx[1];
    read_data->head.length  = ((uint16_t)rx[2] << 8) | rx[3];

    if (WF_RECV_SIZE < length)
    {
        // 先全双工收发 32 字节，剩下的用"只发"补完（和官方驱动一致）
        wf_spi_tr(write_data, read_data->buffer, WF_RECV_SIZE);
        wf_spi_tx(&write_data[WF_RECV_SIZE], (uint16_t)(length - WF_RECV_SIZE));
    }
    else
    {
        // 载荷不足 32 字节时，官方是"把这 32 字节同时收发"——这里用临时缓冲规避别名
        uint8_t tmp[WF_RECV_SIZE];
        memcpy(read_data->buffer, write_data, length);
        memset(tmp, 0, sizeof(tmp));
        wf_spi_tr(read_data->buffer, tmp, WF_RECV_SIZE);
        memcpy(read_data->buffer, tmp, WF_RECV_SIZE);
    }
    WF_CS_HIGH();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数设置（命令 + 数据 -> 等应答 0x80）
// 返回参数     uint8   0 = 成功  1 = 失败
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_set_parameter (uint8_t command, const uint8_t *buffer, uint16_t length, uint32_t wait_time)
{
    uint8_t  state = 1;
    wf_head_t head;

    do
    {
        head.command = command;
        head.reserve = 0;
        head.length  = length;

        if (wf_wait_idle(wait_time)) break;                     // 等模块空闲
        wf_write(&head.command, sizeof(head), buffer, length);  // 发命令 + 数据

        if (wf_wait_idle(wait_time)) break;                     // 等它处理完

        {
            wf_pkt_t p;
            memset(&p, 0, sizeof(p));
            p.head.command = WF_CMD_DATA;
            p.head.length  = 0;
            wf_xfer_command(&p, 0);                             // 取应答
            wf_delay_us(20);
            if (WF_REPLY_OK == p.head.command) state = 0;
        }
    } while (0);

    return state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数获取
// 返回参数     uint8   0 = 成功  1 = 失败
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_get_parameter (uint8_t command, wf_pkt_t *read_data, uint32_t wait_time)
{
    if (wf_wait_idle(wait_time)) return 1;

    read_data->head.command = command;
    read_data->head.reserve = 0;
    read_data->head.length  = 0;

    wf_write(&read_data->head.command, WF_RECV_SIZE, NULL, 0);  // 官方：一次写 32 字节

    if (wf_wait_idle(wait_time)) return 1;

    read_data->head.command = WF_CMD_DATA;
    read_data->head.length  = 0;
    wf_xfer_command(read_data, WF_RECV_SIZE);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 接收（下行）
//-------------------------------------------------------------------------------------------------------------------
//   对照官方 `wifi_spi_read_buffer()` 精简移植：**只保留"取一包"这条最短路径**。
//   🔑 官方那版还带一个 1KB FIFO（外部缓冲不足时把余量存起来）。这里不搬 ——
//      我们的下行只有"短文本命令"（几十字节），一次 32 字节的包就够；
//      而 FIFO 会引入"数据要等下一次调用才出现"的时序不确定性，反而更难排查。
//   ⚠️ 载荷上限就是 32（WF_RECV_SIZE）；只要回的是 0x90（"还有数据"）就继续取下一包，
//      最多 8 轮，防止模块一直说"还有"导致死循环。
//   🔥🔥 2026-10-15 修：**读也要重试**！
//      原来"一次读到 INT 是低（忙）就立刻返回 0" —— 而模块在连续发图时**经常是忙的**
//      ⇒ 实测 `RX=0`：开机到现在**一个字节都没读回来**（命令全部石沉大海）。
//      心跳能发出去是因为文本发送会重试 40 次，而读只试一次 ⇒ 单方面饿死 ✓
//      ⇒ 现在：内部重试 8 次（每次最多等 200µs）⇒ 单次调用最坏 ~1.6ms，仍远小于主循环预算 ✓
#define WF_REPLY_DATA_START     0x90        // 从机回传的数据包，且还有数据待读取
#define WF_REPLY_DATA_END       0x91        // 从机回传的数据包，数据已读完

// ★ 诊断量（`u` 命令会读走显示）：把"模块到底回了什么"记下来
volatile uint32_t wifi_rd_try  = 0;     // 真正发起读事务的次数
volatile uint32_t wifi_rd_busy = 0;     // 因为模块"忙"而连事务都没发起的次数
volatile uint32_t wifi_rd_ok   = 0;     // 拿到 0x90/0x91 包头的次数（>0 说明协议对得上）
volatile uint8_t  wifi_rd_cmd  = 0;     // 最近一次包头里的命令字
volatile uint16_t wifi_rd_len  = 0;     // 最近一次包头里的长度
volatile uint8_t  wifi_rd_int  = 0;     // 最近一次看到的 INT 电平（1 = 空闲，0 = 忙）

/* ★★★ 2026-11-06（FW=2097）：**取一包（只 clock 模块真正返回的字节数）** ★★★
   现场故障（抓包抓出来的）：上位机 20 秒收到 504KB，其中 **95% 是 0x00**，
   25.2 KB/s 全是零；真正的遥测行夹在零里 ⇒ 上位机按行解析时"整行含 0x00 就丢弃"
   ⇒ 遥测永远 `---`。

   零从哪来：原来这里用 `wf_xfer_command(&p, 32)` —— 不管模块有没有数据，
   **都硬 clock 32 个字节出去**。模块把这次 SPI 写进来的字节当"透传数据"，
   于是每个主循环（~800 次/秒）就有 32 个 0x00 被塞进 TCP 流（800×32 ≈ 25 KB/s ✓）。
   而且这些零会把模块 4KB 缓冲挤满，把真正的文本挤出/截断（S 行只剩后半截 ✓）。

   ✅ 现在：先只发 4 字节包头，**看模块回的 length**：
      · 没有数据（length=0 / 不是 0x90/0x91）⇒ **一个字节都不 clock** ⇒ 零泄漏；
      · 有数据 ⇒ 按原来的定长 32 字节收发（数据通路行为不变，风险最小）。*/
static void wf_read_one (wf_pkt_t *p)
{
    uint8_t  tx[4], rx[4];
    uint8_t  zeros[WF_RECV_SIZE];
    uint16_t n;

    tx[0] = WF_CMD_DATA;                    /* 0x02：请求"把电脑发过来的数据给我"*/
    tx[1] = 0U;
    tx[2] = 0U;
    tx[3] = 0U;

    WF_CS_LOW();
    wf_spi_tr(tx, rx, 4U);                  /* 包头（全双工：模块的应答就藏在这 4 字节里）*/

    p->head.command = rx[0];
    p->head.reserve = rx[1];
    p->head.length  = (uint16_t)(((uint16_t)rx[2] << 8) | (uint16_t)rx[3]);

    n = p->head.length;
    if (n > (uint16_t)WF_RECV_SIZE) { n = (uint16_t)WF_RECV_SIZE; }

    /* ★ 关键：**模块没有数据时，一个字节都不要 clock**（零泄漏的根源就在这）。
       有数据时仍然按原来的定长 32 字节收发 —— 数据通路的行为完全不变，只去掉空闲时的泄漏。*/
    if ((0U != n) &&
        ((WF_REPLY_DATA_START == p->head.command) || (WF_REPLY_DATA_END == p->head.command)))
    {
        memset(zeros, 0, sizeof(zeros));
        memset(p->buffer, 0, sizeof(p->buffer));
        wf_spi_tr(zeros, p->buffer, WF_RECV_SIZE);
    }
    WF_CS_HIGH();
}

uint32_t wifi_f4_read (uint8_t *buf, uint32_t len)
{
    wf_pkt_t p;
    uint32_t got   = 0;
    uint8_t  round = 0;
    uint8_t  retry;

    if ((0U == wf_ready) || (NULL == buf) || (0U == len)) { return 0U; }

    // ★ 先重试等一个"空闲窗口"。
    //   🔥 2026-11-06：原来是 8 次 × 200µs = **1.6ms**，而 `cam_cmd_poll()` 在主循环里
    //      **每轮都调**这个函数 ⇒ 模块一忙，主循环绝大部分时间都耗在这段等待里
    //      （图像收割被拖慢），而且等满了照样读不到 ⇒ 上位机按键永远收不到回执
    //      （"按钮要按很多次才有反应"）。
    //   现在只等 300µs：等不到就放过这一轮，主循环转得很快，下一轮立刻再来。
    for (retry = 0U; retry < 3U; retry++)
    {
        if (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) != GPIO_PIN_RESET) { break; }
        wf_delay_us(100);
    }

    wifi_rd_int = (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) != GPIO_PIN_RESET) ? 1U : 0U;

    // ★★ 2026-11-06：**不再因为"忙"就整轮跳过读取** ★★
    //   现场故障：上行图像正常，但上位机所有命令都收不到（`r -1` 重发 20 次无 ACK）。
    //   原因就是这里：模块被 20fps 图像压得 INT 长期为低 ⇒ 这个函数永远在这一行 return 0
    //   ⇒ 下行数据（PC 发来的命令）**从来没被取走**。
    //   现在：忙的时候只让 100µs，然后**照样去读**；模块要是真没数据，
    //   读回来包头不是 0x90/0x91，got 自然是 0，没有任何副作用。
    if (0U == wifi_rd_int) { wifi_rd_busy ++; wf_delay_us(100); }

    do
    {
        memset(&p, 0, sizeof(p));

        p.head.command = WF_CMD_DATA;           // 0x02：请求"把电脑发过来的数据给我"
        p.head.length  = 0;                     // 官方这里固定写 0
        wifi_rd_try ++;
        wf_read_one(&p);                        /* ★ 只 clock 模块真正返回的字节数（见上）*/
        wf_delay_us(20);

        wifi_rd_cmd = p.head.command;           // ★ 记下模块到底回了什么
        wifi_rd_len = p.head.length;

        if (((WF_REPLY_DATA_START == p.head.command) || (WF_REPLY_DATA_END == p.head.command))
            && (p.head.length > 0U))
        {
            uint32_t n = p.head.length;

            wifi_rd_ok ++;
            if (n > (uint32_t)WF_RECV_SIZE) { n = (uint32_t)WF_RECV_SIZE; }
            if (n > (len - got))            { n = len - got; }
            memcpy(&buf[got], p.buffer, n);
            got += n;
        }
        round ++;
    } while ((WF_REPLY_DATA_START == p.head.command) && (got < len) && (round < 8U));

    return got;
}
//===================================================接收（下行）===================================================

// 把应答缓冲里的可打印 ASCII 抠成 C 字符串（模块回的是文本）
static void wf_copy_text (char *dst, uint8_t dst_size, const uint8_t *src, uint8_t src_len)
{
    uint8_t i;

    if (dst_size < 2) return;
    for (i = 0; (i < (uint8_t)(dst_size - 1)) && (i < src_len); i ++)
    {
        char c = (char)src[i];
        if ((c < 0x20) || (0x7E < c)) break;
        dst[i] = c;
    }
    dst[i] = '\0';
    if (0 == i) { strcpy(dst, "-"); }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软复位：帧头 length 固定 0xA5A5（官方驱动如此）
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_reset (void)
{
    wf_head_t head;

    head.command = WF_CMD_RESET;
    head.reserve = 0;
    head.length  = 0xA5A5;

    if (wf_wait_idle(WF_TO_OTHER)) return 1;
    wf_write(&head.command, sizeof(head), NULL, 0);
    return 0;
}

static uint8_t wf_get_version (void)
{
    wf_pkt_t p;

    memset(&p, 0, sizeof(p));
    if (wf_get_parameter(WF_CMD_GET_VERSION, &p, WF_TO_OTHER)) return 1;
    wf_copy_text(wifi_f4_ver_text, sizeof(wifi_f4_ver_text), p.buffer, WF_RECV_SIZE);
    return 0;
}

static uint8_t wf_get_mac (void)
{
    wf_pkt_t p;

    memset(&p, 0, sizeof(p));
    if (wf_get_parameter(WF_CMD_GET_MAC, &p, WF_TO_OTHER)) return 1;
    wf_copy_text(wf_mac_text, sizeof(wf_mac_text), p.buffer, WF_RECV_SIZE);
    return 0;
}

/* ★★★ 自动推导电脑 IP：模块 IP 的前三段 + 末段 77 ★★★
   手机热点每次开都会换网段（已踩 5 次），但电脑末段恒为 .77
   ⇒ 用模块自己的 IP 推导，以后换网段不用改代码重烧 ✓
   兜底：解析失败就用宏 WIFI_PC_IP ✓ */
static char wf_pc_ip[16] = WIFI_PC_IP;

#if !(WF_PC_USE_FIXED)   /* 固定 IP 模式下这个函数不用，包起来免得报"未引用" */
static void wf_derive_pc_ip (void)
{
    unsigned a = 0U, b = 0U, c = 0U;

    if (3 == sscanf(wifi_f4_ip_text, "%u.%u.%u.", &a, &b, &c))
    {
        (void)sprintf(wf_pc_ip, "%u.%u.%u.%u", a, b, c, (unsigned)WF_PC_LAST_OCTET);
    }
}
#endif

static uint8_t wf_get_ip (void)
{
    wf_pkt_t p;

    memset(&p, 0, sizeof(p));
    if (wf_get_parameter(WF_CMD_GET_IP, &p, WF_TO_OTHER)) return 1;
    wf_copy_text(wifi_f4_ip_text, sizeof(wifi_f4_ip_text), p.buffer, WF_RECV_SIZE);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     连热点：报文 "SSID\r\n密码\r\n"
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_wifi_connect (const char *ssid, const char *password)
{
    uint8_t buf[64];
    uint16_t len;

    if (NULL != password)
    {
        len = (uint16_t)sprintf((char *)buf, "%s\r\n%s\r\n", ssid, password);
    }
    else
    {
        len = (uint16_t)sprintf((char *)buf, "%s\r\n", ssid);
    }

    if (wf_set_parameter(WF_CMD_SET_WIFI, buf, len, WF_TO_WIFI)) return 1;
    wf_get_ip();
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     连 TCP 服务器：报文 "TCP\r\n目标IP\r\n目标端口\r\n本机端口\r\n"
//   wait_ms：等模块应答的上限。开机那次给 50s（模块要联网）；**重连必须给短**，
//            否则模块还连着的时候 INT 一直是低 ⇒ 主循环被按住 50 秒 ⇒ 上位机"已连接但 0 B/s"。
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_socket_connect (const char *type, const char *ip, const char *port,
                                  const char *local_port, uint32_t wait_ms)
{
    uint8_t buf[48];
    uint16_t len;

    len = (uint16_t)sprintf((char *)buf, "%s\r\n%s\r\n%s\r\n%s\r\n", type, ip, port, local_port);

    if (wf_set_parameter(WF_CMD_SET_SOCKET, buf, len, wait_ms)) return 1;
    wf_get_ip();
    return 0;
}
//===================================================协议层===================================================


//===================================================对外接口===================================================

static volatile uint8_t wf_last_err = 0;            // 上一次 init/retry 的结果

uint8_t wifi_f4_init (void)
{
    wf_ready           = 0;
    wifi_f4_progress   = 0;
    wf_last_err        = 0;

    wf_dwt_init();
    wf_hw_init();

    // ---- 硬复位：RST 拉低 10ms 再拉高，等模块起 ----
    HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    // ---- 再走一次软复位，把模块内部状态机清到已知状态（length 固定 0xA5A5） ----
    wf_reset();
    HAL_Delay(50);

    // ---- 取版本 / MAC：既是握手，也是"模块到底通没通"的判据 ----
    if (wf_get_version()) { wf_last_err = 1; return 1; }
    wifi_f4_progress = 1;
    wf_get_mac();

    // ---- 连热点 ----
    if (wf_wifi_connect(WIFI_SSID, WIFI_PASSWORD)) { wf_last_err = 2; return 2; }
    if (wf_get_ip())                               { wf_last_err = 2; return 2; }
#if !(WF_PC_USE_FIXED)
    wf_derive_pc_ip();                             /* 自动推导电脑 IP（固定模式下不执行）*/
#endif
    wifi_f4_progress = 2;

    // ---- 连电脑的 TCP 服务端 ----
    // 🔥🔥 实测踩到的坑：会出现 **"模块自己已经把 TCP 连上电脑了、但我们的握手没等到 ACK"**
    //    的情况 —— 上位机那边看到 ESTABLISHED，板子这边却以为失败。
    //    而 `wifi_f4_send()` 的开头是 `if (!wf_ready || NULL == buf) return len;`
    //    ⇒ **从此一个字节都不发**。现象就是：**上位机显示"已连接"，但一条数据都收不到。**
    //    ⇒ 所以这里**即使握手失败，也把 wf_ready 置 1 让数据能发出去**
    //      （真没连上时模块会把数据丢掉，**无害**）。状态码照旧返回，LED/日志不受影响。
    if (wf_socket_connect(WIFI_USE_TCP ? "TCP" : "UDP",
                          wf_pc_ip, WIFI_PC_PORT, WIFI_LOCAL_PORT, WF_TO_SOCKET))
    {
        wf_last_err      = 3;
        wifi_f4_progress = 3;
        wf_ready         = 1;           // ★ 见上面说明：照样让它发
        return 3;
    }
    wifi_f4_progress = 3;
    wf_last_err      = 0;
    wf_ready         = 1;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重试"连电脑"这一步
// 返回参数     uint8   0 = 接上了
// 备注信息     ★ 为什么要它：wifi_f4_init() 只在开机时连一次 socket，
//               如果那一刻上位机还没开，就会**永远连不上、一条数据都不发**。
//               主循环里隔几秒调一次，上位机后开也能自动接上，不用重新上电。
//-------------------------------------------------------------------------------------------------------------------
uint8_t wifi_f4_retry (void)
{
    if (wf_ready) return 0;

    /* ★★★ 2026-11-07（FW=2098）★★★
       现场故障：上位机一直显示"已连接，但 0 B/s"，只能手动断电才好。
       原因就在这里：`wf_socket_connect()` 用的是 **50 秒** 的等待。
       模块一旦"揣着一个半死的 socket、缓冲一直满"（INT 长期低），这个等待就是**满的**
       ⇒ 主循环被按住 50 秒 ⇒ 上位机那边就是"已连接 + 0 B/s"，看起来像死机。
       ⇒ 现在重连只等 **1.5 秒**：等不到就立刻回来，让主循环继续转，
         连续失败会更快走到"整模块硬复位"（app_loop 里 relink_n ≥ 2 就做），
         那才是救"模块揣着半死 socket"的正解。*/
    if (wf_socket_connect(WIFI_USE_TCP ? "TCP" : "UDP",
                          wf_pc_ip, WIFI_PC_PORT, WIFI_LOCAL_PORT, 1500U) == 0)
    {
        wifi_f4_progress = 3;
        wf_last_err      = 0;
        wf_ready         = 1;
        return 0;
    }

    wf_last_err = 3;
    return 1;
}

uint8_t wifi_f4_is_ready (void)
{
    return wf_ready;
}

/* ★★★ 2026-11-07（FW=2099）：**外部判定"链路已死"时，强制走重连** ★★★
   为什么需要它：模块被复位 / 掉线 / 揣着半死 socket 时，**SPI 写照样成功**
   （模块照收，只是发不出去）⇒ `wifi_f4_send()` 不报错 ⇒ `wf_ready` 一直是 1
   ⇒ 板子永远以为"还连着"，一直往黑洞里发 ⇒ 上位机永远"已连接 + 0 B/s"，只能断电。
   ⇒ 由 app.c 用"上位机保活"当见证人：收不到保活就调这个函数，把链路判死重连。*/
void wifi_f4_force_relink (void)
{
    wf_ready = 0U;
}

uint8_t wifi_f4_last_err (void)
{
    return wf_last_err;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ★ 微秒级"等模块空闲"（**专供发送路径**）★
//   🔥🔥 2026-10-15 修一个**大坑**：原来发送路径用的是 `wf_wait_idle(WF_TO_OTHER)`，
//      而那个函数的单位是 1ms ⇒ `WF_TO_OTHER = 1000` ⇒ **最长死等 1 秒**！
//      图像每 6ms 发一行，模块只要偶尔忙一下，主循环就被**按住最多 1 秒** ⇒
//        ① 上位机的命令要等好几秒才被处理（"按 j 半天没反应"就是这个）
//        ② 逐行收割直接崩（`LG=117` 那次就是这个）
//      ⇒ 改成**最多等 300µs**就返回，让调用者下一轮再试 —— **绝不在这里死等**。
//   返回：0 = 空闲（可以发）  1 = 超时（还忙）
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_wait_idle_us (uint32_t us)
{
    uint32_t i;

    for (i = 0U; i < us; i += 10U)
    {
        if (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) != GPIO_PIN_RESET) { return 0U; }
        wf_delay_us(10);
    }
    return 1U;
}

/* ★★★ 2026-11-06（FW=2096）：**文本发送队列**（遥测 / 命令回执）★★★
   现场故障：图像 10fps 正常刷，但上位机的遥测数值**一直是 `---`**、按键也没回执。
   真凶**不是"发不出去"，是"半行文本"**（①），再加上排队规则把遥测顶掉了（②）：

   ① 缓冲区原来只有 192 字节，而 `S` 心跳一行就有 **203 字节**（字段多）
      ⇒ `strncpy` 把它截成 191 字节 ⇒ **行尾的 `\r\n` 被截掉了**。
      上位机是按 `\n` 切行的 ⇒ 这半行会跟**下一条文本粘成一行**：
          `S f=...GP=0T n=1234 r=0 ...\r\n`
      粘出来的行既不是 `T` 也不是 `ACK`/`CFG`/`CAM` ⇒ 上位机**整行丢掉**。
      要命的是：心跳（500ms）和遥测（200ms）每次都在**同一轮主循环**里到期，
      代码顺序是"先遥测入队、后心跳入队"⇒ 心跳顶掉遥测、并且紧跟着被发出去
      ⇒ 从此**每一条遥测都粘在那半行心跳后面被丢**，上位机就永远显示 `---`。
   ② 队列原来"后到的覆盖先到的"（同级也覆盖）⇒ 心跳会顶掉刚排队的遥测。
      现在：**同是低优先级时保留已经排队的那条**（先排先发；丢掉的是心跳，无所谓）。
   ③ 模块被图像压满时文本可能永远排不出去，而"链路卡死"的判断只在
      wifi_f4_send() 里，pump 提前 return 时**永远走不到** ⇒ 只能手动复位。
      现在这里加 3 秒看门狗：排不出去就判链路死，交给主循环重连 / 整模块复位。

   pri：2 = 命令回执/CFG（最重要，可顶掉任何一条）
        1 = 遥测 `T`（新鲜度最重要：新的一定要顶掉旧的）
        0 = 心跳/诊断 `S`（可有可无，绝不能顶掉遥测）*/
static char    wf_txt_buf[320];       /* ★ 必须 > 最长的一行（S 心跳 203，最坏 271）*/
static uint8_t wf_txt_has = 0U;
static uint8_t wf_txt_pri = 0U;
static uint32_t wf_txt_t  = 0U;       /* 入队时刻：判超时 / 判内容是不是已经过期 */
static uint32_t wf_txt_ok  = 0U;      /* 文本发出去几条 */
static uint32_t wf_txt_fail = 0U;     /* 文本没抢到窗口的次数 */

uint32_t wifi_f4_txt_ok   (void) { return wf_txt_ok; }
uint32_t wifi_f4_txt_fail (void) { return wf_txt_fail; }
uint8_t  wifi_f4_text_pending (void) { return wf_txt_has; }

/* 拷进队列：**无论多长，永远以 \r\n 结尾**。
   （截断时必须补上，否则上位机会把这一行和下一行粘在一起 —— 就是上面 ① 那个坑）*/
static void wf_txt_store (const char *s)
{
    uint32_t n = (uint32_t)strlen(s);

    if (n > (uint32_t)(sizeof(wf_txt_buf) - 2U)) { n = (uint32_t)(sizeof(wf_txt_buf) - 2U); }
    memcpy(wf_txt_buf, s, n);

    while ((n > 0U) && ((wf_txt_buf[n - 1U] == '\r') || (wf_txt_buf[n - 1U] == '\n'))) { n --; }
    wf_txt_buf[n] = '\r'; n ++;
    wf_txt_buf[n] = '\n'; n ++;
    wf_txt_buf[n] = '\0';
}

void wifi_f4_text (const char *s, uint8_t pri)
{
    if (NULL == s) { return; }

    if (0U != wf_txt_has)
    {
        if (wf_txt_pri > pri) { return; }       /* 已有更重要的 ⇒ 丢本条（例如：别用心跳顶掉遥测）*/

        /* ★ 同级 0（心跳/诊断）：排着的就让它发完，新的直接丢（省得两条 S 互相打架）。
           排了 1.5 秒还发不出去 ⇒ 内容已经过期，允许新的顶掉。*/
        if ((wf_txt_pri == pri) && (0U == pri) &&
            ((HAL_GetTick() - wf_txt_t) < 1500U)) { return; }
    }

    wf_txt_store(s);
    wf_txt_pri = pri;
    wf_txt_has = 1U;
    wf_txt_t   = HAL_GetTick();
}

void wifi_f4_text_pump (void)
{
    uint32_t left;
    uint8_t  i;

    if (0U == wf_txt_has) { return; }

    if (0U == wf_ready)
    {
        /* 链路断了：这条已经过期，直接丢掉（重连后自然会有新的遥测/心跳）*/
        wf_txt_has = 0U;
        wf_txt_pri = 0U;
        return;
    }

    /* ★ 排了 3 秒还出不去 ⇒ 模块/链路已经死了（INT 一直低）。
       不能一直挂着：原来"链路卡死"的判断只在 wifi_f4_send() 里，而模块忙的时候
       pump 在下面几行就 return 了，**永远走不到** ⇒ 只能手动复位。
       现在：丢掉这条，并把 wf_ready 清 0，交给主循环 wifi_f4_retry()/wifi_f4_init()。*/
    if ((HAL_GetTick() - wf_txt_t) >= 3000U)
    {
        wf_txt_has = 0U;
        wf_txt_pri = 0U;
        wf_txt_fail ++;
        wf_ready   = 0U;
        return;
    }

    /* ★★ 文本抢窗口要比图像**更有耐心** ★★
       图像每轮主循环都重试（机会无数），文本只有 5Hz 的节奏；
       之前文本和图像用同样的 300µs 预算，模块一忙文本永远抢不到
       ⇒ 遥测/ACK 一条都发不出去。
       文本很稀疏（5Hz、150 字节），这里最多等 2ms 完全值得（25ms/s ≈ 2.5%）。*/
    for (i = 0U; i < 20U; i++)
    {
        if (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) != GPIO_PIN_RESET) { break; }
        wf_delay_us(100);
    }
    if (i >= 20U) { wf_txt_fail ++; return; }      /* 这还是忙 ⇒ 下一轮再来（不丢）*/

    left = wifi_f4_send((const uint8_t *)wf_txt_buf, (uint32_t)strlen(wf_txt_buf));
    if (0U == left)
    {
        wf_txt_has = 0U;
        wf_txt_pri = 0U;
        wf_txt_ok ++;
    }
    else
    {
        wf_txt_fail ++;
    }
}

uint32_t wifi_f4_send (const uint8_t *buf, uint32_t len)
{
    uint16_t chunk;
    uint32_t s0;
    wf_pkt_t p;

    if (!wf_ready || (NULL == buf)) return len;

    s0 = len;                          /* 进来时的长度：判断"这一轮有没有发出去东西" ✓ */

    while (len)
    {
        chunk = (len > WF_TX_CHUNK) ? (uint16_t)WF_TX_CHUNK : (uint16_t)len;

        /* ★★★ 2026-11-02 晚：**改回 300µs**（取消上一版的 `chunk*8`）★★★
           上一版把等待放大到 33ms，是为了救"助手模式 6 个块"那种情况；
           但**现在的采集是逐行收割**（`cam_f4.c` 已回退到 `3d389dd`），
           它要求主循环至少每 80ms 跑一圈（环形缓冲只装 4 帧）。
           一次 23ms 的阻塞就会让收割一口气补 138 行、跨过 VSYNC 对齐点
           ⇒ **直接表现为缺行/横线**。
           而 py 协议是"整帧一包 2894B = **1 个块**"，发之前只需要等**一次**空闲，
           300µs 足够（模块真空闲时 INT 一变高就立刻返回，根本等不满）。
           ⇒ 回到 300µs：主循环不被按住，收割才跟得上。 */
        if (wf_wait_idle_us(300U)) break;

        memset(&p, 0, sizeof(p));
        wf_xfer_data(buf, &p, chunk);

        len  -= chunk;
        buf  += chunk;
    }

    /* ★★ 自动重连（2026-10-19 加，2026-11-06 改成**按时间**判）
       原来：连续 150 次"发了没进展"就判链路断。
       🔥 实测这个判据太敏感：每次发送最多等 300µs，150 次 ≈ **45ms**。
          模块只要有一小会儿忙（图像 + 遥测一起压过去就是），45ms 没进展就被判成断链
          ⇒ `wf_ready=0` ⇒ 3 秒后重连 ⇒ 新的 TCP 连接（本机端口 6666→6667→6668…）
          ⇒ 上位机看到的就是"已连接 / 对方断开"**反复刷屏**（用户 2026-11-06 截图）。
       ✅ 现在：**连续 2 秒一个字节都没发出去**才判断链 —— 短暂拥塞不会再拆连接，
          而"上位机重启导致会话死掉"这种情况 2 秒内也一定能发现 ✓ */
    {
        static uint32_t wf_stall_t = 0U;
        uint32_t now = HAL_GetTick();

        if (0U != (s0 - len))              /* 这次有进展 */
        {
            wf_stall_t = now;
        }
        else if (0U == wf_stall_t)          /* 第一次调用 */
        {
            wf_stall_t = now;
        }
        else if ((now - wf_stall_t) >= 2000U)
        {
            wf_stall_t = now;
            wf_ready   = 0U;               /* 2 秒完全发不出去 ⇒ 判链路断，交给主循环重连 */
        }
    }
    return len;
}

// 发文本（命令回执/心跳）：**内部重试**。
//   🔑 文本丢一行，用户就以为"板子没反应"（实测就是这样：`j`/`o` 的回执全丢，
//      只有偶尔一条 `CFG ...` 挤出去），所以这里**必须**给足重试机会；
//      而图像行丢一行无所谓（下一轮会补）⇒ 图像路径不重试 ✓
//   ⚠️ 权衡：每次重试 = 最多等 300µs + 停 400µs；100 次 ⇒ **最坏 ~70ms**。
//      心跳也走这个函数 ⇒ 极端情况下主循环会被按 70ms（会让 `LG` 尖一下、画面掉几行），
//      但**"回执必须发出去"比"图像那几行"重要得多** ⇒ 这个代价值得 ✓
void wifi_f4_send_string (const char *s)
{
    uint8_t i;

    if (NULL == s) { return; }

    /* ★★★ 2026-11-02：100 → **10** ★★★
       切回 cam_view.py 后文本心跳会重新生效（VOFA 模式是禁掉的）。
       100 次重试 × (最多等 300µs + 停 400µs) ⇒ 主循环最多被按 **70ms**，
       而一帧图像（位打包 2894B）只要 ~4.4ms ⇒ 一次心跳就能吞掉十几帧的采集窗口
       ⇒ `LG` 尖峰、画面掉行。10 次 ⇒ 最坏约 7ms，回执仍然发得出去（文档 2.4 的结论）*/
    for (i = 0U; i < 10U; i++)
    {
        if (0U == wifi_f4_send((const uint8_t *)s, (uint32_t)strlen(s))) { return; }
        wf_delay_us(400);
    }
}

// ★ 遥测专用：**最多试 3 次**（最坏约 1ms），发不出去就丢掉。
//   为什么不复用 `wifi_f4_send_string()`：那个会重试 10 次（最坏 7ms），10Hz 遥测用它
//   等于每 100ms 按主循环 7ms，实测把摄像头收割都拖慢了；
//   而只试 1 次又会丢太多（"数据一会有一会没有"）⇒ 折中 **3 次 ≈ 1ms**。
void wifi_f4_send_string_nb (const char *s)
{
    uint8_t i;

    if (NULL == s) { return; }

    for (i = 0U; i < 3U; i++)
    {
        if (0U == wifi_f4_send((const uint8_t *)s, (uint32_t)strlen(s))) { return; }
        wf_delay_us(200);
    }
}
//===================================================对外接口===================================================
