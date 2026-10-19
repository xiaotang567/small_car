/*********************************************************************************************************************
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

uint32_t wifi_f4_read (uint8_t *buf, uint32_t len)
{
    wf_pkt_t p;
    uint32_t got   = 0;
    uint8_t  round = 0;
    uint8_t  retry;

    if ((0U == wf_ready) || (NULL == buf) || (0U == len)) { return 0U; }

    // ★ 先重试等一个"空闲窗口"：模块忙就等一会儿再问（最多 8 次 × 200µs ≈ 1.6ms）
    for (retry = 0U; retry < 8U; retry++)
    {
        if (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) != GPIO_PIN_RESET) { break; }
        wf_delay_us(200);
    }

    wifi_rd_int = (HAL_GPIO_ReadPin(WIFI_INT_PORT, WIFI_INT_PIN) != GPIO_PIN_RESET) ? 1U : 0U;

    if (0U == wifi_rd_int) { wifi_rd_busy ++; return 0U; }        // 一直忙 ⇒ 这一次先不读

    do
    {
        memset(&p, 0, sizeof(p));

        p.head.command = WF_CMD_DATA;           // 0x02：请求"把电脑发过来的数据给我"
        p.head.length  = 0;                     // 官方这里固定写 0
        wifi_rd_try ++;
        wf_xfer_command(&p, WF_RECV_SIZE);      // 全双工 32 字节
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
//-------------------------------------------------------------------------------------------------------------------
static uint8_t wf_socket_connect (const char *type, const char *ip, const char *port, const char *local_port)
{
    uint8_t buf[48];
    uint16_t len;

    len = (uint16_t)sprintf((char *)buf, "%s\r\n%s\r\n%s\r\n%s\r\n", type, ip, port, local_port);

    if (wf_set_parameter(WF_CMD_SET_SOCKET, buf, len, WF_TO_SOCKET)) return 1;
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
    wifi_f4_progress = 2;

    // ---- 连电脑的 TCP 服务端 ----
    // 🔥🔥 实测踩到的坑：会出现 **"模块自己已经把 TCP 连上电脑了、但我们的握手没等到 ACK"**
    //    的情况 —— 上位机那边看到 ESTABLISHED，板子这边却以为失败。
    //    而 `wifi_f4_send()` 的开头是 `if (!wf_ready || NULL == buf) return len;`
    //    ⇒ **从此一个字节都不发**。现象就是：**上位机显示"已连接"，但一条数据都收不到。**
    //    ⇒ 所以这里**即使握手失败，也把 wf_ready 置 1 让数据能发出去**
    //      （真没连上时模块会把数据丢掉，**无害**）。状态码照旧返回，LED/日志不受影响。
    if (wf_socket_connect(WIFI_USE_TCP ? "TCP" : "UDP",
                          WIFI_PC_IP, WIFI_PC_PORT, WIFI_LOCAL_PORT))
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

    if (wf_socket_connect(WIFI_USE_TCP ? "TCP" : "UDP",
                          WIFI_PC_IP, WIFI_PC_PORT, WIFI_LOCAL_PORT) == 0)
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

uint32_t wifi_f4_send (const uint8_t *buf, uint32_t len)
{
    uint16_t chunk;
    wf_pkt_t p;

    if (!wf_ready || (NULL == buf)) return len;

    while (len)
    {
        chunk = (len > WF_TX_CHUNK) ? (uint16_t)WF_TX_CHUNK : (uint16_t)len;

        // ⚠️ 只等 300µs（不是 1 秒！）。还没发完就把"剩余字节数"返回给调用者，
        //    由调用者下一轮主循环再试 —— 这样主循环**永远不会被发送卡住** ✓
        if (wf_wait_idle_us(300U)) break;

        memset(&p, 0, sizeof(p));
        wf_xfer_data(buf, &p, chunk);

        len  -= chunk;
        buf  += chunk;
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

    for (i = 0U; i < 100U; i++)
    {
        if (0U == wifi_f4_send((const uint8_t *)s, (uint32_t)strlen(s))) { return; }
        wf_delay_us(400);
    }
}
//===================================================对外接口===================================================
