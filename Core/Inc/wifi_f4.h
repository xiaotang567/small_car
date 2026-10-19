/*********************************************************************************************************************
* 文件名称          wifi_f4
* 适用平台          STM32F407（HAL 库）
* 说明              逐飞 高速WIFI6B21-SPI V2.0 模块驱动 —— STM32F4 精简移植版
*
* 来历：逐飞的官方驱动 `zf_device_wifi_spi.c` 是给 RT1064/TC264/MSPM0 写的，
*       依赖它们自己的 zf_driver_spi / zf_driver_gpio / zf_driver_delay / zf_fifo。
*       这里**只保留"连上热点 → 连上电脑的 TCP 服务端 → 发数据"这条必需路径**，
*       底层换成 STM32 HAL。收数据(FIFO)那部分暂时没移植。
*
* ★ 模块协议要点（从官方驱动里读出来的，不是猜的）：
*     - SPI **Mode 0**（CPOL=0/CPHA=0），软件 CS（低有效）
*     - 每次 SPI 事务 = 4 字节帧头 + 数据
*         帧头 = [ cmd ][ reserve ][ length 高 8 位 ][ length 低 8 位 ]
*     - INT 脚：**低电平 = 模块忙**，每次事务前必须等它变高（有超时）
*     - 命令： 0x01 复位 / 0x02 透传数据 / 0x10 设置WIFI / 0x11 设置SOCKET
*              0x20 取版本 / 0x22 取IP；   应答： 0x80 = OK
*     - 复位命令的 length 特殊，固定写 0xA5A5
*     - 设置 WIFI   ：报文是 "SSID\r\n密码\r\n"
*     - 设置 SOCKET ：报文是 "TCP\r\n目标IP\r\n目标端口\r\n本机端口\r\n"（每段都以 \r\n 结尾）
*
* ★ 接线（按你的实际接线改下面这几个宏就行）
*
* 修改记录
* 日期              作者                备注
* 2026-10-13        织命                first version（F407/HAL 移植）
********************************************************************************************************************/

#ifndef _WIFI_F4_H_
#define _WIFI_F4_H_

#include "main.h"

//===================================================接线（改这里）===================================================
//   ⚠️ 下面是按"F407 主板方案"给的默认值。你要是接在别的脚上，只改这几个宏即可。
#define WIFI_SCK_PORT       GPIOB
#define WIFI_SCK_PIN        GPIO_PIN_13         // SPI2_SCK
#define WIFI_MISO_PORT      GPIOB
#define WIFI_MISO_PIN       GPIO_PIN_14         // SPI2_MISO
#define WIFI_MOSI_PORT      GPIOB
#define WIFI_MOSI_PIN       GPIO_PIN_15         // SPI2_MOSI
#define WIFI_CS_PORT        GPIOB
#define WIFI_CS_PIN         GPIO_PIN_12         // 软件片选（低有效）
#define WIFI_INT_PORT       GPIOD
#define WIFI_INT_PIN        GPIO_PIN_8          // 模块"忙"信号（低=忙）
#define WIFI_RST_PORT       GPIOD
#define WIFI_RST_PIN        GPIO_PIN_9          // 复位（低有效）

#define WIFI_SPI_INSTANCE   SPI2
// ⚠️ 分频要**跟着 APB1 的频率一起看**：SPI2 挂在 APB1 上。
//    APB1 = 42MHz（168MHz 主频时）⇒ ÷8 得 5.25MHz、÷4 得 10.5MHz、÷2 得 21MHz（上限）。
//    ★ 现在用 ÷8 = **5.25MHz**。
//      原因：之前主频还是 16MHz 时 APB1=16MHz，÷4 只有 **4MHz** 就能通；
//            主频修到 168MHz 之后同样的 ÷4 变成了 10.5MHz，**杜邦线/面包板上的波形会烂掉**，
//            现象就是"模块连上热点了但电脑一条数据都收不到"（SPI 命令根本没进去）。
//      ⇒ 先用 5.25MHz 跑通。想提速再改 ÷4（10.5MHz），一步步试。
#define WIFI_SPI_BAUD       SPI_BAUDRATEPRESCALER_8   // 42MHz/8 = 5.25MHz
//===================================================接线===================================================


//===================================================网络配置（改这里）===================================================
//   ⚠️ 和 MSPM0 那边同一套：**手机热点 + 电脑的 TCP 服务端**
#define WIFI_SSID           "iQOO Neo9S Pro+"
#define WIFI_PASSWORD       "12345678"

#define WIFI_PC_IP          "192.168.53.77"     // 电脑的 IPv4（热点网段会变！用 ipconfig 查）
                                                // 2026-10-19 更新：189.77 → 53.77（SSID 没变，只换了网段 ✓）
#define WIFI_PC_PORT        "8086"              // 上位机监听端口
#define WIFI_LOCAL_PORT     "6666"              // 模块自己的本地端口

#define WIFI_USE_TCP        1                   // 1 = TCP(推荐)  0 = UDP
//===================================================网络配置===================================================


//===================================================对外接口===================================================
// 0 = 名字 1 = 拿到IP地址 2 = 拿到MAC地址 3 = 完成一次TCP握手(就绪，可以发数据了)
extern volatile uint8_t wifi_f4_progress;
extern char             wifi_f4_ip_text[25];    // 模块的 IP:端口（用于打印/显示）
extern char             wifi_f4_ver_text[12];   // 模块固件版本

// 初始化 + 连热点 + 连电脑。⚠️ 阻塞，最长约 60 秒。返回 0 = 成功
uint8_t  wifi_f4_init(void);
// 重试"连电脑"这一步（热点已经连上时用，很便宜）。
// ★ 用途：如果上电时上位机还没开，wifi_f4_init() 会失败并且**永远不再重试**；
//   主循环里每隔几秒调一次这个，上位机后开也能自动接上。返回 0 = 接上了
uint8_t  wifi_f4_retry(void);
// 模块是否就绪（可以发数据了）
uint8_t  wifi_f4_is_ready(void);
// 上一次 init/retry 的结果：0=成功 1=模块没应答 2=连不上热点 3=连不上电脑
uint8_t  wifi_f4_last_err(void);
// 发送（自动按 4088 字节分片）。返回 0 = 全部发出去
uint32_t wifi_f4_send(const uint8_t *buf, uint32_t len);
// 发字符串
void     wifi_f4_send_string(const char *s);

// ★★ 接收：从模块里"取"回下行数据（电脑 TCP 发过来的东西）★★
//   返回实际读到的字节数（0 = 这次没有数据）。非阻塞，主循环里轮询调用即可。
//   🔑 官方驱动里这一步是 `wifi_spi_read_buffer()`：帧头 cmd 固定发 **0x02(透传)**、
//      length=0，模块回 **0x90(还有数据)/0x91(读完了)** + 真实长度 + 载荷。
//      这里只保留"取一包"这条最短路径，不做 FIFO（文本命令很短，够用）。
uint32_t wifi_f4_read(uint8_t *buf, uint32_t len);

// ★ 接收诊断量（用 `u` 命令读走显示）：查"PC 发的数据到底有没有到板子"用
extern volatile uint32_t wifi_rd_try;    // 发起读事务的次数
extern volatile uint32_t wifi_rd_busy;   // 因模块"忙"而没发起的次数
extern volatile uint32_t wifi_rd_ok;     // 拿到 0x90/0x91 包头的次数
extern volatile uint8_t  wifi_rd_cmd;    // 最近一次包头命令字
extern volatile uint16_t wifi_rd_len;    // 最近一次包头长度
extern volatile uint8_t  wifi_rd_int;    // 最近一次 INT 电平（1=空闲 0=忙）
//===================================================对外接口===================================================

#endif
