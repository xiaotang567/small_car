/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>            /* sprintf */
#include "wifi_f4.h"          /* 逐飞 WiFi6B21-SPI 模块驱动（F407/HAL 移植版） */
#include "board_f4.h"         /* 板级：时钟树 168MHz + 总钻风摄像头(DCMI) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCMI_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=========================================== ★★ 实时命令通道（上位机 → 板子）★★ ===========================================
//   为什么要它：图像链路已经通了，但"发多快 / 发多大 / 摄像头怎么配"全靠**重编固件**，
//   试一个参数就要烧一次，效率极低。这里给上位机开一条**纯文本命令**通道：
//     PC 通过同一条 TCP 连接发一行（以 \n 结尾） ⇒ 主循环解析 ⇒ **立刻生效** ⇒ 回一行 ACK。
//
//   命令表（大小写都行）：
//     f <n>        图像刷新率上限 fps（0=不限，上限 30）。**这就是"帧率"旋钮**：
//                  上一张图开始后不足 1000/n ms 就不再发下一张。
//     g <ms>       每行最小发送间隔（1..200，默认 6）。越小整幅图刷新越快、带宽越高。
//     x <n>        横向抽点（1..8，默认 1）：1=发 188 列，2=94 列，4=47 列 ⇒ **每行字节数变小**
//     y <n>        纵向抽点（1..8，默认 1）：1=发 120 行，2=60 行，4=30 行 ⇒ **行数变小、视野不变**
//     a            恢复默认（f=0 g=6 x=1 y=1）
//     s            回一条状态（f/g/x/y + CST/RV/线电平）
//     b <n>        配置串口波特率档位 0..4 = 9600/115200/57600/38400/19200
//     v            给摄像头发 GET_VERSION(0xF2)，回"收到几个字节 + 原始值"
//     c <cmd> <val>  给摄像头发一个 4 字节包（0xA5|cmd|H|L），例：c 3 10 = 把帧率设成 10
//     i            给摄像头发 INIT(0x00)（参数要它重新初始化才生效）
//     p            回两根线的静态电平（bit1=PC12/TX，bit0=PD2/RX）
//     ?            回帮助
//   ⚠️ 回的短文本行会被上位机打印出来（它把"魔数之前的连续 ASCII"当文本）✓
//=========================================== 实时命令通道 ===========================================

uint16_t cam_cmd_fps    = 0U;      // ★★ 默认 **不限**（提速：原来 1帧/秒 → 0=不限）✓
uint16_t cam_cmd_gap_ms = 0U;      // ★★ 默认 **0 = 不等**（开机即最快）；若坏行增多用 `g 1`/`g 2` 回退 ✓
uint8_t  cam_cmd_dx     = 1U;      // 横向抽点（1 = 188 列）
uint8_t  cam_cmd_dy     = 1U;      // 纵向抽点（1 = 120 行）

// ★ 心跳档位（上位机命令 `d <n>` 切换）—— 控制台噪声的**源头开关**
//   0 = 静默（正在调参数、看图像时最干净）
//   1 = **默认**：每 500ms 只回**一行**精简状态
//   2 = 全量诊断（CAM1/CAM2/CAM3/BPF/MD/PIN/P，共 8 行）
//   🔑 默认给 1 而不是 2：那 8 行诊断是"排 DCMI 收不到数据"时用的，
//      现在图像已经通了，它每 500ms 刷 8 行只会把上位机的打印和你的命令回执全冲掉。
uint8_t  cam_dbg_level  = 1U;      // ★ 默认发**一行**状态（上位机最多 5 秒打一次，不吵）
                                   //   `d 0` 彻底静默，`d 2` 全量诊断

// 本此开机的"复位原因"（见 main 里读 RCC->CSR 那一段；bit25..31）
uint32_t cam_rst_flags = 0U;

// ★ 从 WiFi 模块里读回来的字节数（下行总量）—— 用来判"命令到底有没有到板子"
volatile uint32_t cam_rx_bytes = 0U;

// ★ 采集链自愈/诊断用：最近一次量到的像素时钟(kHz) 与 DCMI->RISR 快照（心跳里报告）
static uint32_t t_pclk   = 0U;
static uint32_t dcmi_ris = 0U;
static uint32_t t_vsync_hz = 0U;
static uint32_t t_last_frame = 0U;    // ★ 上次自检时的帧计数（判"有没有新帧"）      // ★ 最近一次量到的 PB7(VSYNC) 边沿率(Hz)

// ★★ 自动探测摄像头接口的结果（**不需要用户敲任何命令**，直接看心跳）
//   0 = 还没找到   1 = SCCB 接法0 通   2 = SCCB 接法1 通   3 = 地址 0xBA 通
//   🔑 为什么要自动做：**调试命令也要走同一条链路**（链路坏的时候，命令根本进不来
//      ⇒ 只能靠用户反复敲命令来验证链路，既慢又费人）⇒ 链路一通，就该让板子自己干、
//      自己把结果写进心跳，用户只要看一行字 ✓
uint8_t  cam_if_ok    = 0U;
uint16_t cam_if_id    = 0U;
uint8_t  cam_if_tries = 0U;

// ★ 最近收到的原始字节（环形，`o` 命令 dump 出来看）—— 用来判"收到的是不是我们发的那几个字符"
volatile uint8_t  cam_rx_dump[32]  = {0};
volatile uint32_t cam_rx_dump_i    = 0U;

static char     cam_cmd_buf[80];
static uint8_t  cam_cmd_len = 0U;

// 抠出字符串开头的十进制整数（不引 stdlib）
static uint32_t cam_cmd_num (const char *s)
{
    uint32_t v = 0U;

    while ((*s == ' ') || (*s == '\t') || (*s == ':')) { s ++; }
    while ((*s >= '0') && (*s <= '9')) { v = v * 10U + (uint32_t)(*s - '0'); s ++; }
    return v;
}

// 跳到第 2 个数字的起点（"c 3 10" 里那个 10）
static const char * cam_cmd_arg2 (const char *s)
{
    while (*s && (*s != ' ')) { s ++; }
    while ((*s == ' ') || (*s == '\t')) { s ++; }
    return s;
}

static uint8_t cam_cmd_clamp (uint32_t v, uint8_t lo, uint8_t hi)
{
    if (v < (uint32_t)lo) { return lo; }
    if (v > (uint32_t)hi) { return hi; }
    return (uint8_t)v;
}

static void cam_cmd_status (void)
{
    char b[140];

    sprintf(b, "CFG f=%u g=%u x=%u y=%u (%ux%u) d=%u CST=%u RV=%06lX LINE=%u\r\n",
            (unsigned)cam_cmd_fps, (unsigned)cam_cmd_gap_ms,
            (unsigned)cam_cmd_dx, (unsigned)cam_cmd_dy,
            (unsigned)(CAM_TXW / cam_cmd_dx), (unsigned)(CAM_H / cam_cmd_dy),
            (unsigned)cam_dbg_level,
            (unsigned)cam_cfg_state, (unsigned long)(cam_cfg_ver & 0xFFFFFFUL),
            (unsigned)cam_cfg_line_state());
    wifi_f4_send_string(b);
}

// ★ ★ 位打包：1 = 二值图每行 188 像素压成 **24 字节**（1字节=8像素）⇒ 数据量÷8 ✓
//   上位机看包头 pkt[6] 的最高位（1 = 打包）自动解包；按 `z` 可切回未打包（排障用）✓
uint8_t cam_pack = 1U;
// ★ 整帧包发送进度：>0 表示正在发帧，此时**不能插文本**（会把字节插进二进制包里 ⇒ 竖线）
static uint32_t fpkt_total = 0U;
static uint32_t fpkt_sent  = 0U;

static void cam_cmd_line (char *s)
{
    char     b[96];
    uint8_t  c;
    uint32_t v, v2;

    while (*s == ' ') { s ++; }
    if (0 == *s) { return; }

    c = (uint8_t)(((*s >= 'A') && (*s <= 'Z')) ? (*s + 32) : *s);      // 转小写

    switch (c)
    {
        case 'f':
            cam_cmd_fps = (uint16_t)cam_cmd_num(s + 1);
            if (cam_cmd_fps > 30U) { cam_cmd_fps = 30U; }
            break;

        case 'g': cam_cmd_gap_ms = (uint16_t)cam_cmd_clamp(cam_cmd_num(s + 1), 0U, 200U); break;
        // ⚠️ 188 模式下 `dx`/`dy` **必须**是 1：
        //    那时 DMA 缓冲天然就是一整帧（120 行 × 188 字节），抽点会直接算错取址 ✗
        //    ⇒ 在 188 模式下**忽略**这两个命令，并明确回执（免得"发了没反应"让人困惑）✓
        case 'x':
            if (1U == cam_mode) { wifi_f4_send_string("ACK x: ignored (MD=1 needs dx=1)\r\n"); return; }
            cam_cmd_dx = cam_cmd_clamp(cam_cmd_num(s + 1), 1U, 8U);
            break;
        case 'y':
            if (1U == cam_mode) { wifi_f4_send_string("ACK y: ignored (MD=1 needs dy=1)\r\n"); return; }
            cam_cmd_dy = cam_cmd_clamp(cam_cmd_num(s + 1), 1U, 8U);
            break;

        case 'a': cam_cmd_fps = 0U; cam_cmd_gap_ms = 6U; cam_cmd_dx = 1U; cam_cmd_dy = 1U; break;

        case 'd':                                   // ★ 心跳档位：0 静默 / 1 精简(默认) / 2 全量
            cam_dbg_level = cam_cmd_clamp(cam_cmd_num(s + 1), 0U, 2U);
            break;

        case 'b':
            v = cam_cmd_num(s + 1);
            cam_cfg_uart_open((uint8_t)((v > 4U) ? 0U : v));
            cam_cfg_state = 0U;
            wifi_f4_send_string("ACK b: uart5 opened\r\n");
            return;

        case 'v':
            v = cam_cfg_read_ver();
            sprintf(b, "ACK v: got %lu bytes RV=%06lX LINE=%u\r\n",
                    (unsigned long)v, (unsigned long)(cam_cfg_ver & 0xFFFFFFUL),
                    (unsigned)cam_cfg_line_state());
            wifi_f4_send_string(b);
            return;

        // ★★ 配置串口"探活"：一次分清"线没接 / 波特率不对 / 其实回话了" ★★
        //   t      = 用 9600 探；t 1..4 = 用 115200/57600/38400/19200
        //   判读：
        //     EDGES=0            ⇒ **线上一点动静都没有** ⇒ 摄像头 TXD → PD2 那根线 / 焊点问题
        //     EDGES>0 且 RX=0    ⇒ **线是活的，但波特率/格式不对**（看 ERR 的 FE/NE）⇒ 换档再试
        //     RX>0               ⇒ **它回话了** ✓ 那就是我的解析问题
        // ★★ 一键配置摄像头：把总钻风配成 **188×120 + PCLK消隐=1 + 50fps**，并自动切到该采集模式 ★★
        //   K       = 用 50fps（手册推荐，避工频）
        //   K <n>   = 用 n fps（手册：1~500；想看慢帧就 K 5 / K 1）
        //   依据：手册 5.1 —— 分辨率 752×480 / 376×240 / 188×120 三档同视野；
        //         **"PCLK 消隐"用 STM32 的 DCMI 时必须设为 1**（仅 MT9V034 V2.1+ 支持）。
        //   ⚠️ 配完摄像头就出 188×120 了 ⇒ 固件必须同步切 `board_cam_set_mode(1)`，
        //      否则固件按 752 收、相机吐 188 ⇒ 数据错位（所以这里**自动切**）。
        //   回退：发 `m 0` 切回 752 模式（若摄像头没被改成功，画面就还能看）。
        case 'k':
            v = cam_cmd_num(s + 1);
            v2 = cam_cfg_apply_188((uint16_t)((v == 0U) ? 50U : v));
            board_cam_set_mode(1U);
            sprintf(b, "ACK k: fps=%lu cfg_reply=%lu bytes, mode=SMALL(188x120) CST=%u\r\n",
                    (unsigned long)((v == 0U) ? 50U : v), (unsigned long)v2,
                    (unsigned)cam_cfg_state);
            wifi_f4_send_string(b);
            return;

        // ★ 采集模式：`m 0` = 752 全分辨率（抽行）；`m 1` = 188×120（缓冲即整帧）
        case 'm':
            v = cam_cmd_num(s + 1);
            board_cam_set_mode((uint8_t)((v != 0U) ? 1U : 0U));
            sprintf(b, "ACK m: mode=%u rowbytes=%u span=%u\r\n",
                    (unsigned)cam_mode, (unsigned)cam_row_bytes, (unsigned)cam_span);
            wifi_f4_send_string(b);
            return;

        // ★★★ SCCB（I2C）探活 ★★★ —— **这才是这颗摄像头的正路**（它没有 51 配置芯片，
        //   转接板上的 TXD/RXD 实际就是传感器的 SDA/SCL）。
        //   两种接法都试；期望读出芯片版本 **0x1313**（MT9V034）。
        //   j        = 自动试两种接法
        //   j 0 / j 1 = 指定接法（0: PD2=SDA / PC12=SCL；1: 反过来）
        case 'j':
        {
            const char *p = s + 1U;
            uint8_t     has_arg = 0U;
            uint8_t     r;

            while ((*p == ' ') || (*p == '\t')) { p ++; }
            if ((*p >= '0') && (*p <= '9')) { has_arg = 1U; }

            // ★★ 进度标记：先回一条，再逐步回 —— 这样"卡在哪一步"一眼可见 ★★
            //   （如果连 `ACK j0` 都没有 ⇒ **命令根本没到板子**，问题在 PC→模块→板子的下行路径）
            wifi_f4_send_string("ACK j0: got cmd\r\n");

            if (0U == has_arg)
            {
                r = cam_sccb_test();
                wifi_f4_send_string("ACK j1: sccb done\r\n");     // SCCB 没卡住 ⇒ 走到这
                cam_pin_tristate_all();
                wifi_f4_send_string("ACK j3: tristate done\r\n"); // 三态检测没卡住 ⇒ 走到这
            }
            else
            {
                uint16_t id = 0U;

                v = cam_cmd_num(s + 1);
                cam_sccb_init((uint8_t)((v != 0U) ? 1U : 0U));
                r = cam_sccb_read_reg(0x0000U, &id);
                if (0U != v) { cam_sccb_id1 = id; } else { cam_sccb_id0 = id; }
                if ((id >> 8) == 0x13U) { r = (0U != v) ? 2U : 1U; }
                cam_pin_tristate_all();                           // 顺便测三态
            }

            sprintf(b, "ACK j: result=%u LV0=%u LV1=%u ID0=%04X ID1=%04X IDba=%04X err=%u\r\n",
                    (unsigned)r, (unsigned)cam_sccb_lv0, (unsigned)cam_sccb_lv1,
                    (unsigned)cam_sccb_id0, (unsigned)cam_sccb_id1, (unsigned)cam_sccb_ba,
                    (unsigned)cam_sccb_err);
            wifi_f4_send_string(b);

            //   三态判读：PU=1 & PD=0 ⇒ **空脚**（线没通/脚没引出）；PU=1 & PD=1 ⇒ 确实接着东西 ✓
            sprintf(b, "ACK jw: PC12 PU=%u PD=%u | PD2 PU=%u PD=%u   (PU=1&PD=0 = floating)\r\n",
                    (unsigned)cam_pin_c12_pu, (unsigned)cam_pin_c12_pd,
                    (unsigned)cam_pin_d2_pu, (unsigned)cam_pin_d2_pd);
            wifi_f4_send_string(b);
            return;
        }

        // ★★★ 下行链路诊断（`u`）：把"PC → 模块 → 板子"这条路一次查清 ★★★
        //   连续强读 30 次（每次内部最多等 1.6ms），然后报：
        //     TRY  = 真正发起读事务的次数     BUSY = 因为模块忙而没发起的次数
        //     OK   = 拿到 0x90/0x91 包头的次数（>0 ⇒ 协议对得上）
        //     CMD/LEN = 模块最近一次回的包头（**这个最关键**：不是 90/91 就说明协议不匹配）
        //     RXtot/RXnow = 累计收到 / 这次读到
        //   判读：
        //     BUSY 一直涨、TRY=0  ⇒ **模块一直"忙"** ⇒ 我们一次都没蹲到空闲窗口
        //     TRY>0 但 OK=0       ⇒ 事务发出去了，但模块回的不是数据包头 ⇒ 看 CMD
        //     RXnow>0             ⇒ **下行通了** ✓
        case 'u':
        {
            uint8_t  ubuf[32];
            uint32_t k, got = 0U;

            for (k = 0U; k < 30U; k++)
            {
                got += wifi_f4_read(ubuf, sizeof(ubuf));
            }
            cam_rx_bytes += got;

            sprintf(b, "ACK u: TRY=%lu BUSY=%lu OK=%lu CMD=%02X LEN=%u INT=%u RXtot=%lu RXnow=%lu\r\n",
                    (unsigned long)wifi_rd_try, (unsigned long)wifi_rd_busy, (unsigned long)wifi_rd_ok,
                    (unsigned)wifi_rd_cmd, (unsigned)wifi_rd_len, (unsigned)wifi_rd_int,
                    (unsigned long)cam_rx_bytes, (unsigned long)got);
            wifi_f4_send_string(b);
            return;
        }

        // ★★★ 下行"原始字节"dump（`o`）★★★ —— 看**收到的是不是我们发的那几个字符**
        //   打印最近 16 个字节的十六进制 + 当前攒行缓冲里的内容。
        //   判读：期望看到 `6A 0A`（= "j\n"）；若是别的（比如一堆积压的旧数据/二进制），
        //         说明模块回的不是我们的命令 ⇒ 问题在 PC 发送或模块缓冲。
        // ★ 位打包开关：`z` 一键切换（1=打包 8倍省带宽 / 0=原始 188 字节）
        case 'z':
            cam_pack = (uint8_t)(cam_pack ? 0U : 1U);
            sprintf(b, "ACK z: pack=%u\r\n", (unsigned)cam_pack);
            wifi_f4_send_string(b);
            return;

        // ★ 曝光：`e <行数>` 直接写 R0x0B（Total Shutter Width，单位=行），下一帧生效 ✓
        //   行 bin4 后帧只有 120 行 ⇒ 行数必须 < 120（出厂 443 会卡死不成帧 ✓）
        case 'e':
            v = cam_cmd_num(s + 1);
            cam_shutter = (uint8_t)cam_cmd_clamp(v, 10U, 110U);
            (void)cam_sccb_write_reg(0x0BU, cam_shutter);
            sprintf(b, "ACK e: shutter=%u rows\r\n", (unsigned)cam_shutter);
            wifi_f4_send_string(b);
            return;

        // ★ 翻转/镜像：`n` 每按一次换一种（0 不翻 → 1 左右镜像 → 2 上下 → 3 180° → 0…）
        //   传感器硬件翻转（R0x0D bit5/bit4），下一帧即生效 ✓ —— 装在车上方向不对就用它 ✓
        case 'n':
            cam_flip = (uint8_t)((cam_flip + 1U) & 3U);
            cam_sccb_apply_flip();
            sprintf(b, "ACK n: flip=%u (0=none 1=mirror 2=flip-vert 3=180)\r\n", (unsigned)cam_flip);
            wifi_f4_send_string(b);
            return;

        case 'o':
        {
            char     hex[96];
            uint32_t k, h = 0U;
            uint32_t base = (cam_rx_dump_i > 16U) ? (cam_rx_dump_i - 16U) : 0U;

            hex[0] = '\0';
            for (k = 0U; k < 16U; k++)
            {
                h += (uint32_t)sprintf(&hex[h], "%02X ", (unsigned)cam_rx_dump[(base + k) & 31U]);
            }

            cam_cmd_buf[cam_cmd_len] = 0;        // 安全终止（cam_cmd_len < 79）
            sprintf(b, "ACK o: N=%lu [%s] LEN=%u BUF=[%s]\r\n",
                    (unsigned long)cam_rx_dump_i, hex,
                    (unsigned)cam_cmd_len, cam_cmd_buf);
            wifi_f4_send_string(b);
            return;
        }

        case 't':
        {
            const char *p = s + 1U;
            uint8_t     has_arg = 0U;

            while ((*p == ' ') || (*p == '\t')) { p ++; }
            if ((*p >= '0') && (*p <= '9')) { has_arg = 1U; }

            if (0U == has_arg)          // ★★ 不带参数 ⇒ **一次扫完 7 档波特率**（推荐先发这个）★★
            {
                cam_cfg_scan();

                sprintf(b, "SCAN C: NP=%u PU=%u PD=%u   (PU=1 & PD=0 = PD2 floating / nothing connected)\r\n",
                        (unsigned)cam_probe_c_np, (unsigned)cam_probe_c_pu, (unsigned)cam_probe_c_pd);
                wifi_f4_send_string(b);

                sprintf(b, "SCAN RX: %lu %lu %lu %lu %lu %lu %lu\r\n",
                        (unsigned long)cam_scan_rx[0], (unsigned long)cam_scan_rx[1],
                        (unsigned long)cam_scan_rx[2], (unsigned long)cam_scan_rx[3],
                        (unsigned long)cam_scan_rx[4], (unsigned long)cam_scan_rx[5],
                        (unsigned long)cam_scan_rx[6]);
                wifi_f4_send_string(b);

                sprintf(b, "SCAN EG: %lu %lu %lu %lu %lu %lu %lu   (顺序 9600/115200/57600/38400/19200/4800/230400)\r\n",
                        (unsigned long)cam_scan_eg[0], (unsigned long)cam_scan_eg[1],
                        (unsigned long)cam_scan_eg[2], (unsigned long)cam_scan_eg[3],
                        (unsigned long)cam_scan_eg[4], (unsigned long)cam_scan_eg[5],
                        (unsigned long)cam_scan_eg[6]);
                wifi_f4_send_string(b);
                return;
            }

            // 带参数 ⇒ 单档细探（含 A/B/C 三段；短接 PC12↔PD2 时用来自检）
            v = cam_cmd_num(s + 1);
            cam_cfg_probe((uint8_t)((v > 6U) ? 0U : v));
            sprintf(b, "ACK t: baud=%lu | A: EDGES=%lu LOW=%lu/%lu RX=%lu ERR=%u | B: TXE=%lu RXE=%lu RX=%lu/%lu | C: NP=%u PU=%u PD=%u | LINE=%u\r\n",
                    (unsigned long)((v > 6U) ? 0U : v),
                    (unsigned long)cam_probe_edges, (unsigned long)cam_probe_low,
                    (unsigned long)cam_probe_n, (unsigned long)cam_probe_rx,
                    (unsigned)cam_probe_err,
                    (unsigned long)cam_probe_b_tx_edges, (unsigned long)cam_probe_b_edges,
                    (unsigned long)cam_probe_b_rx, (unsigned long)cam_probe_b_n,
                    (unsigned)cam_probe_c_np, (unsigned)cam_probe_c_pu, (unsigned)cam_probe_c_pd,
                    (unsigned)cam_cfg_line_state());
            wifi_f4_send_string(b);
            return;
        }

        case 'c':
            v  = cam_cmd_num(s + 1);                    // cmd
            v2 = cam_cmd_num(cam_cmd_arg2(s + 1));      // val
            cam_cfg_send_pkt((uint8_t)v, (uint16_t)v2);
            sprintf(b, "ACK c: sent A5 %02lX %02lX %02lX\r\n",
                    (unsigned long)(v & 0xFFUL), (unsigned long)((v2 >> 8) & 0xFFUL),
                    (unsigned long)(v2 & 0xFFUL));
            wifi_f4_send_string(b);
            return;

        case 'i':
            cam_cfg_send_pkt(0x00, 0x0000);
            wifi_f4_send_string("ACK i: INIT sent\r\n");
            return;

        case 'p':
            sprintf(b, "ACK p: LINE=%u (bit1=PC12/TX bit0=PD2/RX)\r\n",
                    (unsigned)cam_cfg_line_state());
            wifi_f4_send_string(b);
            return;

        case '?':
            wifi_f4_send_string("CMD f<n> g<ms> x<n> y<n> a s b<n> v c<cmd> <val> i p ?\r\n");
            return;

        default:
            break;
    }

    cam_cmd_status();       // 其它任何输入（含单独的 's'）都回一条状态，方便确认"它收到了"
}

// 主循环每轮调一次：把下行字节攒成行，逐行解析
static void cam_cmd_poll (void)
{
    uint8_t  b[32];
    uint32_t n, i;

    n = wifi_f4_read(b, sizeof(b));

    // ★★ 仪器化：**从模块里到底读回来多少字节**（心跳里显示成 `RX=`）★★
    //   🔑 用途：分清"命令没到板子"和"命令到了但没人理" ——
    //      发命令后 RX 涨 ⇒ **命令确实进来了**（那问题在后半段：解析/回执）；
    //      RX 不动     ⇒ **PC → 模块 → 板子 这条路断了**（上位机/模块/接收轮询的问题）。
    cam_rx_bytes += n;

    for (i = 0U; i < n; i++)
    {
        char c = (char)b[i];

        // ★ 记进"最近收到的字节"环形缓冲（`o` 命令会 dump 出来）
        cam_rx_dump[cam_rx_dump_i & 31U] = b[i];
        cam_rx_dump_i ++;

        if ((c == '\n') || (c == '\r'))
        {
            if (cam_cmd_len)
            {
                cam_cmd_buf[cam_cmd_len] = 0;
                cam_cmd_line(cam_cmd_buf);
                cam_cmd_len = 0U;
            }
        }
        else if (cam_cmd_len < (uint8_t)(sizeof(cam_cmd_buf) - 1U))
        {
            cam_cmd_buf[cam_cmd_len ++] = c;
        }
    }
}

//=========================================== ★★ 采集 / 发送 相位机 ★★ ===========================================
//   为什么需要它（这是"图上有横带"的**根因修复**）：
//     DMA 环形缓冲只存 120 行 ⇒ **一条行只活 4.2ms**；而"每 6ms 发一行图像"要和逐行收割
//     抢同一个主循环 ⇒ 一发 210 字节的 SPI 包就占住主循环几百微秒，
//     收割就赶不上 ⇒ 实测 `LG=119`（单次要补**整整一个缓冲**）⇒ **行在被读之前就被覆盖**。
//     后果：一张图里的 120 行其实来自**不同时刻/不同帧**（曝光还不一样）
//     ⇒ 画面出现**硬边平台 / 明暗横带**（用户看到的就是这个，不是灯光频闪）。
//
//   ✅ 做法：把时间**分相**，每一相只干一件事，谁也不抢谁：
//       相 0（歇）：等够 1000/fps ms（默认 1000ms ⇒ 1 帧/秒）
//       相 1（专心采）：主循环**只跑收割**（不发图、不刷心跳）⇒ 一帧 17.5ms 内干净凑齐 188×120
//       相 2（专心发）：冻住画面（`disp_hold=1`），每 g ms 发一行；发满 120 行 ⇒ 回相 0
//     ⇒ 每张图**铁定来自同一次采集** ⇒ 横带消失；刷新率精确等于 1/fps 秒一张。
//
//   🔑 为什么相 1 要等**两次**"凑齐"：刚恢复时收割指针还停在上一轮的位置（差 1 秒 = 上百行），
//      第 1 次"凑齐"其实是从旧位置接上去的**混杂**画面 ⇒ 必须丢掉，用第 2 次那幅干净的。
//   ⚠️ 相 1 带 300ms 超时兜底：摄像头真出问题时不能卡在这里不动（宁可发一张旧的）。
//=========================================== 采集/发送 相位机 ===========================================
static uint8_t  cam_phase     = 0U;      // 0=歇   1=采   2=发
static uint32_t cam_t_phase   = 0;
static uint32_t cam_t_tx      = 0;
static uint16_t cam_tx_row    = 0U;      // 当前正在发的行（发送块用它填 pkt[4] / 取像素）

// ★★ 两个"能一次判定画面为什么还不对"的诊断量（心跳的 S 行里显示）★★
//   TO = 相 1 **超时**（而不是"采齐了"）的次数 —— 非 0 ⇒ 这一幅是**没采齐就发出去的**（必花）
//   F  = 冻结那一刻 **实际填了多少行**（正常应 = 120 即 CAM_H）—— 小于 120 ⇒ 缺行
uint32_t cam_to_cnt  = 0U;
uint8_t  cam_frz_fill = 0U;

// 每个主循环轮调用一次。返回 1 = 这一轮该发一行（行号见 cam_tx_row）
static uint8_t cam_tx_tick (uint32_t now)
{
    if (0U == cam_phase)                        // ---------- 相 0：歇 ----------
    {
        uint32_t period = (cam_cmd_fps > 0U) ? (1000UL / (uint32_t)cam_cmd_fps) : 5UL;
        // ★ fps=0 时的周期：原来 60ms（隐式限速 ~16fps）⇒ 改 **5ms**（真正"不限"）✓

        if ((now - cam_t_phase) >= period)
        {
            // ★★ 开始新一轮：**计数 + 位图 + 三个标志**一起清（封装在 board_f4.c 里，
            //    因为"清零时刻必须和计数起点严格配套"——以前在回绕处顺手清零，
            //    突发补行一跨回绕就丢进度 ⇒ 永远数不到 120 ⇒ 没图）✓
            board_cam_new_pass();
            cam_phase   = 1U;
            cam_t_phase = now;
        }
        return 0U;
    }

    if (1U == cam_phase)                        // ---------- 相 1：专心采 ----------
    {
        uint8_t ready      = 0U;
        uint8_t by_timeout = 0U;                // ★ 是"超时"发的还是"采齐了"发的

        // `disp_done` 是**锁存**的（收割填满 120 行时置位，并且**同一瞬间已经冻住了画面**）
        //   ⇒ 这里没有任何时序压力：晚一点看到，画面也不会再被改写 ✓
        if (disp_done) { ready = 1U; }

        // ⚠️⚠️ 2026-10-15 修：**超时不再"就这么发出去"**！
        //   旧行为：等不到"采齐"（400ms）就 `ready = 1` ⇒ 把**半新半旧的混合帧**发出去 ✗
        //   ⇒ 现象就是**画面里时不时闪过一条横线**（那是"同一次采集没贯穿全帧"的接缝）✓
        //   现在：超时**只计数**（`TO` 会涨，方便发现问题），**不发送**，
        //   继续等下一帧采齐（`disp_done`）—— **宁可这一轮不发，也不发混合帧** ✓
        //   （真的卡死的话心跳里 `TO` 会一直涨、画面停住，一眼就能看出来 ✓）
        // ★★ 自恢复：1 秒内没等到"采齐"⇒ **重启一次 DCMI**（Stop+Start）★★
        //   🔑 理由：传感器的输出格式刚被 SCCB 改过（binning ⇒ 行数/像素时钟都变了），
        //      DCMI 若停在不同步状态就**永远等不到 VSYNC**（症状：心跳 `V=0 f=0 F=0`）✓
        //   ⚠️⚠️ **绝不能在这里量像素时钟**！`board_pclk_measure()` 会把 PA6 借走 20ms
        //      ⇒ DCMI 在"刚重启、正在等 VSYNC"的当口被掐掉像素时钟 ⇒ 永远同步不上 ✗✗
        //      （实测：R=0000/V=0/W=0 恒不变，就是每秒被掐一次导致的）✗
        //      ⇒ 时钟测量已改成**只在配置后量一次**（见上面 CAM CFG）✓
        //   （旧代码在这里"每轮循环 TO++"也是个 bug ⇒ TO 一秒涨几千 ✓ 已修）
        if ((now - cam_t_phase) >= 1000UL)
        {
            char cb[160];

            // ★★★ 2026-10-20 关键修正：自愈重启只在"真没帧"时才做！★★★
            //   实测（CAM1）：同步成功之后，**每秒一次的无条件重启把采集压到 ~1.2fps**
            //   （f 每秒 +1 = "每次重启后只采到 1 帧就被我打断"的铁证 ✓）
            //   ⇒ 只有"连续 1 秒一张新帧都没有"才重启；正常时绝不碰 DCMI ✓
            if (cam_frame_cnt == t_last_frame)
            {
                cam_to_cnt ++;
                (void)board_cam_restart();      // ★ 真卡死了才 Stop + Start
                board_cam_new_pass();           // 计数/位图清零，重新开始数
            }
            t_last_frame = cam_frame_cnt;

            dcmi_ris = DCMI->RISR;              // 只读快照（无副作用 ✓）
            t_vsync_hz = board_vsync_hz();      // 量 PB7(VSYNC) 的真实边沿率（Hz 分辨率 ✓）

            // ★★ 卡住时**每秒自动打一条全量诊断**（用户零操作）★★
            //   ND   = DMA 剩余传输数：**钉死在 5640 = 一个字节都没搬**（关键判据！）
            //   CR   = DCMI->CR：基准 0x4083 = ENABLE + CAPTURE + 极性组 1
            //   SR/RIS = DCMI 状态/原始中断：**全 0 ⇒ 它一点动静都没有**
            //   AFD  = 哪几根 DCMI 引脚没配成 AF13（0 = 11 根全对）
            //   DEN  = DMA2_Stream1 的 EN 位（期待 1）      PF = 实测像素时钟(kHz)
            sprintf(cb, "CAM1 f=%lu er=%lu CR=%04lX ND=%lu SR=%04lX RIS=%04lX AFD=%03lX DEN=%lu PF=%luk V2=%luk CLK=%u MHz=%lu\r\n",
                    (unsigned long)cam_frame_cnt, (unsigned long)cam_err_cnt,
                    (unsigned long)dbg_dcmi_cr, (unsigned long)dbg_dma_ndtr,
                    (unsigned long)dbg_dcmi_sr, (unsigned long)dbg_dcmi_ris,
                    (unsigned long)dbg_pin_bad, (unsigned long)dbg_dma_en,
                    (unsigned long)dbg_pclk_khz, (unsigned long)t_vsync_hz,
                    (unsigned)board_clock_src, (unsigned long)(board_sysclk_hz / 1000000UL));
            wifi_f4_send_string(cb);

            cam_t_phase = now;
        }

        if (ready)
        {
            board_cam_analyze();                 // ★★ 全部在单片机内：Otsu 二值化 + 每行亮/暗交界列 ★★
            if (by_timeout) { cam_to_cnt ++; }   // ★ 没采齐就发 ⇒ 这一幅必有缺陷（心跳里看 TO）
            cam_frz_fill = disp_fill;            // ★ 冻结那一刻实际填了多少行（正常 = 120）

            disp_hold  = 1U;                     // （正常情况下收割早已自己冻好了，这里是双保险）
            cam_tx_row = 0U;
            cam_t_tx   = now;
            cam_phase  = 2U;
        }
        return 0U;
    }

    // ---------- 相 2：专心发 ----------
    if ((now - cam_t_tx) >= (uint32_t)cam_cmd_gap_ms)
    {
        cam_t_tx = now;
        return 1U;                              // ← 让主循环把这一行发出去
    }
    return 0U;
}

// 主循环发完一行后调用：推进行号；发满 ⇒ 解冻 + 回相 0（开始计时下一次）
static void cam_tx_row_done (void)
{
    uint16_t txh = (uint16_t)((uint16_t)CAM_H / (uint16_t)cam_cmd_dy);

    cam_tx_row ++;

    if (cam_tx_row >= txh)
    {
        // ★ 边界包已取消（用户要求不画蓝线）⇒ 不再发 0xFE 包，省 132 字节/帧 ✓
        //   二值化仍然在 board_cam_analyze() 里做（Otsu 那部分保留）✓
        cam_tx_row  = 0U;
        disp_hold   = 0U;                       // 解冻
        cam_phase   = 0U;                       // 回"歇"
        // ★ 计时起点要**回拨到"这张图开始发"的那一刻**，否则周期会变成
        //   "1000/fps + 发送耗时"（实测 1 秒的设定会变成 1.7 秒一张）。
        //   发送耗时 ≈ 行数 × 每行间隔 ⇒ 直接减掉即可，不用再加一个变量。
        cam_t_phase = HAL_GetTick() - (uint32_t)txh * (uint32_t)cam_cmd_gap_ms;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // ★ 重新配时钟到 168MHz（CubeMX 生成的 SystemClock_Config 里 HSE 值可能不对）
  //   调试时看这两个全局变量：
  //      board_clock_src  = 1 用了外部晶振 / 2 晶振没起来退回了内部 HSI
  //      board_sysclk_hz  = 实际系统时钟，应该是 168000000
  board_clock_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DCMI_Init();
  /* USER CODE BEGIN 2 */

  // ★★★ 记录"这次到底是**为什么复位**的"（硬件诊断第一手资料）★★★
  //   RCC->CSR 的高位是**粘滞**的复位来源标志，读一次就知道：
  //     bit25 BORRSTF  = **欠压/掉电**（供电被拉低 —— 摄像头电流大时最可能）
  //     bit26 PINRSTF  = 复位引脚被拉低（按键/RST）
  //     bit27 PORRSTF  = 上电复位
  //     bit28 SFTRSTF  = 软件复位
  //     bit29 IWDGRSTF = 独立看门狗
  //     bit30 WWDGRSTF = 窗口看门狗
  //     bit31 LPWRRSTF = 低功耗复位
  //   ⚠️ 读完要写 RMVF 清掉，否则下次看到的还是老标志。
  cam_rst_flags = (uint32_t)((RCC->CSR >> 25) & 0x7FU);
  RCC->CSR |= RCC_CSR_RMVF;

  // ---- 板载 LED（PB2）：没有屏幕，靠它看状态 ----
  board_led_init();

  // ==================== 摄像头（DCMI + DMA）====================
  //   MX_DCMI_Init() 已经把 DCMI 外设配好了（8位 / 硬件同步 / VSYNC低有效 / PCLK上升沿），
  //   这里补的是：DMA + 两个中断 + 启动第一帧。
  board_cam_init();

  // ==================== WiFi 模块：复位 → 连热点 → 连上位机 ====================
  //   ⚠️ 阻塞，最长约 60 秒。⚠️ **上位机（VOFA）要先把 TCP 服务端开着**，否则这一步会失败。
  //   返回值 = 卡在哪一步，直接由 LED 闪出来：
  //       1 = 模块没应答(SPI/供电/RST) → 闪 1 下
  //       2 = 连不上热点(SSID/密码/不是2.4G) → 闪 2 下
  //       3 = 连不上电脑(IP不对 / 上位机没开 / 防火墙) → 闪 3 下
  //       0 = 成功 → LED 常亮
  {
    uint8_t ret = wifi_f4_init();
    board_led_show((0 == ret) ? LED_CODE_READY : ret);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 几个跨轮次保持的状态（static：只初始化一次）
    static uint8_t  cam_min = 255;      // 最近一帧的最暗值
    static uint8_t  cam_max = 0;        // 最近一帧的最亮值
    static uint32_t cam_avg = 0;        // 最近一帧的平均亮度
    static uint32_t t_retry = 0;
    static uint32_t t_hb    = 0;
    static uint32_t t_camprobe = 0;      // ★ 自动探测摄像头的节流（见下面那段）
    // ---- 还没连上就每 3 秒自动重试一次 ----
    //   ★ 很关键：wifi_f4_init() 只在开机时连一次 socket。如果那一刻上位机没开，
    //     就会永远连不上、一条数据都不发。这里让它自己重连，**上位机后开也能接上**。
    if (!wifi_f4_is_ready())
    {
      if ((HAL_GetTick() - t_retry) > 3000)
      {
        t_retry = HAL_GetTick();
        board_led_show(wifi_f4_retry() ? wifi_f4_last_err() : LED_CODE_READY);
      }
    }

    // ---- 板载 LED 的闪烁节奏（不阻塞）----
    board_led_task();

    // ---- 轮询 DCMI 状态标志（排"f=0"用；只读寄存器，很快）----
    board_cam_poll();

    // ---- ★ 全画面：把 DMA 刚写完的行收割进 disp_img（每 4 行取 1 行、每 4 列取 1 列）----
    //   ⚠️ 必须在**主循环每轮都跑**：一条行在 90KB 环形缓冲里只活 **4.2ms**，
    //      一旦被覆盖就收割不到了 ⇒ 绝不能放进有节流（6ms/500ms）的块里。
    board_cam_harvest();

    // ---- ★ 实时命令：把上位机发来的字节攒成行并**立刻执行**（改帧率/抽点/配摄像头）----
    //   ⚠️ 和收割一样必须**每轮都跑**：塞进带节流的块里会让命令延迟几百毫秒才生效，
    //      调参时就会"发了半天没反应"，很难判断是命令没到还是参数没作用。
    cam_cmd_poll();

    // ---- ★ 引脚"悬浮/断线"检测（**已关闭**，2026-10-15）----
    //   🔥 遗留问题清理：它会在**开机后 9 秒内**给 DCMI 那 4 根引脚（PCLK/VSYNC/HREF/D0）
    //      轮流加内部上拉/下拉 ⇒ **直接干扰正在进行的采集** ✗
    //   当年它是"查断线"的诊断，现在接线已经确认没问题 ⇒ 只会添乱 ⇒ **关掉** ✓
    //   需要时再打开（把它改回调用即可）。
    // board_float_probe();

    // ---- ★ 极性轮换：**必须开着**（只在不花的 3/4/5/6/7 之间轮换）----
    //   🔥 2026-10-14 用户实测纠正：**拆掉 FLASH 之后一直是有图的**（只是传输质量差），
    //      是"我删掉极性轮换"之后才变黑的 ⇒ **轮换是必需的环节**。
    //   为什么必需：`board_cam_apply_polarity()` 每次都会**改变极性的那三个位**，
    //      而"位发生变化"本身会**强制 DCMI 重新同步**。
    //      我后来写的"每 200ms 重写同样的值"——值没变 ⇒ 等于什么都没做 ⇒ 所以没用。
    //   ⇒ 恢复轮换，但**只在"不花"的几组之间轮换**（0/1/2 会花，已排除）。
    board_cam_pol_search();

    // ==================== 摄像头：统计（**默认不算**！）====================
    //   🔥🔥🔥 2026-10-15 **这是"画面被切成几段、接缝"的真凶**：
    //     `cam_ready` 是 **DMA 搬满一圈(120 行)** 时置位的（`HAL_DCMI_ConvCpltCallback`），
    //     而一帧有 480 行 ⇒ **一帧置 4 次、约 228 次/秒**！
    //     下面这个统计要**循环 22560 字节**（disp_img 全图求 min/max/avg）
    //     ⇒ 每秒要跑 500 万次循环，**每次把主循环堵住约 1ms**
    //     ⇒ 逐行收割跟不上（`LG` 尖峰）⇒ 画面被切成几段、段间错位。
    //     而且它是"每帧固定 4 次、时刻固定" ⇒ **接缝永远在同一个位置** ✓ 与实测完全吻合。
    //   ✅ 现在：**只在 `d 2` 看诊断时**才算，并且**每 500ms 最多一次**。
    //      （cam_min / cam_max / cam_avg 只有 CAM2 心跳会用 ⇒ 平时根本不需要。）
    static uint32_t t_stat = 0;
    if (cam_ready)
    {
      cam_ready = 0;

      if ((cam_dbg_level >= 2U) && ((HAL_GetTick() - t_stat) >= 500UL))
      {
        t_stat = HAL_GetTick();
        uint32_t i;
        uint32_t n = (uint32_t)(CAM_H * CAM_TXW);   // ★ 现在统计的是 disp_img（188×120 全画面缩略图）
        uint8_t *p = &disp_img[0][0];
        uint8_t  mn  = 255;
        uint8_t  mx  = 0;
        uint32_t sum = 0;

        for (i = 0; i < n; i ++)
        {
          uint8_t v = p[i];
          if (v < mn) { mn = v; }
          if (v > mx) { mx = v; }
          sum += v;
        }
        cam_min = mn;
        cam_max = mx;
        cam_avg = sum / n;
      }

      // ⚠️ 连续模式 + 循环 DMA 下**不需要重启**，DCMI 一直在采
      //   （这是"确认有图"那版的做法；快照模式/裁剪窗口/CAM_H=240 那几版都已撤销）
    }

    // ==================== ★ 图传：一行一小包，**每轮主循环都推** ★ ====================
    //   🔥 2026-10-14 修的一个关键"位置"错误：
    //      这段原来放在上面 `if (cam_ready)` 的**里面**，而 `cam_ready` 只在 DMA 搬完一圈时
    //      才置位（约 15 次/秒）⇒ **行速率被卡在 ~15 行/秒**，60 行要 4 秒
    //      ⇒ 现象就是"图一条条慢慢长出来、大片是黑的"。
    //      移到主循环层级（用自己的 6ms 节流）后 ⇒ 约 166 行/秒 ⇒ 整帧约 0.36s。
    //
    //   包格式： 0xAA 0x55 0xA5 0x5A | 行号 | W | 极性组号 | H | 像素[W] | 校验和高8 低8
    if (wifi_f4_is_ready())
    {
      // ⚠️ 这里原来有 t_tx / t_img / tx_row 三个 static（节流 + 行号）。
      //    现在**全部搬进相位机** `cam_tx_tick()` / `cam_tx_row_done()` —— 见上面的说明。
      //    （"边收边发抢主循环"正是横带的根因，所以这一块必须整体交给相位机管。）

      // ==================== ★★ 实时旋钮：帧率 / 抽点（上位机随时可改）★★ ====================
      //   txw / txh = 这一张图**实际发**的列数 / 行数。
      //   🔑 为什么"抽点"是关键：发送速率是 g ms/行 ⇒ **数据量直接决定刷新率**。
      //      x=2 y=2 ⇒ 94×60 = 5640 字节/张，比 188×120 少 4 倍 ⇒ 同样的 g 下快 4 倍。
      //      （抽点是在发送时做的，disp_img 里始终是完整的 188×120 ⇒ **视野不会被裁掉**）
      uint16_t txw = (uint16_t)((uint16_t)CAM_TXW / (uint16_t)cam_cmd_dx);
      uint16_t txh = (uint16_t)((uint16_t)CAM_H   / (uint16_t)cam_cmd_dy);
      uint8_t  nbytes = cam_pack ? (uint8_t)((txw + 7U) / 8U) : (uint8_t)txw;  // 线上载荷长度
      uint32_t now = HAL_GetTick();

      // ★★★ 相位机问一句："这一刻该不该发一行？"（采集/冻结/节流全在它里面）★★★
      if (cam_tx_tick(now))
      {
        // ★★ 拼成"10 字节包头 + txw 字节载荷 + 2 字节校验"，**一次 SPI 事务发完** ★★
        //   包头第 8/9 字节带 ER / PIN，上位机显示在窗口标题上 ⇒ 不依赖会被截断的文本心跳。
        // ═══════ 发送：默认"**整帧一包**"（按 z 关掉打包则退回逐行小包）═══════
        //   ★ 为什么：模块对**每个包**都有固定开销（CS/等待/应答 ≈ 1~2ms）
        //     120 个包 ⇒ 120~240ms/帧 ⇒ **这才是只有 5~7fps 的真凶** ✓
        //     合成 1 包后包开销几乎消失 ⇒ 帧率大幅提升 ✓✓
        if (cam_pack)
        {
            /* 整帧包：magic(4) | 0xFF | 行数 | 极性|0x80 | H | 长度高 | 长度低 | 载荷 | 校验高 低 */
            static uint8_t fpkt[12U + 120U * 24U + 2U];
            uint16_t flen = 0U;
            uint16_t r2, i2, k2;

            fpkt[0] = 0xAA; fpkt[1] = 0x55; fpkt[2] = 0xA5; fpkt[3] = 0x5A;
            fpkt[4] = 0xFF;
            fpkt[5] = (uint8_t)txh;
            fpkt[6] = (uint8_t)(dbg_pol_idx | 0x80U);
            fpkt[7] = (uint8_t)CAM_H;

            for (r2 = 0U; r2 < txh; r2++)
            {
                const uint8_t *src = &disp_img[r2 * (uint16_t)cam_cmd_dy][0];
                for (i2 = 0U; i2 < nbytes; i2++)
                {
                    uint8_t byte = 0U;
                    for (k2 = 0U; k2 < 8U; k2++)
                    {
                        uint16_t col = (uint16_t)(i2 * 8U + k2);
                        if (col < txw)
                        {
                            if (src[col * (uint16_t)cam_cmd_dx] >= 128U) { byte |= (uint8_t)(1U << (7U - k2)); }
                        }
                    }
                    fpkt[12 + flen] = byte;
                    flen ++;
                }
            }
            fpkt[8] = (uint8_t)(flen >> 8);
            fpkt[9] = (uint8_t)flen;
            {
                uint16_t ck = 0U;
                for (i2 = 0U; i2 < flen; i2++) { ck = (uint16_t)(ck + fpkt[12 + i2]); }
                fpkt[12 + flen]     = (uint8_t)(ck >> 8);
                fpkt[12 + flen + 1] = (uint8_t)ck;
            }

            /* ★★★ **续发**：`wifi_f4_send` 模块一忙就只发出一半，
               若上层从头重发 ⇒ 上位机收到"半包+整包" ⇒ **数据错位 ⇒ 随机竖线** ✗✗
               现在记住"已发多少"，下一轮只发**剩余部分** ✓✓ */
            {
                uint32_t rem;

                if (0U == fpkt_sent) { fpkt_total = (uint32_t)(12U + flen + 2U); }

                rem = wifi_f4_send(fpkt + fpkt_sent, fpkt_total - fpkt_sent);
                fpkt_sent = fpkt_total - rem;

                if (0U == fpkt_sent)                       /* 整包真正发完了：结束这一帧 */
                {
                    fpkt_total = 0U;
                    cam_tx_row = (uint16_t)(txh - 1U);
                    cam_tx_row_done();
                }
            }
        }
        else
        {
        uint8_t  pkt[10 + CAM_TXW + 2];
        uint16_t s = 0, c2;

        // ⚠️ 冻结 / 计时 / 行号 / 相位 —— 全部由 `cam_tx_tick()` 和 `cam_tx_row_done()` 管好，
        //    这里**只管把这一行拼出来发掉**。原来那段"发图期间冻结收割 + 帧率节流"的逻辑
        //    已经整个搬进相位机（"边收边发抢主循环"就是横带的根因）。

        pkt[0] = 0xAA; pkt[1] = 0x55; pkt[2] = 0xA5; pkt[3] = 0x5A;
        pkt[4] = (uint8_t)cam_tx_row;               // 行号（相位机维护）
        pkt[5] = (uint8_t)nbytes;                   // 线上载荷字节数（打包时 = 24；不打包 = txw）
        pkt[6] = (uint8_t)(dbg_pol_idx | (cam_pack ? 0x80U : 0x00U));  // ★ 最高位 = 位打包标志
        pkt[7] = (uint8_t)txh;                      // 总行数（默认 120；y 抽点后变小）
        // pkt[8..9] 原来放"每帧字节数"，但实测 DCMI 不分行、帧数也数不准 ⇒ 
        //   **pkt[9] 改成"活引脚电平快照"**（在 AF 模式下 IDR 照样能读到真实电平）：
        //      bit3 = PA6(PCLK)   bit2 = PB7(VSYNC)   bit1 = **PA4(HREF)**   bit0 = PC6(D0)
        //   🔑 这个字节能直接回答"HREF 到底还活着没有"，比万用表还快。
        // pkt[8] = **DCMI 看门狗"重启过几次"的低 8 位**（`cam_err_cnt`）
        //   🔑 它同时是"新固件有没有跑起来"的铁证：新版会不断重启/保活 ⇒ 这个数在涨；
        //      旧版这个数恒为 0。显示在窗口标题上（`ER=`），不会像控制台那样被截断看不见。
        // pkt[9] = 活引脚电平快照： bit3=PA6(PCLK) bit2=PB7(VSYNC) bit1=**PA4(HREF)** bit0=PC6(D0)
        pkt[8] = (uint8_t)(cam_err_cnt & 0xFFU);
        pkt[9] = (uint8_t)(((GPIOA->IDR & GPIO_PIN_6) ? 8U : 0U) |     // PCLK  PA6
                           ((GPIOB->IDR & GPIO_PIN_7) ? 4U : 0U) |     // VSYNC PB7
                           ((GPIOA->IDR & GPIO_PIN_4) ? 2U : 0U) |     // HREF  PA4  ★
                           ((GPIOC->IDR & GPIO_PIN_6) ? 1U : 0U));     // D0    PC6

        if (cam_pack)
        {
          // ★ 位打包：每 8 个像素 → 1 字节（≥128 记 1）；188 像素 = 24 字节 ✓
          uint8_t i2, k2;
          for (i2 = 0; i2 < nbytes; i2++)
          {
            uint8_t byte = 0U;
            for (k2 = 0; k2 < 8U; k2++)
            {
              uint16_t col = (uint16_t)((uint16_t)i2 * 8U + k2);
              if (col < txw)
              {
                uint8_t v = disp_img[(uint16_t)cam_tx_row * (uint16_t)cam_cmd_dy][col * (uint16_t)cam_cmd_dx];
                if (v >= 128U) { byte |= (uint8_t)(1U << (7U - k2)); }
              }
            }
            pkt[10 + i2] = byte;
            s = (uint16_t)(s + byte);
          }
        }
        else
        for (c2 = 0; c2 < txw; c2++)
        {
          // ★ 从 **disp_img** 取（"边采边抽点"出来的全画面缩略图）——
          //   一级抽点（每 4 列取 1 → 188 列、每 4 行取 1 → 120 行）在 `board_cam_harvest()` 里做；
          //   这里是**二级抽点**（发送时按 x/y 再抽）⇒ 数据量随时可控，而**视野一点不丢**。
          pkt[10 + c2] = disp_img[(uint16_t)cam_tx_row * (uint16_t)cam_cmd_dy][c2 * (uint16_t)cam_cmd_dx];
          s            = (uint16_t)(s + pkt[10 + c2]);
        }
        pkt[10 + nbytes]     = (uint8_t)(s >> 8);
        pkt[10 + nbytes + 1] = (uint8_t)s;

        // ★★ 发这一行：**返回非 0 = 模块忙、没发完** ⇒ **不推进行号**，下一轮再试 ★★
        //   🔑 为什么必须这样：`wifi_f4_send` 原来会**死等最多 1 秒**（模块一忙就卡住主循环）
        //      ⇒ 现象就是"第一条命令有反应，之后全没反应"（主循环被按住，命令排不进来）、
        //      同时逐行收割崩掉（`LG` 飙到 117）。现在改成"忙就立刻返回"，由这里重试 ✓
        if (0U == wifi_f4_send(pkt, (uint32_t)(10U + nbytes + 2U)))
        {
            cam_tx_row_done();      // 发出去了才推进行号（满 txh 行它会自动解冻 + 回相 0）
        }

        }
        // 否则：什么都不做，下一轮（≥ g ms 后）再发同一行 —— 主循环**永远不会被卡住** ✓
      }
    }

    // ==================== ★★ 自动探测摄像头（**用户不用敲任何命令**）★★ ====================
    //   开机先探一次；**没找到就每 10 秒再试一次**（这样"插线/换接法"之后不用复位也能被发现）。
    //   探测内容 = 软件 SCCB（两种 SDA/SCL 顺序 × 地址 0x90/0xBA），读芯片版本寄存器 R0x0000。
    //   ⚠️ 单次约 10~15ms（软件 I2C 的固有开销）⇒ 每 10 秒一次会让 `LG` 尖一下，
    //      但换来"完全不用人工敲命令"，值得 ✓（找到之后就不再探了）
    //   ⚠️ 现在是"**开机立刻探第一次**"，之后**每 1 秒**重试 ——
    //      快间隔是**故意的**：用它来做"**边碰线边找脚**"（改完硬件不用重烧、不用复位）✓
    if ((0U == cam_if_ok) && ((0U == cam_if_tries) || ((HAL_GetTick() - t_camprobe) >= 1000UL)))
    {
        uint8_t rr;

        t_camprobe  = HAL_GetTick();
        cam_if_tries ++;
        rr = cam_sccb_test();

        if (0U != rr)
        {
            cam_if_ok = rr;
            cam_if_id = (0U != cam_sccb_id0) ? cam_sccb_id0
                      : ((0U != cam_sccb_id1) ? cam_sccb_id1 : cam_sccb_ba);

            // ★★ 一旦找到，**立刻**单独打一行（不等下一个心跳）——"边碰线边找脚"时好读 ✓
            {
                char cb[64];

                sprintf(cb, "CAM FOUND! result=%u ADDR=%02X ID=%04X\r\n",
                        (unsigned)cam_if_ok, (unsigned)cam_sccb_addr, (unsigned)cam_if_id);
                wifi_f4_send_string(cb);

                // ★★ 紧接着**自动配置**（依然零命令）：配成 188×120（列 bin4 + 行 bin4），
                //    这样一帧正好 = 22560 字节 = DMA 缓冲大小 ⇒ 缓冲天然就是一整帧 ✓
                cam_sccb_set_bin4();
                // ★★ 量**一次**像素时钟（⚠️ 只能在这里量！）：
                //   `board_pclk_measure()` 会把 PA6 借走 20ms，若在 DCMI"重启后等 VSYNC"的当口量，
                //   就等于**把它的像素时钟掐掉** ⇒ 永远同步不上（实测 R=0000/V=0/W=0 恒不变）✗
                //   K≈1134kHz ⇔ 188×120@约50fps ⇒ 传感器活着、binning 生效 ✓
                t_pclk = board_cam_pclk_khz();
                sprintf(cb, "CAM CFG: bin4 R0D=%04X SHUT=%04X ok=%u PCLK=%lukHz -> switch 188x120\r\n",
                        (unsigned)cam_r0d, (unsigned)cam_r0e, (unsigned)cam_bin_ok, (unsigned long)t_pclk);
                wifi_f4_send_string(cb);
            }

            board_cam_set_mode(1U);      // ★ 切到 188 宽模式（cam_row_bytes=188 / span=120 / dcol=1）
        }
    }

    // ==================== ★ 心跳：每 500ms **无条件**发一条 ★ ====================
    //   🔥 上一版这里是"只在收到摄像头帧时才发"——结果摄像头一出问题，
    //      **一个字节都发不出去**，看起来像"WiFi 挂了"，极难排查。
    //      ⇒ 输出绝不能挂在另一个可能坏掉的外设上。心跳永远发，摄像头状态放在内容里。
    // ⚠️ 正在发整帧包时不能发文本（会插进二进制包 ⇒ 解错 ⇒ 竖线）
    if (wifi_f4_is_ready() && (0U == fpkt_sent) && ((HAL_GetTick() - t_hb) >= 500))
    {
      char buf[320];
      t_hb = HAL_GetTick();
      board_cam_dbg();                            // 抓一份 DCMI/DMA 寄存器快照

      // ==================== ★★ 心跳分档（上位机命令 `d 0/1/2`）★★ ====================
      //   这一段是**控制台噪声的总开关**：默认只发上面这**一行**，
      //   下面 8 行全量诊断（CAM1/CAM2/CAM3/BPF/MD/PIN/P）只在 `d 2` 时才发。
      if (cam_dbg_level >= 1U)
      {
        //   f = 轮询 FRAME 标志数到的帧数      V = **VSYNC 中断**进了几次
        //   TO = 相 1 超时（没采齐就发）次数   F  = 冻结那一刻实际填了几行（正常 120）
        //   LG = 单次收割最多补了几行（>30 就是在漏行）   P = 当前极性组号
        //   ★ 把"摄像头状态"直接放进心跳 —— 因为**开机时那一次单独发送不一定到得了**
        //     （实测：`CAM FOUND!` 那种"开机瞬间单独发一行"会丢），而心跳是一直在发的、稳 ✓
        //     MD  = 当前采集模式（0 = 752 宽 / 1 = 188 宽）
        //     BIN = 188×120 配置是否成功（1 = 已写并回读一致）
        //     R0D/R0E = 两个 Read Mode 寄存器的**回读值**（列 bin 在 R0D[3:2]、行 bin 在 R0E[1:0]）
        sprintf(buf, "S f=%lu V=%lu LG=%lu TO=%lu F=%u P=%u RX=%lu CAM=%u ID=%04X N=%u MD=%u BIN=%u R0D=%04X W=%u E=%u K=%lu R=%04lX\r\n",
                (unsigned long)cam_frame_cnt, (unsigned long)cam_vsync_isr_cnt,
                (unsigned long)disp_lag_max,
                (unsigned long)cam_to_cnt, (unsigned)cam_frz_fill,
                (unsigned)dbg_pol_idx,
                (unsigned long)cam_rx_bytes,
                (unsigned)cam_if_ok, (unsigned)cam_if_id, (unsigned)cam_if_tries,
                (unsigned)cam_mode, (unsigned)cam_bin_ok,
                (unsigned)cam_r0d,
                (unsigned)board_cam_write_row(), (unsigned)cam_last_start_err,
                (unsigned long)t_pclk, (unsigned long)dcmi_ris);
        wifi_f4_send_string(buf);
      }

      if (cam_dbg_level >= 2U)
      {
      // ---------- 以下为全量诊断（`d 2` 才发，共 8 行）----------

      // ---------- 第 1 行：DCMI / DMA 到底有没有在跑 ----------
      //   f  = 完整帧数        fn = FIFO非空次数      er = DCMI 报错次数
      //   ec = VSYNC 边沿数(500ms内) —— 证明那是"真场同步"而不是噪声
      //   CR = DCMI->CR(基准 4083 = ENABLE+CAPTURE+VSYNC低有效)
      //   ND = DMA 剩余传输数            **必须往下掉**；钉死不动的 5640 = 一个字节都没搬
      //   SR / RIS = DCMI 的状态/原始中断寄存器（RAW）。**全 0 就说明它一点动静都没有**
      //   AFD = 哪几根引脚没配成 **AF13**（MODER 或 AFR 任一不对都算，0 = 11 根全对）
      //   DEN = DMA2_Stream1 的 EN 位（期待 1）    DE = DMA 错误码
      //   PF  = **PA6 上那根线的实测频率(kHz)** ← 用 TIM3 外部时钟量出来的，唯一没量过的量
      //         理想值：MHz 级（188×120@60fps ≈ 1400kHz）。若只有 ~60 ⇒ 接的是 VSYNC；
      //         若 ~7000 ⇒ 接的是 HREF；若是 0 ⇒ 这根线根本没在跳。
      sprintf(buf, "CAM1 f=%lu fn=%lu er=%lu ec=%lu CR=%04lX ND=%lu SR=%04lX RIS=%04lX AFD=%03lX DEN=%lu DE=%lu PF=%luk\r\n",
              (unsigned long)cam_frame_cnt, (unsigned long)cam_fne_cnt,
              (unsigned long)cam_err_cnt, (unsigned long)dbg_vsync_edge,
              (unsigned long)dbg_dcmi_cr, (unsigned long)dbg_dma_ndtr,
              (unsigned long)dbg_dcmi_sr, (unsigned long)dbg_dcmi_ris,
              (unsigned long)dbg_pin_bad, (unsigned long)dbg_dma_en, (unsigned long)dbg_dma_err,
              (unsigned long)dbg_pclk_khz);
      wifi_f4_send_string(buf);

      // ---------- 第 2 行：★ 引脚"悬浮/断线"检测（决定性）----------
      //   每组三个数 = **无拉 / 上拉 / 下拉** 时的"高电平占比(%)"，单位 %
      //   顺序： PCLK(PA6)   VSYNC(PB7)   HREF(PA4)   D0(PC6)
      //   判读： 三个数**基本一样**            ⇒ 有真信号在驱动这根线（内部 40k 拉不动它）
      //          出现 **上拉≈100 / 下拉≈0**   ⇒ 这根线**悬空/断了**（或摄像头没驱动它）
      //   CST = 摄像头串口配置结果：0=还没做 1=成功 2=没收到应答
      //   RV  = ★ 发 GET_VERSION 后**收到的原始字节**（最后 4 个）—— 判断"摄像头有没有说话"
      //          RV=00000000 ⇒ 它根本没答（查 TXD 是否真的到了 PD2 / 波特率 / 摄像头供电）
      //          RV=A5xxxx00 ⇒ **它答了**，只是我们没解析对（那就能对症下药）
      sprintf(buf, "CAM2 CST=%u RV=%08lX FLT %lu/%lu/%lu  %lu/%lu/%lu  %lu/%lu/%lu  %lu/%lu/%lu | mnmx=%u/%u avg=%lu | clk=%luM\r\n",
              (unsigned)cam_cfg_state, (unsigned long)cam_cfg_ver,
              (unsigned long)dbg_flt[0][0], (unsigned long)dbg_flt[0][1], (unsigned long)dbg_flt[0][2],
              (unsigned long)dbg_flt[1][0], (unsigned long)dbg_flt[1][1], (unsigned long)dbg_flt[1][2],
              (unsigned long)dbg_flt[2][0], (unsigned long)dbg_flt[2][1], (unsigned long)dbg_flt[2][2],
              (unsigned long)dbg_flt[3][0], (unsigned long)dbg_flt[3][1], (unsigned long)dbg_flt[3][2],
              (unsigned)cam_min, (unsigned)cam_max, (unsigned long)cam_avg,
              (unsigned long)(board_sysclk_hz / 1000000UL));
      wifi_f4_send_string(buf);

      // ---------- 第 3 行：★「行亮度剖面」—— 一眼看出哪些行是黑的 ----------
      //   把"要发出去的那 60 行"每行的平均亮度压成 **1 个十六进制位**（0=最黑 F=最亮），
      //   60 行就是 60 个字符。用来判断"图上半截全黑"到底是：
      //     ① 摄像头那段本来就没数据 / ② DMA 没写到缓冲那一段 / ③ 行宽对不上导致错位
      {
        char     prof[CAM_H / 2 + 1];
        uint32_t r, cc, acc;

        for (r = 0; r < (uint32_t)(CAM_H / 2U); r++)
        {
          acc = 0;
          for (cc = 0; cc < (uint32_t)CAM_TXW; cc++)
          {
            acc += disp_img[r * 2U][cc];            // ★ 现在统计 disp_img（不再是 cam_image）
          }
          acc      /= (uint32_t)CAM_TXW;                  // 0..255
          prof[r]   = "0123456789ABCDEF"[acc >> 4];       // 压成 0~F
        }
        prof[CAM_H / 2U] = 0;

        sprintf(buf, "CAM3 R0=%s\r\n", prof);
        wifi_f4_send_string(buf);
      }

      // ---------- 第 3.5 行：★ 全画面"收割"健康度 ----------
      //   LG = 单次收割**最多补了几行**。一条行只活 120 行 ≈ 4.2ms，
      //        主循环周期 T ⇒ 每次要补 T/36.5µs 行：T=0.5ms→14；T=2ms→55；T=4.2ms→115（几乎全漏）
      //        ⇒ **LG ≥ 30 就说明主循环太慢，画面一定在漏行**（表现为横向撕裂/重复带）
      //   PS = 已经凑齐多少幅全画面（走完 480 行 +1）。**57fps 下应约 14/秒**
      //        ⇒ 500ms 心跳里 PS 应增加约 7。不涨 ⇒ 收割没在跑；涨太慢 ⇒ 漏行
      sprintf(buf, "CAM4 LG=%lu PS=%lu\r\n",
              (unsigned long)disp_lag_max, (unsigned long)disp_pass);
      wifi_f4_send_string(buf);

      // ---------- 第 4 行：★ 实测「**一帧多少字节**」—— 唯一确定分辨率档位 ★ ----------
      //   🔑 为什么要拆成**两行极短的行**：**长文本心跳会被图像数据冲散**
      //      （日志里 CAM1/CAM2 经常只剩半行），而 `PIN 6` / `P 12 34 56 78` 这类短行
      //      一直都能完整收到 ⇒ 所以这里只发 "BPF 数字" 和 "MD 数字" 两行。
      //   🔑 为什么量"帧字节数"而不是"行宽"：行数只能靠 LINE 标志，而主循环轮询**会漏掉大部分行**
      //      ⇒ 算出来的"行宽"偏大好几倍、反而误导。
      //      帧数则**一个都不漏**（帧率才几十 Hz；FRAME 标志的中断我们不使能，
      //      它就一直挂着等我们去读）⇒ `Δ字节 ÷ Δ帧数` = **一帧字节数 = 列×行** ✓
      //   🔑 一帧字节数直接就是档位（消隐期不产生数据）：
      //        约 360960 ⇒ **752×480**（MD=1）; 约 90240 ⇒ **376×240**（MD=2）; 约 22560 ⇒ **188×120**（MD=3）
      {
        static uint32_t pb = 0, pf = 0;
        uint32_t db  = cam_bytes     - pb;
        uint32_t df  = cam_frame_cnt - pf;
        uint32_t bpf = (df > 0U) ? (db / df) : 0U;
        uint32_t md  = 0U;

        pb = cam_bytes;
        pf = cam_frame_cnt;

        if      ((bpf > 288000U) && (bpf < 434000U)) { md = 1U; }   // 360960 ±20%
        else if ((bpf >  72000U) && (bpf < 108000U)) { md = 2U; }   //  90240 ±20%
        else if ((bpf >  18000U) && (bpf <  27000U)) { md = 3U; }   //  22560 ±20%

        sprintf(buf, "BPF %lu\r\n", (unsigned long)bpf);
        wifi_f4_send_string(buf);
        sprintf(buf, "MD %lu\r\n", (unsigned long)md);
        wifi_f4_send_string(buf);
      }

      // ---------- 第 5 行：★ 活引脚电平（**单独一行、很短**，不容易被截断成碎片）----------
      //   bit3 = PA6(PCLK)   bit2 = PB7(VSYNC)   bit1 = **PA4(HREF)**   bit0 = PC6(D0)
      //   ⚠️ 这是一瞬间的采样：PCLK 在高速跳变、D0 随图像变 ⇒ 那两位是随机的；
      //      **VSYNC 和 HREF 应该稳定是 1。**
      //   🔑 判据：**PA4 那一位（bit1）如果一直是 0 ⇒ HREF 真的没有信号**，
      //      那就是接线/焊点的问题，跟软件无关了。
      {
        uint8_t pin = (uint8_t)(((GPIOA->IDR & GPIO_PIN_6) ? 8U : 0U) |     // PCLK  PA6
                                ((GPIOB->IDR & GPIO_PIN_7) ? 4U : 0U) |     // VSYNC PB7
                                ((GPIOA->IDR & GPIO_PIN_4) ? 2U : 0U) |     // HREF  PA4 ★
                                ((GPIOC->IDR & GPIO_PIN_6) ? 1U : 0U));     // D0    PC6

        sprintf(buf, "PIN %X\r\n", (unsigned)pin);
        wifi_f4_send_string(buf);
      }

      // ---------- 第 6 行：★ 活引脚"占空比"（**实时！**，很短，容易被看到）----------
      //   顺序： PA6(PCLK)  PB7(VSYNC)  **PA4(HREF)**  PC6(D0)    单位 %
      //   🔑 与上面 FLT 的区别：**这个是实时的**（FLT 只在开机头 9 秒测，早就是旧数据了）。
      //   判读 HREF（第 3 个数）：
      //      ~80  ⇒ 在跳，**HREF 是活的** ✓
      //      0    ⇒ 恒低（没信号 / 被拉低）
      //      100  ⇒ 恒高（不跳，同样是坏的）
      {
        uint32_t n = (0U == live_n) ? 1U : live_n;

        sprintf(buf, "P %lu %lu %lu %lu\r\n",
                (unsigned long)(live_hi[0] * 100UL / n),
                (unsigned long)(live_hi[1] * 100UL / n),
                (unsigned long)(live_hi[2] * 100UL / n),
                (unsigned long)(live_hi[3] * 100UL / n));
        wifi_f4_send_string(buf);

        live_hi[0] = 0; live_hi[1] = 0; live_hi[2] = 0; live_hi[3] = 0;
        live_n = 0;
      }
      }   // ← 全量诊断（d >= 2）到此为止；d 0/1 时这一段整块跳过
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
