/*********************************************************************************************************************
* 文件名称          cmd_f4
* 适用平台          STM32F407
* 说明              上位机命令通道：TCP 下行的纯文本行 → 解析 → 立即生效 + 回 ACK
*                       可调参数定义也在这里（app.h 对外声明 extern）
*
* 修改记录
* 日期              作者                备注
* 2026-11-01        织命                从 main.c 拆分而来
********************************************************************************************************************/

#include "app.h"
#include "board_f4.h"
#include "wifi_f4.h"
#include "ctrl_f4.h"
#include <stdio.h>
#include <string.h>

/* ★ 图像模式下不能往流里塞 ACK 文本（会把图像帧冲散） ⇒ 统一从这里走 */
static void cmd_txt (const char *s)
{
    if ((0U == SEND_MODE_VOFA) && (NULL != s)) { wifi_f4_text(s, 2U); }
}

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

/* ★★★ 2026-11-02 最终值：**0（不限速）** ★★★
   沿革与结论（三次实测）：
     f=20 + 助手(22568B/帧) → 11.1fps
     f=0  + 助手(22568B/帧) → **0.9fps**（模块内部只有一块缓冲，被连续压榨后
                               丢弃/半包 ⇒ 助手反复重新同步 ⇒ TCP 窗口收缩，吞吐掉 10 倍）
     f=30 + cam_view.py(2894B/帧) → 仍然偏慢
   ⇒ **关键不是 f，而是"一帧几个块"**：
       助手 22568B = 6 个 4088B 块 ⇒ 块间等待模块排空，f 一旦调小就崩；
       cam_view.py 位打包 2894B = **1 个块**，发完即完，不存在"等下一块"的环节
       ⇒ 这时 f=0 是安全的，相 0 只等 5ms。
   ⇒ 一帧 = 打包 ~1ms + 发送 ~4.4ms + 歇 5ms ≈ 10ms，上限由传感器 50fps 决定。
   ⚠️ 只有**切回助手模式**（SEND_MODE_VOFA=1）时才需要把它调回 20~30，否则会重演 0.9fps。*/
/* ★★★ 2026-11-07（FW=2123）：**默认帧率 10 → 6** ★★★
   实测（图传默认打开后）25 秒里只收到 7 条 S + 5 条遥测（正常应是 50 + 125），
   S 行还都被截断 —— 说明 10fps × 2894B ≈ 29KB/s 把这个模块（实测吞吐 25~33KB/s）
   占满了，遥测/ACK 挤不出去 ⇒ 上位机点按钮没有回执。
   ⇒ 6fps ≈ 17KB/s，给文本留出 ~10KB/s 余量。想更流畅可以发 `f 8`（8fps），
     但按钮回执可能又开始丢。*/
uint16_t cam_cmd_fps    = 6U;
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

// ★ 采集链自愈/诊断用：最近一次量到的像素时钟(kHz) 与 DCMI->RISR 快照（心跳里报告）

// ★★ 自动探测摄像头接口的结果（**不需要用户敲任何命令**，直接看心跳）
//   0 = 还没找到   1 = SCCB 接法0 通   2 = SCCB 接法1 通   3 = 地址 0xBA 通
//   🔑 为什么要自动做：**调试命令也要走同一条链路**（链路坏的时候，命令根本进不来
//      ⇒ 只能靠用户反复敲命令来验证链路，既慢又费人）⇒ 链路一通，就该让板子自己干、
//      自己把结果写进心跳，用户只要看一行字 ✓
uint8_t  cam_if_ok    = 0U;
uint16_t cam_if_id    = 0U;
uint8_t  cam_if_tries = 0U;

// ★ 最近收到的原始字节（环形，`o` 命令 dump 出来看）—— 用来判"收到的是不是我们发的那几个字符"

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

// 带符号版本（车模命令要能写 -3000 这种负占空比）
static int32_t cam_cmd_snum (const char *s)
{
    int32_t sign = 1;

    while ((*s == ' ') || (*s == '\t')) { s ++; }
    if (*s == '-')      { sign = -1; s ++; }
    else if (*s == '+') { s ++; }
    return sign * (int32_t)cam_cmd_num(s);
}

// 跳到第 2 个数字的起点（"c 3 10" 里那个 10）
static const char * cam_cmd_arg2 (const char *s)
{
    while (*s && (*s != ' ')) { s ++; }
    while ((*s == ' ') || (*s == '\t')) { s ++; }
    return s;
}

// 跳到"下一个数字"的起点：跳过空格 + 当前这个数（带不带负号都行）
//   —— 给车模命令用，`w2000 2000` 和 `w 2000 2000` 两种写法都能认
static const char * cam_cmd_next (const char *s)
{
    while ((*s == ' ') || (*s == '\t')) { s ++; }
    if ((*s == '-') || (*s == '+'))     { s ++; }
    while ((*s >= '0') && (*s <= '9'))  { s ++; }
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

    /* ★ 2026-11-05：后面加上 FW / EDG / EL / ER / DRP / DST / SELF——
       FW  = 固件标签（烧的是不是这一版，一眼确认，不用翻心跳）
       EDG = 已经发出去几包"左右边界数组"（0xFD）；**在涨 ⇒ 新固件在发** ✓
       EL/ER = 最近一包里左/右边界各有几行有效（120 = 整幅都爬到了）✓
       DRP = 发送看门狗**主动放弃**的帧数（一直飙 ⇒ 链路/模块发不动）✓
       DST = DMA 停摆看门狗重启采集的次数（>0 ⇒ 采集链卡过，但已经自己救回来了）✓
       SELF= 连续 2 秒没有完整帧 ⇒ 重启采集的次数（和 DST 一起看）✓ */
    sprintf(b, "CFG f=%u g=%u x=%u y=%u (%ux%u) d=%u CST=%u RV=%06lX LINE=%u FW=2123 EDG=%lu EL=%u ER=%u DRP=%lu DST=%lu SELF=%lu\r\n",
            (unsigned)cam_cmd_fps, (unsigned)cam_cmd_gap_ms,
            (unsigned)cam_cmd_dx, (unsigned)cam_cmd_dy,
            (unsigned)(CAM_TXW / cam_cmd_dx), (unsigned)(CAM_H / cam_cmd_dy),
            (unsigned)cam_dbg_level,
            (unsigned)cam_cfg_state, (unsigned long)(cam_cfg_ver & 0xFFFFFFUL),
            (unsigned)cam_cfg_line_state(),
            (unsigned long)cam_edge_pkt_cnt,
            (unsigned)cam_edge_l_cnt, (unsigned)cam_edge_r_cnt,
            (unsigned long)cam_tx_drop,
            (unsigned long)cam_dma_stall,
            (unsigned long)cam_self_heal);
    cmd_txt(b);
}

// ★ ★ 位打包：1 = 二值图每行 188 像素压成 **24 字节**（1字节=8像素）⇒ 数据量÷8 ✓
//   上位机看包头 pkt[6] 的最高位（1 = 打包）自动解包；按 `z` 可切回未打包（排障用）✓
uint8_t cam_pack = 1U;

/* ★★ 2026-11-07（FW=2099）：**上位机保活计数** ★★
   上位机每 2 秒发一行 `!`。它只用来证明"下行还通"，不回任何包。
   app.c 用它当见证人：见过 `!` 之后，6 秒收不到就判定链路死（模块被复位/掉线），
   强制重连（连续失败会整模块硬复位）—— 修"已连接但 0 B/s、只能断电"。*/
uint8_t cam_ka_seen = 0U;

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
        /* ★ 保活：上位机每 2 秒发一行 `!`。**只记一笔，绝不回包**（回包会把控制台刷满）。
           它同时说明"PC → 模块 → 板子"这条路还通着。*/
        case '!':
            cam_ka_seen = 1U;
            return;

        case 'f':
            cam_cmd_fps = (uint16_t)cam_cmd_num(s + 1);
            if (cam_cmd_fps > 30U) { cam_cmd_fps = 30U; }
            break;

        case 'g': cam_cmd_gap_ms = (uint16_t)cam_cmd_clamp(cam_cmd_num(s + 1), 0U, 200U); break;
        // ⚠️ 188 模式下 `dx`/`dy` **必须**是 1：
        //    那时 DMA 缓冲天然就是一整帧（120 行 × 188 字节），抽点会直接算错取址 ✗
        //    ⇒ 在 188 模式下**忽略**这两个命令，并明确回执（免得"发了没反应"让人困惑）✓
        case 'x':
            if (1U == cam_mode) { cmd_txt("ACK x: ignored (MD=1 needs dx=1)\r\n"); return; }
            cam_cmd_dx = cam_cmd_clamp(cam_cmd_num(s + 1), 1U, 8U);
            break;
        case 'y':
            if (1U == cam_mode) { cmd_txt("ACK y: ignored (MD=1 needs dy=1)\r\n"); return; }
            cam_cmd_dy = cam_cmd_clamp(cam_cmd_num(s + 1), 1U, 8U);
            break;

        /* ⚠️ 恢复默认值要和"当前模式"配套：cam_view.py 模式下 0 是安全的（1 个块/帧）*/
        case 'a': cam_cmd_fps = 0U; cam_cmd_gap_ms = 0U; cam_cmd_dx = 1U; cam_cmd_dy = 1U; break;

        case 'd':                                   // ★ 心跳档位：0 静默 / 1 精简(默认) / 2 全量
            cam_dbg_level = cam_cmd_clamp(cam_cmd_num(s + 1), 0U, 2U);
            break;

        case 'b':
            v = cam_cmd_num(s + 1);
            cam_cfg_uart_open((uint8_t)((v > 4U) ? 0U : v));
            cam_cfg_state = 0U;
            cmd_txt("ACK b: uart5 opened\r\n");
            return;

        case 'v':
            v = cam_cfg_read_ver();
            sprintf(b, "ACK v: got %lu bytes RV=%06lX LINE=%u\r\n",
                    (unsigned long)v, (unsigned long)(cam_cfg_ver & 0xFFFFFFUL),
                    (unsigned)cam_cfg_line_state());
            cmd_txt(b);
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
            cmd_txt(b);
            return;

        // ★ 采集模式：`m 0` = 752 全分辨率（抽行）；`m 1` = 188×120（缓冲即整帧）
        case 'm':
            v = cam_cmd_num(s + 1);
            board_cam_set_mode((uint8_t)((v != 0U) ? 1U : 0U));
            sprintf(b, "ACK m: mode=%u rowbytes=%u span=%u\r\n",
                    (unsigned)cam_mode, (unsigned)cam_row_bytes, (unsigned)cam_span);
            cmd_txt(b);
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
            cmd_txt("ACK j0: got cmd\r\n");

            if (0U == has_arg)
            {
                r = cam_sccb_test();
                cmd_txt("ACK j1: sccb done\r\n");     // SCCB 没卡住 ⇒ 走到这
                cam_pin_tristate_all();
                cmd_txt("ACK j3: tristate done\r\n"); // 三态检测没卡住 ⇒ 走到这
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
            cmd_txt(b);

            //   三态判读：PU=1 & PD=0 ⇒ **空脚**（线没通/脚没引出）；PU=1 & PD=1 ⇒ 确实接着东西 ✓
            sprintf(b, "ACK jw: PC12 PU=%u PD=%u | PD2 PU=%u PD=%u   (PU=1&PD=0 = floating)\r\n",
                    (unsigned)cam_pin_c12_pu, (unsigned)cam_pin_c12_pd,
                    (unsigned)cam_pin_d2_pu, (unsigned)cam_pin_d2_pd);
            cmd_txt(b);
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
            cmd_txt(b);
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
            cmd_txt(b);
            return;

        // ★ 曝光：`e <行数>` 直接写 R0x0B（Total Shutter Width，单位=行），下一帧生效 ✓
        //   行 bin4 后帧只有 120 行 ⇒ 行数必须 < 120（出厂 443 会卡死不成帧 ✓）
        case 'e':
            v = cam_cmd_num(s + 1);
            cam_shutter = (uint8_t)cam_cmd_clamp(v, 10U, 110U);
            (void)cam_sccb_write_reg(0x0BU, cam_shutter);
            sprintf(b, "ACK e: shutter=%u rows\r\n", (unsigned)cam_shutter);
            cmd_txt(b);
            return;

        // ★ 翻转/镜像：`n` 每按一次换一种（0 不翻 → 1 左右镜像 → 2 上下 → 3 180° → 0…）
        //   传感器硬件翻转（R0x0D bit5/bit4），下一帧即生效 ✓ —— 装在车上方向不对就用它 ✓
        case 'n':
            cam_flip = (uint8_t)((cam_flip + 1U) & 3U);
            cam_sccb_apply_flip();
            sprintf(b, "ACK n: flip=%u (0=none 1=mirror 2=flip-vert 3=180)\r\n", (unsigned)cam_flip);
            cmd_txt(b);
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
            cmd_txt(b);
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
                cmd_txt(b);

                sprintf(b, "SCAN RX: %lu %lu %lu %lu %lu %lu %lu\r\n",
                        (unsigned long)cam_scan_rx[0], (unsigned long)cam_scan_rx[1],
                        (unsigned long)cam_scan_rx[2], (unsigned long)cam_scan_rx[3],
                        (unsigned long)cam_scan_rx[4], (unsigned long)cam_scan_rx[5],
                        (unsigned long)cam_scan_rx[6]);
                cmd_txt(b);

                sprintf(b, "SCAN EG: %lu %lu %lu %lu %lu %lu %lu   (顺序 9600/115200/57600/38400/19200/4800/230400)\r\n",
                        (unsigned long)cam_scan_eg[0], (unsigned long)cam_scan_eg[1],
                        (unsigned long)cam_scan_eg[2], (unsigned long)cam_scan_eg[3],
                        (unsigned long)cam_scan_eg[4], (unsigned long)cam_scan_eg[5],
                        (unsigned long)cam_scan_eg[6]);
                cmd_txt(b);
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
            cmd_txt(b);
            return;
        }

        case 'c':
            v  = cam_cmd_num(s + 1);                    // cmd
            v2 = cam_cmd_num(cam_cmd_arg2(s + 1));      // val
            cam_cfg_send_pkt((uint8_t)v, (uint16_t)v2);
            sprintf(b, "ACK c: sent A5 %02lX %02lX %02lX\r\n",
                    (unsigned long)(v & 0xFFUL), (unsigned long)((v2 >> 8) & 0xFFUL),
                    (unsigned long)(v2 & 0xFFUL));
            cmd_txt(b);
            return;

        case 'i':
            cam_cfg_send_pkt(0x00, 0x0000);
            cmd_txt("ACK i: INIT sent\r\n");
            return;

        case 'p':
            sprintf(b, "ACK p: LINE=%u (bit1=PC12/TX bit0=PD2/RX)\r\n",
                    (unsigned)cam_cfg_line_state());
            cmd_txt(b);
            return;

        /* ==================== ★★ 车模控制（2026-11-05 从 MSPM0 工程移植）★★ ====================
           q <rpm>          速度环目标（0 = 停；速度环驱动电机 1）
           w <d1> <d2>      开环直接给两个电机占空比（-10000..+10000，0 0 = 停）
           r <pos>          舵机位置 0..4095（**中位 2748；安全限位 1700..3024**，超出会被夹住）
           l                编码器累计清零
           h                一行汇总：编码器 / 转速 / 速度环 / 姿态 / 舵机 / IMU
        =============================================================================== */
        case 'q':
            {
                int32_t r = cam_cmd_snum(s + 1);
                if (r >  6000) { r =  6000; }
                if (r < -6000) { r = -6000; }
                ctrl_open_loop(0U);             /* ★ 回闭环：速度环重新接管 */
                if (0 != r) { ctrl_motor_clear_stall(); }   /* ★ 非零目标 = 明确要重启电机 */
                ctrl_speed_set_target((int16_t)r);
                sprintf(b, "ACK q: target=%d state=%s\r\n",
                        (int)ctrl_speed_get_target(), ctrl_speed_state_name());
                cmd_txt(b);
            }
            return;

        case 'w':
            {
                int32_t d1 = cam_cmd_snum(s + 1);
                int32_t d2 = cam_cmd_snum(cam_cmd_next(s + 1));

                ctrl_open_loop(1U);             /* ★ 进开环：速度环让位，只认这里的占空比 */
                ctrl_motor_set(1U, (int16_t)d1);
                ctrl_motor_set(2U, (int16_t)d2);
                sprintf(b, "ACK w: d1=%d d2=%d (open-loop)\r\n", (int)d1, (int)d2);
                cmd_txt(b);
            }
            return;

        case 'r':
            {
                int32_t p = cam_cmd_snum(s + 1);

                if (-1 == p)                        /* r -1 = 切单线半双工 */
                {
                    ctrl_servo_set_half(1U);
                    cmd_txt("ACK r: duplex=half(1wire on PC10)\r\n");
                    return;
                }
                if (-2 == p)                        /* r -2 = 切回全双工 */
                {
                    ctrl_servo_set_half(0U);
                    cmd_txt("ACK r: duplex=full(PC10=TX PC11=RX)\r\n");
                    return;
                }
                if (p < 0)      { p = 0; }
                if (p > 4095)   { p = 4095; }
                (void)ctrl_servo_set_pos((uint16_t)p);
                sprintf(b, "ACK r: servo -> %u (now %u)\r\n",
                        (unsigned)p, (unsigned)ctrl_servo_get_pos());
                cmd_txt(b);
            }
            return;

        case 'l':
            ctrl_encoder_clear();
            cmd_txt("ACK l: encoder cleared\r\n");
            return;

        case 'h':
            {
                int  yw = (int)ctrl_yaw_deg();
                int  yf = (int)((ctrl_yaw_deg() - (float)yw) * 10.0f);
                if (yf < 0) { yf = -yf; }
                sprintf(b, "ACK h: enc=%ld d=%ld rpm=%d yaw=%d.%d rd=%u st=%s duty=%d srv=%u e=%u who=%02X\r\n",
                        (long)ctrl_encoder_total(), (long)ctrl_encoder_delta(),
                        (int)ctrl_speed_get_rpm(),
                        yw, yf,
                        (unsigned)ctrl_yaw_ready(),
                        ctrl_speed_state_name(), (int)ctrl_speed_get_duty(),
                        (unsigned)ctrl_servo_get_pos(), (unsigned)ctrl_servo_get_err(),
                        (unsigned)ctrl_imu_whoami());
                cmd_txt(b);
            }
            return;

        /* s            = 回状态（落到最后）
           s 0 / s 1    = 上位机图像模式：0 = 原图（8bit 灰度缩略图 94×60）/ 1 = 二值图（默认）*/
        case 's':
            {
                const char *a = s + 1;

                while (*a == ' ') { a ++; }
                if (0 == *a) { break; }                 /* 单单一个 s ⇒ 回状态 */

                v = cam_cmd_num(a);

                /* ★ 2026-11-06：s 2 = **停图传**（链路全留给文本，调遥测用）
                               s 3 = 恢复图传 */
                if (2U == v)
                {
                    cam_img_off = 1U;
                    cmd_txt("ACK s: image OFF (telemetry only)\r\n");
                    return;
                }
                if (3U == v)
                {
                    cam_img_off = 0U;
                    cmd_txt("ACK s: image ON\r\n");
                    return;
                }

                cam_view_mode = (uint8_t)((0U == v) ? 0U : 1U);
                sprintf(b, "ACK s: view=%u (%s)\r\n", (unsigned)cam_view_mode,
                        (0U == cam_view_mode) ? "GRAY 94x60" : "BIN 188x120");
                cmd_txt(b);
            }
            return;

        case '?':
            cmd_txt("CMD f<n> g<ms> x<n> y<n> a s b<n> v c<cmd> <val> i p ?\r\n");
            cmd_txt("CAR q<rpm> speed-loop  w<d1> <d2> open-loop  r<pos> servo  l clr-enc  h status\r\n");
            cmd_txt("IMG s 0 = gray 94x60   s 1 = binary 188x120\r\n");
            return;

        default:
            break;
    }

    cam_cmd_status();       // 其它任何输入（含单独的 's'）都回一条状态，方便确认"它收到了"
}

// 主循环每轮调一次：把下行字节攒成行，逐行解析
void cam_cmd_poll (void)
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
        /* ★★★ 2026-11-07（FW=2123）：**只收可打印字符，别把模块的 0x00 填充塞进命令行** ★★★
           现场：上位机点按钮一直重发，板子却回的是 `CFG`（default 分支），不是 `ACK r`。
           真凶：模块把下行数据放在 32 字节块里，**前面补了一堆 0x00**；
           这些 0 被当成命令字符塞进 `cam_cmd_buf`，于是 `cam_cmd_line()` 看到的第一个字节是
           0x00 ⇒ `switch(0)` 落到 default ⇒ 只会回状态，`r`/`h`/`!` 这些命令**永远匹配不上**。
           （上位机那边早就做了同样的 NUL 过滤，板子这边漏了。）
           ⇒ 现在只接受 0x20~0x7E，其余（0x00 填充、噪声）全部丢掉。*/
        else if ((c >= 0x20) && (c < 0x7F) &&
                 (cam_cmd_len < (uint8_t)(sizeof(cam_cmd_buf) - 1U)))
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

// ★★ 两个"能一次判定画面为什么还不对"的诊断量（心跳的 S 行里显示）★★
//   TO = 相 1 **超时**（而不是"采齐了"）的次数 —— 非 0 ⇒ 这一幅是**没采齐就发出去的**（必花）
//   F  = 冻结那一刻 **实际填了多少行**（正常应 = 120 即 CAM_H）—— 小于 120 ⇒ 缺行
uint32_t cam_to_cnt  = 0U;
uint8_t  cam_frz_fill = 0U;

// 每个主循环轮调用一次。返回 1 = 这一轮该发一行（行号见 cam_tx_row）
