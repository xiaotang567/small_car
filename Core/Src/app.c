/*********************************************************************************************************************
* 文件名称          app
* 适用平台          STM32F407
* 说明              主循环 + 采集/发送相位机 + VOFA 帧发送
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


/* ★★★ 图像模式下的文本闸门 ★★★
   逐飞助手/上位机是按"固定字节数"切帧的（协议里没有长度字段、没有校验和）。
   只要往同一条 TCP 流里插进去几十字节 ASCII，后面所有帧就整体错位 ⇒ 画面撕裂 → 丢同步。
   ⇒ SEND_MODE_VOFA=1（发图像）时，**所有文本输出一律丢弃**；只有发自定义协议时才放行。 */
static void dbg_txt (const char *s)
{
    /* ★★ 2026-11-06：这里**必须是低优先级(0)** ★★
       之前写成 1（高），结果 500ms 一条的 `S` 心跳（448 字节）长期占着发送队列，
       而遥测 `T` 是低优先级 ⇒ 被那条判断直接丢掉 ⇒ **遥测一条都发不出去**。
       （`CAM FOUND` 能收到，正是因为它也是高优先级，能顶掉心跳。）
       心跳/诊断都是"有就用、丢了无所谓"的信息，不该挡住遥测。*/
    if ((0U == SEND_MODE_VOFA) && (NULL != s)) { wifi_f4_text(s, 0U); }
}
volatile uint32_t cam_rx_bytes = 0U;
static uint32_t t_last_frame = 0U;    // ★ 上次自检时的帧计数（判"有没有新帧"）      // ★ 最近一次量到的 PB7(VSYNC) 边沿率(Hz)
volatile uint8_t  cam_rx_dump[32]  = {0};
volatile uint32_t cam_rx_dump_i    = 0U;
static uint32_t cam_t_tx      = 0;
static uint16_t cam_tx_row    = 0U;      // 当前正在发的行（发送块用它填 pkt[4] / 取像素）

static uint32_t t_pclk   = 0U;
static uint32_t dcmi_ris = 0U;
static uint32_t t_vsync_hz = 0U;
static uint8_t  cam_phase     = 0U;      // 0=歇   1=采   2=发
static uint8_t  g_txt_win_active = 0U;   // 1 = 文本专用窗口正在开（图像让路）
static uint32_t cam_t_phase   = 0;
static uint32_t cam_t_p2      = 0;       // ★ 进相 2（开始发这一帧）的时刻：给发送看门狗用
uint32_t        cam_tx_drop   = 0U;      // ★ 因为发不出去而**主动放弃**的帧数（CFG 里的 DRP）
uint32_t        cam_self_heal = 0U;      // ★ 连续 2 秒没有完整帧 ⇒ 重启采集的次数（CFG 里的 SELF）

/* ★★★ 2026-11-07（FW=2100）：**主循环看门狗**（在 SysTick 里判，见 stm32f4xx_it.c）★★★
   现场：PC10/PC11 并上以后 OLED 直接定格 ⇒ 主循环停了（模块还揣着 TCP）
   ⇒ 上位机永远"已连接 + 0 B/s"，只能断电。这里让板子自己发现、自己重启。*/
volatile uint32_t g_loop_tick  = 0U;     /* 主循环每圈踩一下 */
volatile uint8_t  g_wdt_on     = 0U;     /* 1 = 开始看门（app_loop 启动后才置，避开开机初始化）*/
volatile uint8_t  g_wdt_pause  = 0U;     /* 1 = 暂停（只包住必然长时间阻塞的 wifi_f4_init）*/
volatile uint8_t  g_loop_mark  = 0U;     /* 这一圈走到第几步（卡死时由看门狗写进 RTC，开机显示 LM=）*/

/* ★★★ 2026-11-06：上位机"原图/二值图"开关 ★★★
   0 = 发 **8bit 灰度缩略图**（94×60，`0xFC` 包，8 块）
   1 = 发 **位打包二值整帧**（188×120，`0xFF` 包）—— 默认，就是原来的行为
   命令：`s 0` / `s 1` */
uint8_t         cam_view_mode = 1U;

static void cam_send_edges (void);
static void cam_send_gray (void);


/* ★ 2026-11-06：**停图传**开关（`s 2` 停 / `s 3` 恢复）
   用途：调遥测/命令时让链路完全空闲，把"图传占满链路"这个变量彻底排除掉。
   ★ 2026-11-07（FW=2131）：**默认改回"发图"**（用户要求把图传弄回来）。
     要临时停：上位机点【停图传】（`s 2`）或者发 `s 2`。*/
uint8_t cam_img_off = 0U;

static uint8_t cam_tx_tick (uint32_t now)
{
    if (0U != cam_img_off) { return 0U; }        /* 停图传：一帧都不发，链路全留给文本 */

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
        static uint32_t t_done = 0U;            // 上一次看到"完整帧"的时刻
        if (disp_done) { ready = 1U; t_done = now; }
        /* ★★★ 2026-11-05 晚2：**2 秒一张完整帧都没有 ⇒ 采集链自愈一次** ★★★
           用户实测的"上位机收不到图片"：心跳 `f` 还在涨（软件看得见帧首），
           但 `disp_done` 永远不置位 ⇒ 一张图都发不出去。
           这种"半死"状态以前只能复位板子；现在每 2 秒重开一次采集
           （`board_cam_restart()` 内部先 force_stop 再 start，**不死等**）✓
           正常时 50fps ⇒ 2 秒有 100 帧，绝不会误触发 ✓ */
        else if ((0U != cam_if_ok) && ((now - t_done) > 2000UL))
        {
            t_done = now;
            cam_self_heal ++;
            (void)board_cam_restart();
        }

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
            static char cb[160];   /* ★ static：不吃栈 */

            // ★★★ 2026-10-20 关键修正：自愈重启只在"真没帧"时才做！★★★
            //   实测（CAM1）：同步成功之后，**每秒一次的无条件重启把采集压到 ~1.2fps**
            //   （f 每秒 +1 = "每次重启后只采到 1 帧就被我打断"的铁证 ✓）
            //   ⇒ 只有"连续 1 秒一张新帧都没有"才重启；正常时绝不碰 DCMI ✓
            if (cam_frame_cnt == t_last_frame)
            {
                cam_to_cnt ++;
                /* ⚠️⚠️ 不再调用 `board_cam_restart()`！
                   它内部的 `HAL_DCMI_Stop()` **没有超时**，DCMI 状态不干净时会**死等**
                   ⇒ 板子整个挂住、连心跳都没了（实测 2026-10-19 22:04 那次就是：发了 4 行后 13 分钟无声）✗✗
                   现在只"重开一趟"（不碰硬件）⇒ 即使采集真死了，板子也不会挂，心跳照发 ✓ */
                board_cam_new_pass();           // 计数/位图清零，重新开始数
                /* ★★★ 2026-11-05：顺手把"冻结"也解掉 ★★★
                   如果上一帧在相 2 卡住（发不出去），`disp_hold` 会一直是 1
                   ⇒ 收割（board_cam_harvest）每一轮都直接返回 ⇒ disp_done 永远不置位
                   ⇒ 就算链路恢复了，也永远等不到"采齐"⇒ 画面永久定格 ✗
                   这里无条件解冻：最多是多采一帧，绝不会有坏处 ✓ */
                disp_hold = 0U;
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
            dbg_txt(cb);

            cam_t_phase = now;
        }

        if (ready)
        {
            /* ★★★ 2026-11-06：**原图（灰度）模式** ★★★
               用户在上位机上点"原图"时（`s 0`），这一帧改成发 8bit 灰度缩略图，
               不再发位打包的二值整帧。
               ⚠️ 必须**先发灰度、再** `board_cam_analyze()`：analyze 会把 disp_img **原地二值化**，
                  发完就再也拿不到灰度了。
               ⚠️ 这一路**不进相 2**：灰度包在相 1 里一次发完（约 30~50ms，此时 disp_hold=1，
                  收割是冻住的，不会和采集打架），发完直接回相 0。 */
            if (0U == cam_view_mode)
            {
                cam_send_gray();
                board_cam_analyze();             // 仍然算边界（原图上也要画红线）
                cam_send_edges();
                if (by_timeout) { cam_to_cnt ++; }
                cam_frz_fill = disp_fill;
                disp_hold   = 0U;
                cam_phase   = 0U;
                cam_t_phase = now;
                return 0U;
            }

            board_cam_analyze();                 // ★★ 全部在单片机内：Otsu 二值化 + 每行亮/暗交界列 ★★
            if (by_timeout) { cam_to_cnt ++; }   // ★ 没采齐就发 ⇒ 这一幅必有缺陷（心跳里看 TO）
            cam_frz_fill = disp_fill;            // ★ 冻结那一刻实际填了多少行（正常 = 120）

            disp_hold  = 1U;                     // （正常情况下收割早已自己冻好了，这里是双保险）
            cam_tx_row = 0U;
            cam_t_tx   = now;
            cam_t_p2   = now;                    // ★ 相 2 的看门狗计时起点（见相 2）
            cam_phase  = 2U;
        }
        return 0U;
    }

    // ---------- 相 2：专心发 ----------
    /* ★★★ 2026-11-05 晚：**发送看门狗**（"运行一段时间卡住"的直接解药）★★★
       正常一帧：整帧包 2894B ≈ 4~10ms（逐行包 = 行数 × gap）。
       如果 `wifi_f4_send()` 一直返回"发不出去"（模块卡住 / TCP 窗口满了 / 上位机不收了），
       旧代码会**永远重试这一帧**，而 `disp_hold` 一直是 1 ⇒ 收割永久停住
       ⇒ 上位机画面**定格**，只能复位板子 ✗✗
       ⇒ 现在超过预算就直接**放弃这一帧**：解冻收割、回相 0，下一帧照常开始 ✓
          （丢一帧没人看得出来，但整条链路不会死）
       预算 = 300ms + 逐行模式的 2 倍理论发送时间（`g 6` 时 120 行 ≈ 720ms）。*/
    if ((now - cam_t_p2) > (300UL + (uint32_t)((uint16_t)CAM_H / (uint16_t)cam_cmd_dy)
                                    * (uint32_t)cam_cmd_gap_ms * 2UL))
    {
        cam_tx_row  = 0U;
        disp_hold   = 0U;                       // 解冻：让收割重新跑起来
        cam_phase   = 0U;
        cam_t_phase = now;
        cam_tx_drop ++;
        return 0U;
    }

    if ((now - cam_t_tx) >= (uint32_t)cam_cmd_gap_ms)
    {
        cam_t_tx = now;
        return 1U;                              // ← 让主循环把这一行发出去
    }
    return 0U;
}

/* ★★★ 上位机模式：1 = VOFA+ JustFloat（发波形，不发图像）  0 = 自定义上位机（发图像）★★★
   VOFA+ 设置：协议 JustFloat，通道数 8
   ch0=行110边界 ch1=行92边界 ch2=行67边界 ch3=偏航dev
   ch4=cam_now  ch5=cam_ahead  ch6=FPS  ch7=阈值T */

/* 把 buf 全部发出去（wifi_f4_send 发不完会把剩余长度返回，必须重试，
   否则一帧少一截 ⇒ 助手永远拼不出完整帧）
   ★★★ 2026-11-02 改：**返回"实际发出去多少字节"** ★★★
   原来返回 void，链路过载时 `stall > 12` 直接 `return` 把剩下的字节**扔掉**
   ⇒ 上位机是按固定 22568 字节切帧的（助手协议里**没有长度、没有校验**）
   ⇒ 少一截就**永久错位**，画面从此斜切/错行，只有重连才恢复。
   这正是"时不时畸变"的来源：只要某一帧赶上模块忙，之后就一直歪。
   ⇒ 现在把"发了多少"告诉调用方，由 `vofa_send_frame()` 用**补零**把长度补齐
     （这一帧后半截是黑的，但**字节流长度始终对齐** ⇒ 下一帧立刻恢复正常 ✓）*/
static uint32_t zf_send_all (const uint8_t *buf, uint32_t len)
{
    uint32_t total = len;
    uint32_t rem;
    uint16_t guard = 0U;
    uint16_t stall = 0U;

    while ((0U < len) && (guard < 60U))
    {
        guard ++;
        rem = wifi_f4_send(buf, len);
        if (0U == rem)                     { return total; }    /* 发完了 */
        if (rem >= len)                                         /* 一点没动 */
        {
            /* ⚠️ 阈值从 12 降到 **3**：现在每次 `wifi_f4_send()` 最坏会等 33ms
               （按块大小给的预算），12 次就是 **400ms 阻塞主循环** —— 太狠了。
               3 次 ≈ 100ms 已经足够判定"链路真的堵死"，再多等也没用。 */
            if (++stall > 3U)              { break; }           /* 链路卡死 ⇒ 交给调用方补零 */
        }
        else
        {
            stall = 0U;
            buf  += (len - rem);
            len   = rem;
        }
    }
    return (uint32_t)(total - len);        /* 已发出的字节数 */
}

/* ★ 补零到指定总长：只为了"把字节流长度凑齐"，内容是什么无所谓。
   带时间预算（最坏 ≈46ms），绝不允许它把主循环按死。
   ⚠️ 用封顶块 256B 而不是按剩余长度一次发：模块对大块的等待预算更贵，
      分小块反而更快停手（真堵死时差距很明显）。 */
static void zf_pad_to (uint32_t sent, uint32_t total)
{
    static const uint8_t z[256] = {0};
    uint8_t  guard = 0U;
    uint32_t n     = (total > sent) ? (total - sent) : 0U;

    while ((0U < n) && (guard < 20U))
    {
        uint32_t k = (n > (uint32_t)sizeof(z)) ? (uint32_t)sizeof(z) : n;

        guard ++;
        if (0U != wifi_f4_send(z, k)) { break; }   /* 连补零都发不动 ⇒ 放弃，别死等 */
        n -= k;
    }
}

/* ★★★ 发送抽点系数（2026-11-02）★★★
   为什么要抽：助手上实测 FPS 6.9、字节/秒 24528 ⇒ 这条链路真实吞吐只有
   **约 204KB/s**（模块每消化 4088B 要 ~20ms）。而一帧 188×120 原始灰度 = 22568B
   ⇒ 22568 ÷ 4088 = **6 个块** ⇒ 6 × 26ms ≈ 157ms ⇒ **6.4fps**（和实测 6.9 吻合）。
   ⇒ **188×120 在这条链路上物理上到不了 16fps**，无论怎么调间隔。
   抽 2×2 后：94×60 = 5640B = **2 个块** ⇒ 约 28~52ms/帧 ⇒ **19~35fps** ✓
   ✅ 抽点**不裁视野**：还是整个画面、宽高比也一样（94/60 = 188/120），只是采样变粗；
   ✅ 助手按包头里的宽高自适应显示，**上位机不用改任何设置**；
   ⚠️ 想回到全分辨率：把下面这个 2 改成 1 即可（代价：帧率掉回 6~7fps）。*/
#define VOFA_DECIM      2U
#define VOFA_TXW        ((uint16_t)(CAM_TXW / VOFA_DECIM))   /* 94  */
#define VOFA_TXH        ((uint16_t)(CAM_H   / VOFA_DECIM))   /* 60  */

static void vofa_send_frame (void)
{
    /* ★ 逐飞助手（新版协议，说明书 V1.4 / 库 seekfree_assistant_camera_data_send）
       8 字节包头 + 原始灰度，**没有校验和**：

         [0] 0xAA                          帧头
         [1] 0x02                          功能码 CAMERA
         [2] (type<<5) | (无图像<<4) | 边界数
                type=2 (MT9V03X/GRAY) ⇒ 2<<5 = 0x40
                有图像 ⇒ 第4位 = 0
                不带边线 ⇒ 低4位 = 0     ⇒ 0x40
         [3] 0x08                          本协议段长度 = sizeof(struct) = 8
         [4..5] 宽  小端   [6..7] 高 小端
         ↓
         VOFA_TXW x VOFA_TXH = 94 x 60 = 5640 字节灰度

       ⚠️ 旧版（12 字节 + check_sum）已废弃 —— 那是 2022 年的老库，助手不认
    */
    /* ★ 整帧一个缓冲：包头 + 图像**一次发出**
       之前分成两次 wifi_f4_send（8 字节 + 22560 字节），模块会拆成两个"包"，
       助手的解析如果按包边界走就会把起点算错 ⇒ 画面整体右移。
       放 CCM（34KB 空闲），只被 CPU 读写。*/
    static uint8_t tx_frame[8U + (uint32_t)VOFA_TXW * (uint32_t)VOFA_TXH]
        __attribute__((section(".ccmram")));
    uint8_t hdr[8];
    uint16_t r, c;

    hdr[0] = 0xAAU;
    hdr[1] = 0x02U;
    hdr[2] = 0x40U;
    hdr[3] = 0x08U;
    hdr[4] = (uint8_t)(VOFA_TXW & 0xFFU);
    hdr[5] = (uint8_t)((VOFA_TXW >> 8) & 0xFFU);
    hdr[6] = (uint8_t)(VOFA_TXH & 0xFFU);
    hdr[7] = (uint8_t)((VOFA_TXH >> 8) & 0xFFU);

    memcpy(tx_frame, hdr, 8U);

    /* ★ 2×2 抽点（横竖各隔一个取一个）—— 抽的同时完成，不需要中间缓冲 */
    for (r = 0U; r < VOFA_TXH; r++)
    {
        const uint8_t *src = &disp_img[(uint32_t)r * VOFA_DECIM][0];
        uint8_t       *dst = &tx_frame[8U + (uint32_t)r * VOFA_TXW];

        for (c = 0U; c < VOFA_TXW; c++)
        {
            dst[c] = src[(uint32_t)c * VOFA_DECIM];
        }
    }

    /* ★★★ 2026-11-02：发不完就**补零凑齐长度**，绝不让字节流错位 ★★★
       上位机按固定 8 + W×H 字节切帧，协议里没有长度也没有校验
       ⇒ 这一次少发 N 字节，从这一帧起**全部歪掉**（斜切/错行/半屏灰），
         只有重连才会恢复 —— 这就是"时不时错位"的真正来源。
       补零后：这一帧后半截是黑的（一眼能看出丢了一帧），但字节数仍然对齐
       ⇒ **下一帧立刻恢复正常**，不需要任何人去重启。 */
    {
        uint32_t total = 8U + (uint32_t)VOFA_TXW * (uint32_t)VOFA_TXH;
        uint32_t sent  = zf_send_all(tx_frame, total);

        if (sent < total) { zf_pad_to(sent, total); }
    }
}

/* ★★★ 左右边界数组包（0xFD）—— 把八邻域巡线的结果**原样**发给上位机 ★★★
   包格式（和整帧包同一个魔数，上位机按长度切 ⇒ 丢包只会丢这一包，不会错位）：
     magic(4) | 0xFD | 行数 | 保留 | CAM_H | 长度高 | 长度低 | 载荷 | 校验高 低
     载荷 = 每行 2 字节：[左边界x][右边界x]     0xFF = 这一行没找到
   ⇒ 上位机把 120 对坐标连成**两条红线**（左/右各一条），自己不再扫边界 ✓
   ⇒ 板子这边不做任何转向判断（用户要求先不做），只把点发出去 ✓

   代价：240 字节 ≈ 链路一帧里一个块的 6%，20fps 下多约 5KB/s，可以忽略。
   时间：这一包**紧跟整帧包**发，发不出去就丢掉（下一帧还有一包），绝不重试/死等。
   ⚠️ 必须在 `cam_tx_row_done()` 里"清 disp_hold **之前**"发：那之后收割会立刻
      重新填 disp_img、下一帧还会重算边界 ⇒ 先把 120 对坐标拷进 epkt 再发。 */
static uint8_t  cam_edge_pkt[12U + 2U * 120U + 2U];
uint32_t        cam_edge_pkt_cnt = 0U;      /* 成功发出去几包（上位机 `s` 里能看） */
uint8_t         cam_edge_l_cnt   = 0U;      /* 这一包里有几行左边界有效 */
uint8_t         cam_edge_r_cnt   = 0U;      /* 这一包里有几行右边界有效 */

static void cam_send_edges (void)
{
    uint16_t len = 0U, r, ck = 0U;

    cam_edge_pkt[0] = 0xAA; cam_edge_pkt[1] = 0x55;
    cam_edge_pkt[2] = 0xA5; cam_edge_pkt[3] = 0x5A;
    cam_edge_pkt[4] = 0xFD;                       /* 包类型：边界数组 */
    cam_edge_pkt[5] = (uint8_t)CAM_H;             /* 行数 */
    cam_edge_pkt[6] = 0x00;                       /* 保留 */
    cam_edge_pkt[7] = (uint8_t)CAM_H;

    cam_edge_l_cnt = 0U;
    cam_edge_r_cnt = 0U;

    for (r = 0U; r < (uint16_t)CAM_H; r++)
    {
        uint8_t l  = cam_bound[r];
        uint8_t rr = cam_bound_r[r];

        if (l >= (uint8_t)CAM_TXW)  { l  = 0xFFU; } else { cam_edge_l_cnt ++; }
        if (rr >= (uint8_t)CAM_TXW) { rr = 0xFFU; } else { cam_edge_r_cnt ++; }

        cam_edge_pkt[12U + len] = l;  len ++;
        cam_edge_pkt[12U + len] = rr; len ++;
    }

    cam_edge_pkt[8] = (uint8_t)(len >> 8);
    cam_edge_pkt[9] = (uint8_t)len;

    for (r = 0U; r < len; r++) { ck = (uint16_t)(ck + cam_edge_pkt[12U + r]); }
    cam_edge_pkt[12U + len]       = (uint8_t)(ck >> 8);
    cam_edge_pkt[12U + len + 1U]  = (uint8_t)ck;

    if (0U == wifi_f4_send(cam_edge_pkt, (uint32_t)(12U + len + 2U)))
    {
        cam_edge_pkt_cnt ++;
    }
}

/* ★★★ 2026-11-06：**灰度缩略图包（0xFC）**—— 上位机"原图"模式用 ★★★
   数据 = disp_img（188×120 灰度）做 **2×2 抽取** ⇒ 94×60 的 8bit 缩略图。
   ⚠️ 必须在 `board_cam_analyze()` **之前**调：分析会把 disp_img 原地二值化。

   为什么分 8 块发：整张 94×60 = 5640 字节，**超过模块一次 SPI 事务的上限 4088**
   ⇒ 一次发不完会被拆成两块、失败时就是"半个包"⇒ 字节流错位。
   分块后每块 94×8 = 752 字节 + 包头 14 = 766 字节，**每块都是一个完整事务** ✓

   包格式（沿用同一个魔数，上位机按长度切）：
     magic(4) | 0xFC | 块号 | 保留 | 本块行数 | 长度高 | 长度低 | 2 预留 | 数据 | 校验高 低
   一共 8 块，第 8 块只有 4 行（60 = 7×8 + 4）。 */
#define GRAY_W      94U
#define GRAY_H      60U
#define GRAY_ROWS   8U

static void cam_send_gray (void)
{
    static uint8_t gp[12U + GRAY_W * GRAY_ROWS + 2U];
    uint16_t blk;

    for (blk = 0U; blk < 8U; blk++)
    {
        uint16_t r0 = (uint16_t)(blk * GRAY_ROWS);
        uint16_t rows, ri, ci, len = 0U, ck = 0U;

        if (r0 >= GRAY_H) { break; }
        rows = (uint16_t)(GRAY_H - r0);
        if (rows > GRAY_ROWS) { rows = GRAY_ROWS; }

        gp[0] = 0xAA; gp[1] = 0x55; gp[2] = 0xA5; gp[3] = 0x5A;
        gp[4] = 0xFC;                       /* 包类型：灰度缩略图 */
        gp[5] = (uint8_t)blk;               /* 块号 0..7 */
        gp[6] = 0U;
        gp[7] = (uint8_t)rows;              /* 本块几行 */

        for (ri = 0U; ri < rows; ri++)
        {
            const uint8_t *src = &disp_img[(uint16_t)(r0 + ri) * 2U][0];

            for (ci = 0U; ci < GRAY_W; ci++)
            {
                gp[12U + len] = src[ci * 2U];
                len ++;
            }
        }

        gp[8] = (uint8_t)(len >> 8);
        gp[9] = (uint8_t)len;

        for (ri = 0U; ri < len; ri++) { ck = (uint16_t)(ck + gp[12U + ri]); }
        gp[12U + len]      = (uint8_t)(ck >> 8);
        gp[12U + len + 1U] = (uint8_t)ck;

        if (0U != wifi_f4_send(gp, (uint32_t)(12U + len + 2U))) { break; }   /* 发不动就丢，下一帧还有 */
    }
}

static void cam_tx_row_done (void)
{
    uint16_t txh = (uint16_t)((uint16_t)CAM_H / (uint16_t)cam_cmd_dy);

    cam_tx_row ++;

    if (cam_tx_row >= txh)
    {
        // ★ 边界包已取消（用户要求不画蓝线）⇒ 不再发 0xFE 包，省 132 字节/帧 ✓
        //   二值化仍然在 board_cam_analyze() 里做（Otsu 那部分保留）✓
        //
        // ★★★ 2026-11-05：整帧发完，**紧跟一包"左右边界数组"** ★★★
        //   必须在这里发（disp_hold 还冻着 ⇒ disp_img / cam_bound 都还是这一帧的）；
        //   等下面 `disp_hold = 0` 之后收割就重新开始了。
        if ((0U == SEND_MODE_VOFA) && (0U == cam_img_off)) { cam_send_edges(); }
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

//===================================================主循环===================================================
void app_loop (void)
{
  g_loop_tick = HAL_GetTick();
  g_wdt_on    = 1U;                    /* ★ 从这一圈起，主循环 6 秒不动就自动复位 */

  while (1)
  {
    g_loop_tick = HAL_GetTick();       /* ★ 踩脚印（主循环看门狗，见 stm32f4xx_it.c）*/

    // 几个跨轮次保持的状态（static：只初始化一次）
    static uint8_t  cam_min = 255;      // 最近一帧的最暗值
    static uint8_t  cam_max = 0;        // 最近一帧的最亮值
    static uint32_t cam_avg = 0;        // 最近一帧的平均亮度
    static uint32_t t_retry = 0;
    static uint8_t  relink_n = 0;      /* 连续重连次数，够 3 次就整模块硬复位 */
    static uint32_t t_hb    = 0;
    static uint32_t t_tel   = 0;         // ★ 遥测行节流（10Hz）
    static uint32_t t_camprobe = 0;      // ★ 自动探测摄像头的节流（见下面那段）

    // ==================== ★★ 链路见证人：上位机保活 ★★ ====================
    //   2026-11-07（FW=2099）现场故障：上位机突然一直"已连接 + 0 B/s"，只能断电才好。
    //   根子在于**板子根本发现不了链路已经死了**：
    //     模块被复位 / 掉线 / 揣着一个半死的 socket 时，SPI 写**照样成功**
    //     （模块照收，只是发不出去）⇒ `wifi_f4_send()` 不报错 ⇒ `wf_ready` 一直是 1
    //     ⇒ 板子永远以为还连着，一直往黑洞里发 ⇒ 上位机永远 0 B/s。
    //   ⇒ 这里用"上位机每 2 秒一行的 `!`"当见证人：见过 `!` 之后，6 秒收不到就判链路死，
    //     清 `wf_ready` ⇒ 主循环去重连（连续失败还会整模块硬复位）。
    //   ⚠️ 见过 `!` 才开始看：老上位机不发保活，不能让板子一开机就自己判死。
    {
      static uint32_t rx_last = 0U;
      static uint32_t t_rx    = 0U;

      if (cam_rx_bytes != rx_last)
      {
        rx_last = cam_rx_bytes;
        t_rx    = HAL_GetTick();
      }
      else if ((0U != cam_ka_seen) && (0U != wifi_f4_is_ready()) &&
               ((HAL_GetTick() - t_rx) > 6000U))
      {
        t_rx = HAL_GetTick();
        wifi_f4_force_relink();          /* 判链路死 ⇒ 交给下面的重连逻辑 */
      }
    }

    // ---- 还没连上就每 3 秒自动重试一次 ----
    //   ★ 很关键：wifi_f4_init() 只在开机时连一次 socket。如果那一刻上位机没开，
    //     就会永远连不上、一条数据都不发。这里让它自己重连，**上位机后开也能接上**。
    //   🔥 2026-11-06 二次修复：光重连 socket **救不了"模块数据通路卡死"**——
    //      现象是上位机控制台反复刷 `[已连接]/[对方断开]`，TCP 连上了但一个字节都不发
    //      （模块的 INT 一直低 = 一直"忙"，控制命令还能应答，数据通道已经死了）。
    //      ⇒ 连续重连失败 3 次后**整模块硬复位再连**（wifi_f4_init）。
    if (!wifi_f4_is_ready())
    {
      if ((HAL_GetTick() - t_retry) > 3000)
      {
        t_retry = HAL_GetTick();

        if (relink_n >= 2U)
        {
          relink_n = 0;
          g_wdt_pause = 1U;                     /* ★ 整模块复位最长要几十秒，先暂停看门狗 */
          (void)wifi_f4_init();                 /* 拉 RST + 重新连热点/上位机（阻塞几秒）*/
          g_wdt_pause = 0U;
          g_loop_tick = HAL_GetTick();
          board_led_show(wifi_f4_last_err());
        }
        else
        {
          relink_n ++;
          board_led_show(wifi_f4_retry() ? wifi_f4_last_err() : LED_CODE_READY);
        }
      }
    }
    else
    {
      relink_n = 0;                             /* 连上了就清计数 */
    }

    // ---- ★★ 文本优先时间窗：有文本待发就**让图像让路**，直到发出去（最长 250ms）★★ ----
    //   🔥 为什么必须这么干：模块被图像占满时，文本（遥测/ACK）怎么等都抢不到窗口，
    //      实测就是"图像流畅、实时数据和按键全死"。与其跟图像抢，不如让图像**停一下**，
    //      给文本一条专用通道 —— 少量帧率换遥测/命令可靠，值得。
    {
      static uint32_t t_win_end   = 0;     /* 上一次关窗的时刻（两次窗口之间留一点图像时间）*/
      static uint32_t t_win_start = 0;
      static uint8_t  txt_win     = 0;
      uint32_t now = HAL_GetTick();

      /* ★ 2026-11-06（FW=2096）：**一有待发文本就开窗**，不再"每秒只开一次、固定 30ms"。
         原来那版实测不够：停下图像后模块还要把上一帧（2.9KB）排空才肯收新数据，
         时间到就把窗关了 ⇒ 文本继续排队、继续饿死。
         现在：图像一直让路，直到文本真的发出去（发完立刻恢复，代价只有排空那一小段）。
         上限 250ms 是止损：真发不出去就放图像走，链路复位交给驱动里 3 秒看门狗。*/
      if ((0U == txt_win) && (0U != wifi_f4_is_ready()) &&
          (0U != wifi_f4_text_pending()) && ((now - t_win_end) >= 40U))
      {
        txt_win     = 1U;
        t_win_start = now;
      }

      if (0U != txt_win)
      {
        wifi_f4_text_pump();                       /* 窗口里拼命发文本 */

        if ((0U == wifi_f4_text_pending()) || ((now - t_win_start) > 250U))
        {
          txt_win    = 0U;                         /* 发完 / 超时 ⇒ 关窗，图像继续 */
          t_win_end  = now;
        }
      }

      /* 窗口期间图像让路（下面的 cam_tx_tick 会被跳过）*/
      g_txt_win_active = txt_win;
    }

    // ---- ★ 文本队列（遥测/回执）：每轮试一次，和图像平权抢空闲窗口 ----
    wifi_f4_text_pump();

    // ---- 板载 LED 的闪烁节奏（不阻塞）----
    board_led_task();

    // ---- 轮询 DCMI 状态标志（排"f=0"用；只读寄存器，很快）----
    g_loop_mark = 2U;
    board_cam_poll();

    // ---- ★ 全画面：把 DMA 刚写完的行收割进 disp_img（每 4 行取 1 行、每 4 列取 1 列）----
    //   ⚠️ 必须在**主循环每轮都跑**：一条行在 90KB 环形缓冲里只活 **4.2ms**，
    //      一旦被覆盖就收割不到了 ⇒ 绝不能放进有节流（6ms/500ms）的块里。
    g_loop_mark = 3U;
    board_cam_harvest();

    // ---- ★ 实时命令：把上位机发来的字节攒成行并**立刻执行**（改帧率/抽点/配摄像头）----
    //   ⚠️ 和收割一样必须**每轮都跑**：塞进带节流的块里会让命令延迟几百毫秒才生效，
    //      调参时就会"发了半天没反应"，很难判断是命令没到还是参数没作用。
    g_loop_mark = 4U;
    cam_cmd_poll();

    // ---- ★★ 车模控制节拍（20ms 自己节流）：编码器 / 速度环 / 舵机 / IMU / yaw ★★
    //   🔑 必须放在主循环里、且**绝不阻塞**：内部所有等待都有上界，
    //      舵机走中断发送、IMU 一次约 0.3ms、其余都是几次整数运算。
    g_loop_mark = 5U;
    board_ctrl_tick();

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
    g_loop_mark = 6U;
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
    g_loop_mark = 8U;
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
      if ((0U == g_txt_win_active) && cam_tx_tick(now))
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
            /* ★ 诊断用：把"文本发包成功/失败次数"塞进包头空字节(10/11)
               上位机直接显示 ⇒ 不用靠遥测也能知道文本到底发没发出去 */
            fpkt[10] = (uint8_t)(wifi_f4_txt_ok() & 0xFFU);
            fpkt[11] = (uint8_t)(wifi_f4_txt_fail() & 0xFFU);

            fpkt[8] = (uint8_t)(flen >> 8);
            fpkt[9] = (uint8_t)flen;
            {
                uint16_t ck = 0U;
                for (i2 = 0U; i2 < flen; i2++) { ck = (uint16_t)(ck + fpkt[12 + i2]); }
                fpkt[12 + flen]     = (uint8_t)(ck >> 8);
                fpkt[12 + flen + 1] = (uint8_t)ck;
            }

            if (SEND_MODE_VOFA)
            {
                /* ★ 逐飞助手：整帧一次发完，然后**必须**推进行号
                   ⇒ cam_tx_row_done() 里 `cam_tx_row >= txh` ⇒ 解冻 disp_hold + 回相 0 ✓
                   🔥 少了这两句的后果（实测 2026-11-01）：disp_hold 永远是 1
                      ⇒ board_cam_harvest() 每轮都被跳过 ⇒ disp_img 冻住
                      ⇒ 助手 FPS 正常涨到 26，但**画面定格在第一帧** ✗✗ */
                vofa_send_frame();
                cam_tx_row = (uint16_t)(txh - 1U);
                cam_tx_row_done();

            }
            else if (0U == wifi_f4_send(fpkt, (uint32_t)(12U + flen + 2U)))
            {
                cam_tx_row = (uint16_t)(txh - 1U);      /* 一次性把这一帧算发完 */
                cam_tx_row_done();
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
        if (SEND_MODE_VOFA) { /* 不发逐行包 */ }
        else if (0U == wifi_f4_send(pkt, (uint32_t)(10U + nbytes + 2U)))
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
    g_loop_mark = 9U;
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
                dbg_txt(cb);

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
                dbg_txt(cb);
            }

            board_cam_set_mode(1U);      // ★ 切到 188 宽模式（cam_row_bytes=188 / span=120 / dcol=1）
        }
    }

    // ==================== ★★ 遥测：10Hz 一行（新上位机的仪表用）★★ ====================
    //   格式（故意短，别超过 130 字符 —— 上位机的文本行有长度上限）：
    //     T r=<转速RPM> ax ay az gx gy gz <原始LSB> y=<偏航角, 单位 0.1°> s=<舵机位置> m=<图像模式>
    //   上位机只按 = 取数，顺序/字段名可以随意加（加新字段要同步改 car_ui.py）
    g_loop_mark = 10U;
    if (wifi_f4_is_ready() && ((HAL_GetTick() - t_tel) >= 200U))
    {
      static char tb[192];
      static uint16_t tel_seq = 0U;
      int16_t ax, ay, az, gx, gy, gz;
      int32_t y10, yi, yf;

      t_tel = HAL_GetTick();
      tel_seq ++;
      ctrl_imu_accel(&ax, &ay, &az);
      ctrl_imu_gyro(&gx, &gy, &gz);

      y10 = (int32_t)(ctrl_yaw_deg() * 10.0f);
      yi  = y10 / 10;
      yf  = (y10 < 0) ? -(y10 % 10) : (y10 % 10);

      /* n = 序号：上位机用它算丢包率 —— "板子没发"和"链路丢了"一下子就分开了 */
      sprintf(tb, "T n=%u r=%d ax=%d ay=%d az=%d gx=%d gy=%d gz=%d y=%ld.%ld s=%u m=%u d1=%d d2=%d si=%u sr=%u sb=%u st=%u sba=%lu sl=%u k1=%u k2=%u ol=%u\r\n",
              (unsigned)tel_seq,
              (int)ctrl_speed_get_rpm(),
              (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz,
              (long)yi, (long)yf,
              (unsigned)ctrl_servo_get_pos(), (unsigned)cam_view_mode,
              (int)ctrl_motor_get(1U), (int)ctrl_motor_get(2U),
              (unsigned)ctrl_servo_id(),
              (unsigned)(ctrl_servo_rx_bytes() & 0xFFFFU), (unsigned)(ctrl_servo_rx_bad() & 0xFFFFU),
              (unsigned)(ctrl_servo_tx_bytes() & 0xFFFFU),
              (unsigned long)ctrl_servo_baud(), (unsigned)ctrl_servo_rx_pin(),
              (unsigned)ctrl_key(1U), (unsigned)ctrl_key(2U), (unsigned)ctrl_is_open_loop());
      /* ★ 遥测走**不重试**的发送：10Hz × 每次重试最多 7ms 会按住主循环，
         把摄像头收割和图像发送都拖慢（实测就是"数据一会有一会没有"）。
         丢一条遥测没关系，下一条 100ms 后就来。 */
      if (0U == SEND_MODE_VOFA) { wifi_f4_text(tb, 1U); }   /* pri 1 = 遥测：比心跳优先 */
    }

    // ==================== ★ 心跳：每 500ms **无条件**发一条 ★ ====================
    //   🔥 上一版这里是"只在收到摄像头帧时才发"——结果摄像头一出问题，
    //      **一个字节都发不出去**，看起来像"WiFi 挂了"，极难排查。
    //      ⇒ 输出绝不能挂在另一个可能坏掉的外设上。心跳永远发，摄像头状态放在内容里。
    g_loop_mark = 11U;
    if (wifi_f4_is_ready() && ((HAL_GetTick() - t_hb) >= 500))
    {
      static char buf[320];   /* ★ static：进 .bss，不吃栈（最长的一行 S 约 203B，最坏 271B）*/
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
        sprintf(buf, "S f=%lu V=%lu VS=%lu LG=%lu TO=%lu F=%u FL=%u P=%u RX=%lu CAM=%u ID=%04X N=%u MD=%u BIN=%u R0D=%04X W=%u E=%u K=%lu R=%04lX IE=%04lX MI=%04lX LW=%lu GP=%lu FW=2131 LM=%lu RST=%02lX PC=%08lX CF=%04lX BA=%08lX\r\n",
                (unsigned long)cam_frame_cnt, (unsigned long)cam_vsync_isr_cnt,
                (unsigned long)cam_vs_cap_cnt,
                (unsigned long)disp_lag_max,
                (unsigned long)cam_to_cnt, (unsigned)cam_frz_fill,
                (unsigned)cam_frm_lines,
                (unsigned)dbg_pol_idx,
                (unsigned long)cam_rx_bytes,
                (unsigned)cam_if_ok, (unsigned)cam_if_id, (unsigned)cam_if_tries,
                (unsigned)cam_mode, (unsigned)cam_bin_ok,
                (unsigned)cam_r0d,
                (unsigned)board_cam_write_row(), (unsigned)cam_last_start_err,
                (unsigned long)t_pclk, (unsigned long)dcmi_ris,
                (unsigned long)dbg_dcmi_ier, (unsigned long)dbg_dcmi_misr,
                (unsigned long)cam_line_isr_cnt, (unsigned long)cam_ln_gap_cnt,
                (unsigned long)flt_mark,
                (unsigned long)cam_rst_flags,
                (unsigned long)flt_pc, (unsigned long)flt_cfsr, (unsigned long)flt_bfar);
        dbg_txt(buf);
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
      dbg_txt(buf);

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
      dbg_txt(buf);

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
        dbg_txt(buf);
      }

      // ---------- 第 3.5 行：★ 全画面"收割"健康度 ----------
      //   LG = 单次收割**最多补了几行**。一条行只活 120 行 ≈ 4.2ms，
      //        主循环周期 T ⇒ 每次要补 T/36.5µs 行：T=0.5ms→14；T=2ms→55；T=4.2ms→115（几乎全漏）
      //        ⇒ **LG ≥ 30 就说明主循环太慢，画面一定在漏行**（表现为横向撕裂/重复带）
      //   PS = 已经凑齐多少幅全画面（走完 480 行 +1）。**57fps 下应约 14/秒**
      //        ⇒ 500ms 心跳里 PS 应增加约 7。不涨 ⇒ 收割没在跑；涨太慢 ⇒ 漏行
      sprintf(buf, "CAM4 LG=%lu PS=%lu\r\n",
              (unsigned long)disp_lag_max, (unsigned long)disp_pass);
      dbg_txt(buf);

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
        dbg_txt(buf);
        sprintf(buf, "MD %lu\r\n", (unsigned long)md);
        dbg_txt(buf);
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
        dbg_txt(buf);
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
        dbg_txt(buf);

        live_hi[0] = 0; live_hi[1] = 0; live_hi[2] = 0; live_hi[3] = 0;
        live_n = 0;
      }
      }   // ← 全量诊断（d >= 2）到此为止；d 0/1 时这一段整块跳过
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}
