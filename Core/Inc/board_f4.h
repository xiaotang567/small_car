/*********************************************************************************************************************
* 文件名称          board_f4
* 适用平台          STM32F407
* 说明              板级支持：时钟树 + 总钻风摄像头(DCMI)采集
*
* ★ 为什么要自己配时钟：
*     CubeMX 生成的 SystemClock_Config() 里 HSE_VALUE 是 **25MHz**（CubeMX 的默认值），
*     而这块板子（立创·梁山派-天空星）的晶振丝印是 **YXC 8.000 = 8MHz**。
*     ⇒ 直接用它，PLL 会把 168MHz 算成 53.8MHz，所有串口波特率全错。
*     所以这里重新配一遍，并把它放在 USER CODE 区调用 —— **CubeMX 重新生成也不会被覆盖**。
*
* ★ 怎么确认晶振到底是几 MHz（10 秒）：
*     1. 拿起板子，看那颗 4 脚的金属小方块（晶振），上面印着 "8.000" 或 "25.000"
*     2. 就按印的数填下面的 BOARD_HSE_MHZ，重编一次即可
*   还没法确认时先按 8 跑：如果 VOFA 上收到的 hello world **间隔变成约 1.5 秒**
*   （本该 500ms），说明实际是 25MHz，把宏改成 25 重编。
*
* 修改记录
* 日期              作者                备注
* 2026-10-13        织命                first version
********************************************************************************************************************/

#ifndef _BOARD_F4_H_
#define _BOARD_F4_H_

#include "main.h"

//===================================================板子参数（改这里）===================================================

// 晶振频率（MHz）。**只能是 8 或 25**（其它值要自己补 PLL 分频）
//   立创·梁山派-天空星：照片上晶振丝印为 YXC 8.000 ⇒ 默认 8
#define BOARD_HSE_MHZ       8

// 摄像头分辨率（DCMI/DMA 按这个采）
//   🔥🔥 2026-10-14 20:15 **用图定量实锤：真实行宽 = 752 字节**（摄像头在 752×480 档）
//      证据：画面有**严格的 2 行周期**（行均值自相关 lag2=+0.97、lag1=-0.40，
//      奇偶行差恒为 +104~+118）⇒ 缓冲一行(376)只装了**半条**摄像头行
//      ⇒ 偶数行显示某行的左 1/4、奇数行显示它的第 2 个 1/4 ⇒ 横向细条纹 ✓
//      且 PCLK≈20.6MHz ÷ (752×480) ≈ 57fps ✓ 两条独立证据吻合。
//   ⇒ `CAM_W` 必须是 **752**（一行 752 字节）；
//     `CAM_H=120` ⇒ 缓冲装 120/480 = **1/4 个画面**（用户最早说的"1/4"✓）。
//   ⚠️ RAM：cam_image = 752×120 = 90240B，快照另算（见 CAM_TXW）⇒ 共 ~110KB，F407 放得下。
#define CAM_W               752
#define CAM_H               120

// 发送/快照的列数：**固定 188**（= 752 的左 1/4）。
//   ⚠️ 不能按 `CAM_W/CAM_DEC` 算（那会是 376）——快照缓冲会变成 120×376=45120B，
//      加上 cam_image 90240B = 135KB，**超过 F407 的 128KB RAM** ⇒ 编译都过不了。
//   ⇒ 只发每行开头 188 个字节 ⇒ 看到的是画面**左上角 1/16**（188×120 的窗口，
//      内容比例 188:120 = 1.57:1，和整幅 752:480 一样 ⇒ **无拉伸、无变形** ✓）。
//   📌 想看更大的视野只有两条路：① 用串口把摄像头配成 376×240（整帧 90KB 正好装下）；
//      ② 加大 RAM（换 F407 的 CCM 放快照）。
#define CAM_TXW             188
#define CAM_DEC             2       // （历史遗留，现在发送列数已固定为 CAM_TXW，不再用它算）
//===================================================板子参数===================================================


//===================================================对外接口===================================================
extern volatile uint8_t board_clock_src;        // 1 = 用了 HSE  2 = HSE 没起来，退回 HSI
extern volatile uint32_t board_sysclk_hz;       // 实际系统时钟（Hz）

// 重新配置时钟树到 168MHz。放在 USER CODE BEGIN SysInit 里调用（在 SystemClock_Config 之后）
void    board_clock_init (void);

// ---- 摄像头（DCMI + DMA）----
extern volatile uint32_t cam_frame_cnt;         // 已采到的完整帧数
extern volatile uint8_t  cam_ready;             // 是否有新帧待处理（处理完自己清 0）
extern uint8_t           cam_image[CAM_H][CAM_W];

// ---- ★ 全画面缩略图（每 4 行取 1 行、每 4 列取 1 列）----
//   整帧 752×480 = 360KB，128KB RAM 装不下 ⇒ 只能"边采边抽点"：
//   主循环的 `board_cam_harvest()` 把 DMA 刚写完的每一行（752B）抽点成 188 列，
//   480 行每 4 行取 1 行 ⇒ **整幅画面 188×120 = 22560B**（比例 1.57:1，和整帧一致）。
//   帧首由 VSYNC 中断对齐（`disp_ln=0`、`disp_next_row=当时写入行`）。
extern uint8_t           disp_img[CAM_H][CAM_TXW];   // ★ 全画面 188×120（22560B）
extern volatile uint32_t disp_frame;                 // 全画面的帧数（VSYNC 一次 +1）
extern volatile uint8_t  disp_hold;                  // 1 = 冻结收割（发一张图期间，保证整帧来自同一次）
extern volatile uint32_t disp_pass;                  // 行号回绕次数（**仅诊断**，别拿它判"凑齐"）
extern volatile uint32_t disp_lag_max;               // ★ 单次收割最多补了几行（≥30 ⇒ 主循环太慢，在漏行）
extern volatile uint8_t  disp_fill;                  // ★★ 本帧已填到第几行（1..CAM_H）；VSYNC 清零
extern volatile uint8_t  disp_done;                  // ★★ 1 = **本帧 120 行全部来自同一帧**（可以发）
extern volatile uint8_t  disp_armed;
extern volatile uint32_t cam_vsync_isr_cnt;          // ★★ VSYNC 中断进了几次（V= 在涨 ⇒ 中断真的在工作）
extern volatile uint16_t cam_frm_lines;              // ★ 188 模式实测帧长（相邻两次 VSYNC 之间走了多少行）
extern volatile uint16_t cam_frm_start_row;          // ★ 最近一次 VSYNC 时 DMA 写在环形缓冲的第几行
extern volatile uint16_t cam_span;                   // ★★ 一帧的摄像头行数（实测 240；`board_cam_set_span` 可改）
void    board_cam_set_span (uint16_t rows);          // ★ 实时改"一帧多少行"（抽行步长随之重算）
void    board_cam_harvest (void);                    // 主循环每轮调一次（逐行收割）

// ---- ★★ 摄像头"配置串口"(UART5：PC12=TX / PD2=RX) 的**实时手动控制** ★★ ----
//   🔑 为什么拆得这么细：这个口**至今没有任何应答**（CST=2，RV=0），而"一整套配置包灌下去"
//      还可能把摄像头配死（VSYNC 直接没了）。所以改成"**一个命令一个动作**"，
//      配合上位机实时下发、边发边看图 —— 用最小步长把它试出来，而不是一次赌一把。
extern volatile uint8_t  cam_cfg_state;              // 0=没做 1=成功 2=没应答 3=配了没应答 4=TX低 5=RX低
extern volatile uint32_t cam_cfg_ver;                // 收到的原始字节（低 24 位）

// 打开/切换配置串口波特率档位：0=9600 1=115200 2=57600 3=38400 4=19200
void     cam_cfg_uart_open  (uint8_t baud_idx);
// 发一个 4 字节配置包： 0xA5 | cmd | val 高8 | val 低8  （cmd=0x00 就是 INIT）
void     cam_cfg_send_pkt   (uint8_t cmd, uint16_t val);
// 发 GET_VERSION(0xF2) 并收回原始应答；返回"收到几个字节"（0 = 它一声不响）
uint32_t cam_cfg_read_ver   (void);
// 读两根线的静态电平（UART 空闲时都该是高）： bit1 = PC12(TX)   bit0 = PD2(RX)
uint8_t  cam_cfg_line_state (void);

// ---- ★★ 配置串口"探活"（一次分清：线没接 / 波特率不对 / 它其实回话了）★★ ----
//   发一个**文档里确有的** SET_COL 命令，然后 60ms 内直接量 PD2 的电平变化 + UART 错误位
extern volatile uint32_t cam_probe_edges;   // 0 ⇒ 线上一点动静都没有（接线/焊点问题）
extern volatile uint32_t cam_probe_low;
extern volatile uint32_t cam_probe_n;
extern volatile uint32_t cam_probe_rx;      // >0 ⇒ 收到字节了 ✓
extern volatile uint8_t  cam_probe_err;     // bit0=FE 帧错误 bit1=NE 噪声 bit2=ORE 溢出
// 【阶段B】一边持续发 0x55 一边采样 —— 用于"PC12↔PD2 短接"自检：
//   短接后 B 阶段 EDGES 很大/RX>0 ⇒ 我们这侧通路正常 ⇒ 问题在摄像头那根线
extern volatile uint32_t cam_probe_b_edges;
extern volatile uint32_t cam_probe_b_rx;
extern volatile uint32_t cam_probe_b_n;
extern volatile uint32_t cam_probe_b_tx_edges;  // 阶段B 里 PC12(我们TX) 的边沿数（>0 ⇒ 我们确实在发）
extern volatile uint8_t  cam_probe_c_np;        // 阶段C：PD2 无拉/上拉/下拉 三种情况下读到的电平
extern volatile uint8_t  cam_probe_c_pu;        //   PU=1 且 PD=0 ⇒ **PD2 悬空（没接东西 / 脚没引出来）**
extern volatile uint8_t  cam_probe_c_pd;
void     cam_cfg_probe (uint8_t baud_idx);  // 0..4 = 9600/115200/57600/38400/19200

// ★★★ 波特率扫描：一次把 7 档都试一遍（9600/115200/57600/38400/19200/4800/230400）★★★
//   先做 PD2 三态检测，再逐档发 SET_COL 听 30ms，结果存进下面两个数组。
//   🔑 用途：分清"线没接"（每档都是 0）和"波特率不对"（换对档就有回话）。
void     cam_cfg_scan (void);
extern volatile uint32_t cam_scan_rx[7];    // 每档收到的字节数
extern volatile uint32_t cam_scan_eg[7];    // 每档 PD2 上的跳变次数

// ---- ★★★ 软件 I2C / SCCB（真正的正路：这颗摄像头没有 51 配置芯片，只能直接读写传感器寄存器）★★★ ----
//   🔑 接线不用动：就是你现在接的那两根（PD2 / PC12），它们本来就是 SDA/SCL。
//   ⚠️ 哪根是 SDA 不确定 ⇒ `cam_sccb_test()` 两种接法都试，哪个读出 ID 就是对的。
//   期望：MT9V034 的芯片版本寄存器 R0x0000 = **0x1313**
void     cam_sccb_init       (uint8_t swap);        // 0: PD2=SDA/PC12=SCL；1: 反过来
uint8_t  cam_sccb_test       (void);                // 返回 0=都没通 1=接法0通 2=接法1通
uint8_t  cam_sccb_write_reg  (uint16_t reg, uint16_t val);
uint8_t  cam_sccb_read_reg   (uint16_t reg, uint16_t *val);   // 0 = 成功
extern volatile uint16_t cam_sccb_id0;              // 接法0 + 地址0x90 读到的 ID
extern volatile uint16_t cam_sccb_id1;              // 接法1 + 地址0x90
extern volatile uint16_t cam_sccb_ba;               // 地址 0xBA（部分模组）
extern volatile uint8_t  cam_sccb_lv0;              // 接法0 的静态电平：bit1=SDA bit0=SCL
extern volatile uint8_t  cam_sccb_lv1;              // 接法1 的静态电平
extern volatile uint8_t  cam_sccb_err;              // 读寄存器卡在第几步（1 = 器件地址没 ACK）
// ★ 手册给的四个从机地址（0x90/0x98/0xB0/0xB8，由 S_CTRL_ADR0/ADR1 决定）——
//   之前只试了 0x90 和一个手册里没有的 0xBA，**这就是一直读不到 ID 的原因之一**
extern volatile uint8_t  cam_sccb_addr;             // 探测成功时用的是哪个地址

// ★ 把传感器配成 188×120（列 bin4 + 行 bin4，手册 R0x0D[3:2] / R0x0E[1:0]）
//   成功 ⇒ 一帧正好 22560 字节 = DMA 缓冲大小 ⇒ 配合 `board_cam_set_mode(1)` 走"缓冲即整帧"
uint8_t  cam_sccb_set_bin4 (void);
void     board_cam_analyze (void);           // ★ Otsu 二值化 + 每行亮/暗交界列（采齐后调）
extern volatile uint8_t  cam_bound[120];      /* 左边界 */
extern volatile uint8_t  cam_bound_r[120];    /* 右边界 */
extern volatile uint8_t  cam_center[120];     /* 中线 = (左+右)/2 */
extern volatile uint8_t  cam_now;
extern volatile uint8_t  cam_ahead;
extern volatile uint16_t cam_fps10;
extern volatile float    lf_dbg_a, lf_dbg_b, lf_dbg_c, lf_dbg_dev;     // 每行交界列（0xFF=无）
extern volatile uint8_t  cam_thresh;         // Otsu 阈值
extern volatile uint8_t  cam_shutter;        // 曝光 R0x0B（行数，`e <n>` 可调）
void     cam_sccb_apply_flip (void);         // ★ 按 cam_flip 重写 R0x0D 的翻转位（bit5=左右镜像 bit4=上下）
extern volatile uint8_t  cam_flip;           // 0=不翻 1=镜像 2=上下 3=180°
extern volatile uint8_t  cam_bin_ok;                // 1 = 配置成功且回读一致
extern volatile uint16_t cam_r0d;                   // R0x0D 回读值
extern volatile uint16_t cam_r0e;                   // R0x0E 回读值

// ★ 对 PC12 / PD2 的"三态检测"：**这两个脚上到底有没有东西接着？**
//   PU=1 且 PD=0 ⇒ **空脚**（线没通 / 脚没引出）✗    PU=1 且 PD=1 ⇒ 确实接着东西 ✓
extern volatile uint8_t  cam_pin_c12_pu, cam_pin_c12_pd;
extern volatile uint8_t  cam_pin_d2_pu,  cam_pin_d2_pd;
void     cam_pin_tristate_all (void);

// ---- ★★★ 运行时的"采集模式"（配合摄像头被配成 188×120）★★★ ----
//   0 = 相机全分辨率 752×480（走环形缓冲 + 抽行，当前默认）
//   1 = 相机就是 188×120 ⇒ **缓冲天然就是一整帧**，不需要抽行（每行 188 字节、一帧 120 行）
//   ⚠️ 切模式会**重启 DMA**（传输长度必须跟着改）⇒ 调用后画面会短暂中断，正常。
extern volatile uint16_t cam_row_bytes;              // 摄像头一行的字节数（752 或 188）
extern volatile uint16_t cam_mode;                   // 0 / 1
void     board_cam_set_mode (uint8_t mode);          // ★ 切采集模式（含重启 DMA + 重算相位）
uint8_t  board_cam_restart (void);                   // ★ Stop+Start 重启 DCMI（传感器改寄存器后必须调）
uint32_t board_cam_pclk_khz (void);                  // ★ 量 PA6 的实际频率（kHz）：0=传感器没输出 / 5300≈列bin4 / 21000=没生效
// ★ 开始新一轮采集：把"已填计数 + 120 位位图 + 采齐标志"一起清零（相位机进入相 1 时调用）
//   🔑 计数/位图**只在这里**清零 —— 回绕处绝不再清零（那是"突发补行跨回绕丢进度"的根源）
void     board_cam_new_pass (void);

// ★ 按手册把摄像头配成 188×120 + **PCLK消隐=1** + 指定帧率；返回"收到几个字节"（0 = 它没回）
//   ⚠️ 这一位（消隐=1）是 STM32 的 DCMI 能正确分帧的前提，仅 MT9V034 **V2.1+** 支持
uint32_t cam_cfg_apply_188 (uint16_t fps);
// ---- 摄像头诊断（排"f=0"用，全部塞进心跳发出去）----
extern volatile uint32_t cam_err_cnt;           // DCMI 报错的次数（同步错/溢出）
extern volatile uint32_t cam_vsync_cnt;         // 轮询到 **VSYNC 有效** 的次数  ← 最关键
extern volatile uint32_t cam_fne_cnt;           // 轮询到 **FIFO 非空** 的次数（说明有像素进来）
extern volatile uint8_t  cam_last_start_err;    // 上一次 HAL_DCMI_Start_DMA 的返回（1=启动就失败）

// 🔥 丈量行宽用：**Δcam_bytes ÷ Δcam_line = 每行字节数**（= 摄像头真实行宽）
//   —— 有了它就不用再猜 752 / 376 / 188 了。
extern volatile uint32_t cam_bytes;             // DMA 累计搬运的字节数（从 NDTR 增量累加）
extern volatile uint32_t cam_line;              // DCMI 报"一行结束"的次数（RISR 的 LINE 位）

// 🔑 **帧边界在缓冲里的位置（第几行）** —— 由 VSYNC 回调用 `NDTR` 实时量出来。
//   上传时从这一行开始取 ⇒ 图从"帧首"开始，"绕回缝"被推到缓冲末尾（画面最底边）。
extern volatile uint16_t cam_off;

// 🔑 **DMA 当前写到缓冲的第几行**（= 缓冲里最老的那一行）。
//   🔥 发图前拿它当"读取起点" ⇒ 读到的就是**按时间排序**的连续行（从最老到最新），
//      而不是"地址顺序"里夹着一个绕回点的乱序数据 ⇒ 那"一道线"被推到画面最底边。
//   ⚠️ 必须每次读取前**现读**（缓冲 4.4ms 就整圈覆盖一次，缓存下来的值立刻就过期）。
uint32_t board_cam_write_row (void);

// 🔥 **帧边界落在缓冲的第几行**（0xFFFF = 还没抓到）。
//   🔑 抓法：主循环轮询 `DCMI->RISR` 的 **FRAME 位**（硬件置起），置起那一刻读一次 `NDTR`。
//      它 = "上一帧刚写完、下一帧即将开始"的位置 ⇒ 从这一行开始读 = **整帧从帧首开始** ✓
//   ⚠️ 与 `board_cam_write_row()` 的区别：
//      `board_cam_write_row()` = 缓冲里**最老**的那一行（消除"时间错乱"）
//      `cam_frame_row`        = **帧首**那一行（消除"整帧转了个圈"）—— 这两个值不一样。
extern volatile uint16_t cam_frame_row;

// 🔥 活引脚"占空比"采样：主循环每轮采一次 PA6/PB7/PA4/PC6 的电平，累计高电平次数。
//   与 dbg_flt 的区别：**这个是实时的**（dbg_flt 只在开机头 9 秒做悬浮检测）。
//   顺序： 0=PA6(PCLK)  1=PB7(VSYNC)  2=**PA4(HREF)**  3=PC6(D0)
extern volatile uint32_t live_hi[4];
extern volatile uint32_t live_n;
void    board_cam_poll (void);                  // 主循环每轮调一次（只读标志位，很快）

// ---- 🔑🔑 引脚"悬浮/断线"检测（不需要示波器，决定性）----
//   ⚠️ 为什么需要它：**悬空的输入脚会被邻近走线耦合，读出来也像 ~50% 在跳**，
//      和真的 PCLK 看上去一模一样 —— 光看"占比"根本分不出来。
//   原理：给被查的线轮流加"内部上拉 / 内部下拉"（STM32 内部约 40kΩ），
//         再看它的高电平占比会不会被"拉过去"：
//           真信号在驱动 ⇒ 40k 拉不动它 ⇒ 三次占比**基本一样**
//           线悬空 / 断了 / 摄像头没驱动它 ⇒ **上拉→≈100%，下拉→≈0%，变化巨大**
//   读法：dbg_flt[pin][0]=无拉  [1]=上拉  [2]=下拉，单位 %
//         顺序：0=PA6(PCLK)  1=PB7(VSYNC)  2=PA4(HREF)  3=PC6(D0)
//   主循环每轮自动更新；开机后只跑约 9 秒就收手，不挡 DCMI 干活。
extern volatile uint32_t dbg_flt[4][3];
extern volatile uint8_t  dbg_flt_done;          // 1 = 至少扫完过一轮
extern volatile uint32_t dbg_vsync_edge;        // VSYNC 边沿数（证明这是真场同步，不是噪声）
void    board_float_probe (void);               // 主循环每轮调一次

// ---- 🔑 DMA 流到底使能没有（ND 卡住时的必查项）----
extern volatile uint32_t dbg_dma_en;            // DMA2_Stream1->CR 的 EN 位。**期待 1**
extern volatile uint32_t dbg_dma_lisr;          // DMA2->LISR（低 6 位 = Stream1 的 FEIF/DMEIF/TEIF/HTIF/TCIF）

// ---- 🔑 PCLK 的实际频率（kHz）—— 唯一没量过的量 ----
//   用 TIM3 的"外部时钟模式1"把 PA6(TIM3_CH1) 借来当计数器，量 20ms，然后立刻还给 DCMI。
//   这一步能一刀切开两种情况：
//     几十~几万 kHz 里的"MHz 级"  ⇒ 是真像素时钟 ⇒ 问题在 DCMI 侧
//     只有几十 kHz（≈60Hz 是 VSYNC、≈7kHz 是 HREF）⇒ **这根线上的不是 PCLK**（接错/换线）
extern volatile uint32_t dbg_pclk_khz;
extern volatile uint32_t dbg_vsync_khz;
uint32_t board_vsync_hz (void);          // ★ 量 PB7(VSYNC) 的实际边沿率：~50=正常成帧 / 0=传感器没在成帧

// ---- 🔑 摄像头串口配置（UART5：PC12=TX / PD2=RX @9600）----
//   总钻风的分辨率存在它**自己的 EEPROM** 里，所以开机必须下发一次；
//   🔥 而且手册第 20 页明确要求：**用 STM32 的 DCMI 采集时，"PCLK 消隐信号" 必须设为 1**
//      （官方驱动里的默认值是 0）—— 这一位不设，DCMI 采不到东西。
//   cam_cfg_state: 0=还没做  1=配置成功(收到 A5 FF FF)  2=没收到应答(先查 TXD/RXD 是否交叉接反)
extern volatile uint8_t  cam_cfg_state;
extern volatile uint32_t cam_cfg_ver;
uint8_t cam_cfg_camera (void);

// ---- 🔑 DCMI 寄存器快照：看"它到底有没有真的在跑" ----
//   CR   : DCMI->CR。**期待 ≈ 0x4081**（CAPTURE=1 位0 + ENABLE=1 位14 + VSYNC低有效 位7）
//          如果 CAPTURE 或 ENABLE 是 0  ⇒ 启动没生效（启动函数被挡了/被别处关掉了）
//   NDTR : DMA2_Stream1->NDTR，剩余传输数。**它应该在往下掉**；
//          一动不动 ⇒ 一个字节都没从 DCMI 搬过来
//   CS/DS: hdcmi.State / hdma.State（0=RESET 1=READY 2=BUSY 3=TIMEOUT 4=ERROR）
//   DE   : hdma.ErrorCode（DMA 报的错）
extern volatile uint32_t dbg_dcmi_cr, dbg_dma_ndtr, dbg_dma_err;
extern volatile uint8_t  dbg_dcmi_state, dbg_dma_state;
void    board_cam_dbg (void);

// ---- 🔑 引脚复用/时钟是否真的生效（DCMI->SR 全 0 时必须查这个）----
//   BM : GPIOB->MODER 里 PB7 那两位。**期待 2**（=复用模式）。0/1 ⇒ 引脚还是输入/输出模式，
//        ⇒ 信号到不了 DCMI（但 IDR 照样能读到跳变，所以之前看不出来）
//   BA : GPIOB->AFR[0] 里 PB7 的复用号。**期待 13**（0xD = DCMI）
//   BC : RCC->AHB2ENR 的 DCMI 时钟位。**期待 1**
extern volatile uint32_t dbg_pb7_moder, dbg_pb7_af, dbg_dcmi_clk;
// 🔑 11 根 DCMI 引脚里，**没配成复用模式(AF)** 的那些（按位表示，0 = 全都配对了）
//    bit0 PA4  bit1 PA6  bit2 PB6  bit3 PB7  bit4..7 PC6..PC9  bit8..10 PE4..PE6
//    ⚠️ 之前只查了 PB7 一根，其余 10 根没查过 —— 这个掩码一次把 11 根全查了
extern volatile uint32_t dbg_pin_bad;

// ---- 🔑 DCMI 原始状态寄存器（比 HAL 的 FLAG 宏可信）----
extern volatile uint32_t dbg_dcmi_sr, dbg_dcmi_ris;   // SR=同步脚状态  RIS=原始中断标志(ERR/OVR/FRAME)
extern volatile uint32_t dbg_dcmi_ier, dbg_dcmi_misr; // ★ IER=使能位  MISR=已使能且挂起的中断
extern volatile uint32_t cam_line_isr_cnt;            // ★ LINE 中断进了几次（诊断：应 ~120×帧率）
extern volatile uint32_t cam_ln_gap_cnt;              // ★ 其中"行间隔变大"（场消隐）几次
extern volatile uint32_t cam_vs_cap_cnt;              // ★ TIM4 捕获到几次 VSYNC（188 模式）
extern volatile uint32_t cam_vs_hz;                   // ★ TIM4 测出的场频
void board_vsync_poll_init (void);                    // 188 模式：TIM6 每 50µs 轮询 DCMI->SR 的 VSYNC
void board_vsync_poll_irq  (void);                    // TIM6_DAC 中断转发
extern volatile uint8_t  dbg_pol_idx;                 // 当前试到第几种极性组合(0~7)
extern volatile uint8_t  dbg_pol_locked;              // 1 = 已锁定(说明采到帧了)

// 🔑 极性自动搜索：每 2 秒换一种 (PCKPOL,HSPOL,VSPOL) 组合，谁采到帧就停在那儿。
//   总钻风 / 各家 DVP 摄像头的极性没有统一标准，与其猜不如让板子自己试。
void    board_cam_pol_search (void);

// 🔑 **当前极性的组号（0~7）** —— 存在 RTC 备份寄存器里，**按一下 RST 就换下一组**。
//   `board_cam_init()` 在**启动采集之前**就按它把 DCMI->CR 的极性位配好
//   （DCMI 跑起来之后中途改极性是**同步不上**的 ⇒ 必须在 Start 之前配）。
//   上位机窗口标题上的"极性=N"就是这个数（包头的 pkt[6]）。
uint8_t board_cam_pol_index (void);

// 初始化 DCMI + DMA（SNAPSHOT 模式：采满一帧自动停）
uint8_t board_cam_init (void);
// 启动下一帧（主循环处理完上一帧后调用）
uint8_t board_cam_start (void);
// 中断转发（在 stm32f4xx_it.c 里调用，别在别处调）
void    board_cam_dma_irq (void);
void    board_cam_irq (void);

// ---- 板载 LED（PB2，高电平点亮）—— 没有屏幕时用它报状态 ----
//   板子上的绿色小灯，靠它就能看出 WiFi 卡在哪一步，不用接调试器。
#define LED_CODE_READY      0       // 常亮       : 一切就绪，正在发数据
#define LED_CODE_NO_MODULE  1       // 闪 1 下    : 模块没应答（SPI/供电/RST）
#define LED_CODE_NO_WIFI    2       // 闪 2 下    : 连不上热点（SSID/密码/不是2.4G）
#define LED_CODE_NO_PC      3       // 闪 3 下    : 连不上电脑（IP/上位机没开/防火墙）
#define LED_CODE_RETRY      4       // 慢闪       : 还没就绪，正在自动重试

void    board_led_init (void);
void    board_led_show (uint8_t code);      // 设定要显示的状态码
void    board_led_task (void);              // 主循环每轮调一次（负责闪烁节奏）
//===================================================对外接口===================================================

#endif
