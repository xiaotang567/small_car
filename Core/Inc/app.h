#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

/* ★ 上位机模式：1 = 逐飞助手（发图像，**不发任何文本**）
        0 = 自定义协议（发图像 + 文本心跳）
   ★★★ 2026-11-02 切回 0（cam_view.py）★★★
   原因：助手协议是 **8bit/像素、无压缩**，一帧 188×120 = 22568B（6 个 4088B 块），
        这条链路只有约 250KB/s ⇒ 一帧 90ms ⇒ **11fps 封顶**，压到 f=0 直接崩成 0.9fps。
   而这条老协议是 **整帧一包 + 位打包**：12 + 120×24 + 2 = **2894B，1 个块** ⇒
        一帧约 4.4ms ⇒ 帧率由传感器（50fps）和"歇"决定，轻松 16fps 以上，且画面仍是 188×120。
   ⇒ 这就是 cbb73c8 之前那个"16 帧"的配置，原样恢复。*/
#ifndef SEND_MODE_VOFA
#define SEND_MODE_VOFA      0      /* 0 = 自研上位机 cam_view.py（默认）；1 = 逐飞助手 */
#endif

/* ---- 上位机可调参数（命令通道 / 相位机 / 主循环 共享）---- */
extern uint16_t          cam_cmd_fps;
extern uint16_t          cam_cmd_gap_ms;
extern uint8_t           cam_cmd_dx;
extern uint8_t           cam_cmd_dy;
extern uint8_t           cam_dbg_level;
extern uint32_t          cam_rst_flags;
extern volatile uint32_t flt_pc, flt_lr, flt_cfsr, flt_hfsr, flt_bfar;
extern uint8_t           cam_pack;
extern uint32_t          cam_to_cnt;
extern uint8_t           cam_frz_fill;
extern uint8_t           cam_if_ok;
extern uint16_t          cam_if_id;
extern uint8_t           cam_if_tries;
extern volatile uint32_t cam_rx_bytes;
extern volatile uint8_t  cam_rx_dump[32];
extern volatile uint32_t cam_rx_dump_i;

/* ★ 边界数组包（0xFD）的发送统计：上位机 `s` 里回 EL/ER/EDG，
   用来确认"新固件在发边界数组"+"这一帧左右各爬到多少行"（不用看图也能判断） */
extern uint32_t          cam_edge_pkt_cnt;
extern uint8_t           cam_edge_l_cnt;
extern uint8_t           cam_edge_r_cnt;

/* ★ 发送看门狗：因为"一帧迟迟发不出去"而主动放弃的帧数（CFG 里的 DRP）
   ⇒ 板子不会因为某一帧发不动就永久停住；这个数偶尔涨几下是正常的，
     一直飙 = 链路/模块有问题 */
extern uint32_t          cam_tx_drop;

/* ★ 采集自愈：连续 2 秒没有"完整帧"（disp_done）就重启一次 DCMI 的次数（CFG 里的 SELF）
   ⇒ 正常时恒 0；一直涨 = 采集链真的有问题（配合 board_f4.h 的 cam_dma_stall 看） */
extern uint32_t          cam_self_heal;

void cam_cmd_poll (void);
void app_loop     (void);

#endif
