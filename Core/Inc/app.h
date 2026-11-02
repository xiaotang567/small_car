#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

/* ★ 上位机模式：1 = 逐飞助手（发图像，**不发任何文本**）
        0 = 自定义协议（发图像 + 文本心跳）*/
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
extern volatile uint32_t flt_pc, flt_lr, flt_cfsr, flt_hfsr;
extern uint8_t           cam_pack;
extern uint32_t          cam_to_cnt;
extern uint8_t           cam_frz_fill;
extern uint8_t           cam_if_ok;
extern uint16_t          cam_if_id;
extern uint8_t           cam_if_tries;
extern volatile uint32_t cam_rx_bytes;
extern volatile uint8_t  cam_rx_dump[32];
extern volatile uint32_t cam_rx_dump_i;

void cam_cmd_poll (void);
void app_loop     (void);

#endif
