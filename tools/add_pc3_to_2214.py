# -*- coding: utf-8 -*-
"""在 46a9bff（FW=2214）上补 PC3 = 直接启动/停止按钮（把 0f727e1 里的这部分摘出来，
   不带那一版的其他改动，比如按行宽度表）。"""
import io
import sys

SRC = r'C:\Users\SHUIGUI\Documents\small_car\Core\Src'
INC = r'C:\Users\SHUIGUI\Documents\small_car\Core\Inc'

def rd(p):
    return io.open(p, 'r', encoding='utf-8', newline='').read().replace('\r\n', '\n')

def wr(p, s):
    io.open(p, 'w', encoding='utf-8', newline='\n').write(s)

def sub(t, old, new, tag):
    if t.count(old) != 1:
        print('FAIL %-30s hits=%d' % (tag, t.count(old)))
        sys.exit(1)
    print('ok   %-30s' % tag)
    return t.replace(old, new)

# ---------- 1) ctrl_f4.h：加声明 ----------
p = INC + r'\ctrl_f4.h'
t = rd(p)
t = sub(t,
        'uint8_t  ctrl_key (uint8_t idx);           /* idx = 1=PC2 / 2=PC3，返回 1 = **按下**（高电平有效，接 3.3V） */\n',
        'uint8_t  ctrl_key (uint8_t idx);   /* idx = 1=PC2 / 2=PC3，返回 1 = **按下**（高电平有效，接 3.3V） */\n'
        'void     ctrl_key_tick (void);     /* \u2605 PC3 = 直接启动/停止按钮（脱机跑，不用连电脑）*/\n',
        'ctrl_f4.h decl')
wr(p, t)

# ---------- 2) ctrl_f4.c：接到控制节拍里 ----------
p = SRC + r'\ctrl_f4.c'
t = rd(p)
t = sub(t,
        '    ctrl_protect_tick();         /* 3) 堵转保护 + 按键重启 */\n',
        '    ctrl_protect_tick();         /* 3) 堵转保护 + 按键重启 */\n'
        '    ctrl_key_tick();             /* 3.5) \u2605 PC3 = 直接启动/停止（脱机跑，不用连电脑）*/\n',
        'ctrl_f4.c call')
wr(p, t)

# ---------- 3) io_f4.c：函数本体 ----------
p = SRC + r'\io_f4.c'
t = rd(p)
if 'ctrl_key_tick' in t:
    print('skip io_f4.c（已经有）')
else:
    body = '''
//===================================================★ PC3 = **直接启动按钮** ===================================================
//   用户要求："PC3 改为直接启动的按钮，我不想每次都要连上电脑才能跑"
//   行为（完全脱机，不需要上位机）：
//     · 按一下 PC3 ⇒ 闭环启动，目标转速 = CTRL_RUN_RPM（100 RPM），蜂鸣一声
//     · 再按一下 ⇒ 停车（目标 0 + 占空比 0），蜂鸣一声
//     · 如果当前处于"堵转停机"，按 PC3 先清堵转再启动（等于原来的重启功能）
//   按键消抖：连续两次采样间距 CTRL_PERIOD_MS，要求电平稳定 ≥2 拍（40ms）才认；上升沿视为按下。
//   ⚠️ PC2 保持原来的"堵转后重启"用途，没动。
//------------------------------------------------------------------------------------------------------------------
void ctrl_key_tick (void)
{
    static uint8_t  s_run    = 0U;      /* 1 = 按键启动的巡航状态 */
    static uint8_t  s_prev   = 0U;      /* 上一拍 PC3 电平 */
    static uint8_t  s_stable = 0U;      /* 稳定计数 */
    static uint8_t  s_beep   = 0U;      /* 蜂鸣剩余拍数 */

    uint8_t k = ctrl_key(2U);           /* PC3 */

    if (k != s_prev)
    {
        s_prev   = k;
        s_stable = 0U;
    }
    else if (s_stable < 2U)
    {
        s_stable ++;
    }

    /* 确认按下（上升沿 + 稳定）*/
    if ((1U == k) && (2U == s_stable))
    {
        if (0U == s_run)
        {
            ctrl_motor_clear_stall();           /* 堵转停机状态下也能一键起来 */
            ctrl_open_loop(0U);                 /* 闭环 */
            ctrl_speed_set_target((int16_t)CTRL_RUN_RPM);
            s_run = 1U;
        }
        else
        {
            ctrl_speed_set_target(0);
            ctrl_motor_set(1U, 0);
            s_run = 0U;
        }
        s_stable = 3U;                          /* 等松手，避免长按连续触发 */
        s_beep   = 8U;                          /* 约 160ms 蜂鸣 */
    }

    if (s_beep > 0U)
    {
        s_beep --;
        ctrl_buzzer(1U);
    }
    else
    {
        ctrl_buzzer(0U);
    }
}
'''
    t = t.rstrip('\n') + '\n' + body
    print('ok   io_f4.c 加函数')
wr(p, t)

# ---------- 4) FW 标签：2214 -> 2227（这是"2214 + PC3"的新构建，别和 2223 混）----------
for p, old, new in (
        (SRC + r'\app.c',     'FW=2214', 'FW=2227'),
        (SRC + r'\cmd_f4.c',  'FW=2214', 'FW=2227')):
    t = rd(p)
    if old not in t:
        print('WARN %s 里没找到 %s' % (p, old))
    else:
        t = t.replace(old, new)
        print('ok   %-22s %s -> %s' % (p.split('\\')[-1], old, new))
        wr(p, t)
print('done')
