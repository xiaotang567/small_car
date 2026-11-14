# -*- coding: utf-8 -*-
"""2228：**不连电脑也能跑**（脱机运行）
   1) 开机不再死等 WiFi：连热点/连上位机的等待各压到 1.5 秒，连不上就直接进主循环；
   2) 主循环里"整模块硬复位重连"只在**车停着**的时候做；车在跑就完全不碰 WiFi，
      免得一次 50 秒的等待把控制循环按死；
   3) 删掉"链路断就停车"（wf_ready=0 时车会被强制停 ⇒ 脱机根本跑不起来）。
"""
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
        print('FAIL %-34s hits=%d' % (tag, t.count(old)))
        sys.exit(1)
    print('ok   %-34s' % tag)
    return t.replace(old, new)

# ================= 1) wifi_f4.c：开机等待改短 =================
p = SRC + r'\wifi_f4.c'
t = rd(p)

t = sub(t,
        '#define WF_TO_SOCKET            50000       // 50 秒   连 TCP 服务器\n',
        '#define WF_TO_SOCKET            50000       // 50 秒   连 TCP 服务器（重连里不用，见下）\n'
        '\n'
        '/* \u2605\u2605 2228：**脱机跑**（用户要求"不连接也能跑"）\n'
        '   原来开机要死等：连热点 10 秒 + 连 TCP 服务器 50 秒 = 最长 60 秒，\n'
        '   上位机没开/热点没开的时候，车在这 60 秒里完全不响应按键，等于开不了机。\n'
        '   \u21d2 开机这两步各只等 1.5 秒，连不上就返回失败，主循环照常跑（车能按键启动）；\n'
        '     真要连上位机，停下来以后主循环会自己重连（见 app.c）。*/\n'
        '#define WF_TO_WIFI_BOOT         1500U       // 开机：设热点信息最多等 1.5 秒\n'
        '#define WF_TO_SOCKET_BOOT       1500U       // 开机：连 TCP 服务器最多等 1.5 秒\n',
        'wifi_f4.c 加开机等待宏')

t = sub(t,
        'static uint8_t wf_wifi_connect (const char *ssid, const char *password)\n',
        'static uint8_t wf_wifi_connect_to (const char *ssid, const char *password, uint32_t wait_ms)\n',
        'wifi_f4.c wf_wifi_connect 带超时')

t = sub(t,
        '    if (wf_set_parameter(WF_CMD_SET_WIFI, buf, len, WF_TO_WIFI)) return 1;\n'
        '    wf_get_ip();\n'
        '    return 0;\n'
        '}\n',
        '    if (wf_set_parameter(WF_CMD_SET_WIFI, buf, len, wait_ms)) return 1;\n'
        '    wf_get_ip();\n'
        '    return 0;\n'
        '}\n',
        'wifi_f4.c 用传入的超时')

# 开机用短等待
t = sub(t,
        '    if (wf_wifi_connect(WIFI_SSID, WIFI_PASSWORD)) { wf_last_err = 2; return 2; }\n',
        '    /* \u2605 2228：开机只等 1.5 秒（原来是 10 秒）——连不上就先跑起来 */\n'
        '    if (wf_wifi_connect_to(WIFI_SSID, WIFI_PASSWORD, WF_TO_WIFI_BOOT))\n'
        '    { wf_last_err = 2; return 2; }\n',
        'wifi_f4.c 开机连热点 1.5s')

t = sub(t,
        '                          wf_pc_ip, WIFI_PC_PORT, WIFI_LOCAL_PORT, WF_TO_SOCKET))\n',
        '                          wf_pc_ip, WIFI_PC_PORT, WIFI_LOCAL_PORT, WF_TO_SOCKET_BOOT))\n',
        'wifi_f4.c 开机连上位机 1.5s')
wr(p, t)

# ================= 2) app.c：车在跑就不碰 WiFi =================
p = SRC + r'\app.c'
t = rd(p)
t = sub(t,
        '    if (!wifi_f4_is_ready())\n'
        '    {\n'
        '      if ((HAL_GetTick() - t_retry) > 3000)\n',
        '    /* \u2605\u2605 2228：**车在跑的时候完全不碰 WiFi**（脱机跑的关键）\n'
        '       重连里最坏要走"整模块硬复位 + 连热点 + 连上位机"，那是几十秒的阻塞；\n'
        '       车正在跑的时候被按住几十秒 = 速度环/堵转保护全停 = 车失控。\n'
        '       \u21d2 只有在**停机状态**（目标转速 0 且不是开环）才去连上位机；\n'
        '         停机时连上以后，按 PC3 启动就能边跑边看图/调参，两不误。*/\n'
        '    if ((0U == ctrl_speed_get_target()) && (0U == ctrl_is_open_loop()))\n'
        '    {\n'
        '      if (!wifi_f4_is_ready() && ((HAL_GetTick() - t_retry) > 3000))\n',
        'app.c 重连加"停机才连"闸门')

# 原来 else { relink_n = 0; } 的配对：把结构补完整
t = sub(t,
        '    else\n'
        '    {\n'
        '      relink_n = 0;                             /* 连上了就清计数 */\n'
        '    }\n',
        '      if (wifi_f4_is_ready())\n'
        '      {\n'
        '        relink_n = 0;                           /* 连上了就清计数 */\n'
        '      }\n'
        '    }\n'
        '    else if (wifi_f4_is_ready())\n'
        '    {\n'
        '      relink_n = 0;\n'
        '    }\n',
        'app.c 配对收尾')
wr(p, t)

# ================= 3) pid_f4.c：删掉"断链就停车" =================
p = SRC + r'\pid_f4.c'
t = rd(p)
import re
i = t.find('if ((0U == wifi_f4_is_ready()) && (0U != g_protect_enable))')
if i < 0:
    print('FAIL pid_f4.c 断链停车没找到'); sys.exit(1)
# 找到这条 if 语句的整块（到匹配的 } 为止）
j = t.find('{', i)
depth = 0
k = j
while k < len(t):
    if t[k] == '{':
        depth += 1
    elif t[k] == '}':
        depth -= 1
        if depth == 0:
            break
    k += 1
old_block = t[i:k + 1]
new_block = ('/* \u2605\u2605 2228：**删掉"断链强制停车"**（用户要求"不连接也能跑"）\n'
             '       原来 wf_ready=0（没连上位机 / 没连热点）时这里会把目标转速清 0 \u21d2\n'
             '       脱机根本没有上位机，等于按键启动后立刻被自己停下。\n'
             '       \u21d2 现在链路状态和车的运行完全解耦：断链不停车（想恢复就把下面 #if 改 1）。*/\n'
             '#define SC_STOP_ON_LINK_LOST   0\n'
             '#if (SC_STOP_ON_LINK_LOST != 0)\n'
             + old_block +
             '\n#endif')
t = t[:i] + new_block + t[k + 1:]
print('ok   pid_f4.c 删断链停车')
wr(p, t)

# ================= 4) FW 标签 2227 -> 2228 =================
for p in (SRC + r'\app.c', SRC + r'\cmd_f4.c'):
    t = rd(p)
    if 'FW=2227' not in t:
        print('WARN %s 没找到 FW=2227' % p)
    else:
        wr(p, t.replace('FW=2227', 'FW=2228'))
        print('ok   %-20s FW -> 2228' % p.split('\\')[-1])
print('done')
