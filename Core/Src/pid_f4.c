/*********************************************************************************************************************
* 文件名              pid_f4.c
* 适用平台            STM32F407
* 说明                速度环 PID（PI + 比例前馈 + 设定值斜坡 + 极性自动识别）+ 开环/闭环切换
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"
#include "wifi_f4.h"          /* 断链强制停车要用 wifi_f4_is_ready() */
#include "board_f4.h"         /* ★ 2211：弯道占空比上限要用 cam_curved */


//===================================================★ 速度环（PI + 比例前馈 + 斜坡，参数照抄原工程）===================================================
#define SC_RAMP_STEP_X10    ((int32_t)CTRL_SPEED_RAMP_RPM_S * CTRL_PERIOD_MS / 100)   /* 3000*20/100 = 600 */

static int32_t g_sc_target_x10 = 0;
static int32_t g_sc_ramp_x10   = 0;
static int32_t g_sc_i_acc      = 0;
static int16_t g_sc_duty       = 0;
static int16_t g_sc_err_rpm    = 0;
static uint8_t g_sc_state      = 0U;
static int8_t  g_sc_sign       = 1;
static uint8_t g_sc_sign_locked = 0U;
static uint8_t g_sc_was_active  = 0U;      /* 上一拍速度环是不是在出力（用来只清一次电机） */

void ctrl_speed_init (void)
{
    g_sc_target_x10  = 0;
    g_sc_ramp_x10    = 0;
    g_sc_i_acc       = 0;
    g_sc_duty        = 0;
    g_sc_err_rpm     = 0;
    g_sc_state       = 0U;
    g_sc_sign        = 1;
    g_sc_sign_locked = 0U;
    ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, 0);
}

void ctrl_speed_set_target (int16_t rpm)
{
    g_sc_target_x10 = (int32_t)rpm * 10;
    g_sc_i_acc      = 0;                  /* 换目标就清积分（原工程同） */
}

int16_t ctrl_speed_get_target (void) { return (int16_t)(g_sc_target_x10 / 10); }
int16_t ctrl_speed_get_duty   (void) { return g_sc_duty; }
int16_t ctrl_speed_get_error  (void) { return g_sc_err_rpm; }
int16_t ctrl_speed_get_rpm    (void) { return (int16_t)(ctrl_encoder_rpm_x10() / 10); }
uint8_t ctrl_speed_get_state  (void) { return g_sc_state; }

const char * ctrl_speed_state_name (void)
{
    switch (g_sc_state)
    {
        case 0U: return "STOP";
        case 1U: return "RAMP";
        case 2U: return "RUN ";
        default: return "FAULT";
    }
}

/* ★★★ 2026-11-07（FW=2119）：**开环 / 闭环切换** ★★★
   现象：上位机用"占空比模式"发 `w 300 0`，电机没反应。
   原因：固件上电就自动闭环跑 500RPM，`speed_tick()` **每 20ms 都会写一次占空比**
         ⇒ 你发的值下一个节拍就被覆盖了。
   做法：收到 `w` ⇒ 进开环，速度环直接让位（什么都不写），电机只认你给的占空比；
        收到 `q`（含 `q 0` 停止）⇒ 回闭环。
   顺便：开环时堵转保护也不参与（没目标转速可比）。*/
static uint8_t g_open_loop = 0U;

void ctrl_open_loop (uint8_t on)
{
    g_open_loop = (0U != on) ? 1U : 0U;
}

uint8_t ctrl_is_open_loop (void) { return g_open_loop; }

void ctrl_speed_tick (void)
{
    int32_t meas_x10, err_x10, p_term, i_step, ff_term, out;

    if (0U != g_open_loop) { return; }      /* ★ 开环模式：速度环完全让位 */

    /* ★★★ 安全：**断链强制停车** ★★★
       上位机一关（或 WiFi 掉线），板子不知道，会按最后一次目标转速一直跑 —— 很危险。
       `wifi_f4_is_ready()` 在链路断开/发送失败时会被驱动清 0（遥测 100ms 一次，
       所以最多几百 ms 就能发现）⇒ 这里直接把目标清 0，走下面的停机分支。*/
    if (0U == wifi_f4_is_ready())
    {
        g_sc_target_x10 = 0;
    }

    /* 目标 0 ⇒ 停机（原工程同）。
       ⚠️ 只在"从出力变回停机"的那一拍写一次 0：否则每 20ms 都写一遍，
          会把 `w` 命令开环给的占空比立刻冲掉，没法做开环测试。 */
    if (0 == g_sc_target_x10)
    {
        g_sc_ramp_x10 = 0;
        g_sc_i_acc    = 0;
        g_sc_duty     = 0;
        g_sc_err_rpm  = 0;
        g_sc_state    = 0U;
        if (0U != g_sc_was_active)
        {
            g_sc_was_active = 0U;
            ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, 0);
        }
        return;
    }
    g_sc_was_active = 1U;

    /* 设定值斜坡 */
    if (g_sc_ramp_x10 < g_sc_target_x10)
    {
        g_sc_ramp_x10 += SC_RAMP_STEP_X10;
        if (g_sc_ramp_x10 > g_sc_target_x10) { g_sc_ramp_x10 = g_sc_target_x10; }
    }
    else if (g_sc_ramp_x10 > g_sc_target_x10)
    {
        g_sc_ramp_x10 -= SC_RAMP_STEP_X10;
        if (g_sc_ramp_x10 < g_sc_target_x10) { g_sc_ramp_x10 = g_sc_target_x10; }
    }

    meas_x10 = ctrl_encoder_rpm_x10();

    /* ★★★ 2026-11-07（FW=2117）：**测速一阶低通** ★★★
       日志实测：占空比很稳（2824~3212），但 rpm 在 466~526 之间慢慢摆（周期约 2s）。
       20ms 窗口里编码器量化的 ±1 个计数 ≈ ±7RPM，会被 P 项放大成可见的摆动。
       这里做 y += (x-y)/4（时间常数约 4 拍 = 80ms）：对真实加减速几乎无延迟，
       但把量化噪声压掉大半，速度环就不会再"追噪声"。*/
    {
        static int32_t meas_f_x10 = 0;
        meas_f_x10 += (meas_x10 - meas_f_x10) / 4;
        meas_x10 = meas_f_x10;
    }

#if CTRL_SPEED_SIGN_AUTO
    /* 反馈极性自动识别：上一拍给了明显正占空比，实测却是明显负转速 ⇒ 反了 */
    if ((0U == g_sc_sign_locked) && (g_sc_duty > 800))
    {
        if (meas_x10 < -2000)      { g_sc_sign = -1; g_sc_sign_locked = 1U; }
        else if (meas_x10 > 2000)  { g_sc_sign =  1; g_sc_sign_locked = 1U; }
    }
#endif
    meas_x10 *= (int32_t)g_sc_sign;

    err_x10    = g_sc_ramp_x10 - meas_x10;
    g_sc_err_rpm = (int16_t)(err_x10 / 10);

    p_term = (err_x10 * CTRL_SPEED_KP_X1000) / 10000;
    i_step = (err_x10 * CTRL_SPEED_KI_X1000) / 10000;

    g_sc_i_acc += i_step;
    if (g_sc_i_acc >  CTRL_SPEED_I_LIMIT) { g_sc_i_acc =  CTRL_SPEED_I_LIMIT; }
    if (g_sc_i_acc < -CTRL_SPEED_I_LIMIT) { g_sc_i_acc = -CTRL_SPEED_I_LIMIT; }

    /* ★ 2193：前馈加"死区补偿"。纯比例前馈在低速时给出的占空比太小
       （100RPM ⇒ 5%），电机有静摩擦/齿槽，5% 根本转不动。
       加一个常量偏置：只要在跑就给 FF_OFFSET。*/
    ff_term = (g_sc_ramp_x10 * CTRL_SPEED_FF_PER_1000) / 10000;
    if (g_sc_ramp_x10 > 0)      { ff_term += CTRL_SPEED_FF_OFFSET; }
    else if (g_sc_ramp_x10 < 0) { ff_term -= CTRL_SPEED_FF_OFFSET; }

    out = ff_term + p_term + g_sc_i_acc;   /* ← 2211 我一度把这一行漏掉了（编译警告 p_term 未使用），已补回 */

    /* ★★ 2211：**弯道把占空比上限放开到 80%**（直道仍 50%）
       实测（11-09 17:5x）：直道 d1≈2000~2500（20~25%），弯道里 d1=5000 顶死、转速掉到 26~65（目标 100）
       ⇒ 弯里阻力大，50% 的上限卡住了速度环 ⇒ 这就是"转弯减速"的原因。*/
    {
        int32_t dmax = (0U != cam_curved) ? (int32_t)CTRL_SPEED_DUTY_MAX_CUR : (int32_t)CTRL_SPEED_DUTY_MAX;

        if (out > dmax) { out = dmax; }
        if (out < 0)    { out = 0; }
    }
    /* ★★ 2195：**起转踹一脚 —— 只在开始那一下**
       用户要求："踹只能开始的时候踹"。
       所以不再按"转速低就踹"，而是：
         · 目标由 0 变非 0 的那一刻记下时间；
         · 之后 **CTRL_SPEED_KICK_WIN_MS（800ms）** 之内，占空比至少给 KICK_DUTY(40%)；
         · 过了这个窗口就不再踹（跑起来以后速度掉下去也不踹，交给 PID 和堵转保护）。
        目标回到 0 时复位，下次启动重新踹。*/
    {
        static uint32_t s_kick_t    = 0U;
        static uint8_t  s_kick_act  = 0U;
        static int16_t  s_kick_tgt  = 0;

        if (0 == g_sc_ramp_x10)
        {
            s_kick_act = 0U;                       /* 停机 ⇒ 复位，下次启动重新踹 */
            s_kick_t   = 0U;
        }
        else
        {
            if ((0 != s_kick_t) && (0 == s_kick_tgt)) { s_kick_t = HAL_GetTick(); s_kick_act = 1U; }
            if (0U == s_kick_t) { s_kick_t = HAL_GetTick(); s_kick_act = 1U; }

            if ((0U != s_kick_act) &&
                ((HAL_GetTick() - s_kick_t) < (uint32_t)CTRL_SPEED_KICK_WIN_MS))
            {
                if (out < CTRL_SPEED_KICK_DUTY) { out = CTRL_SPEED_KICK_DUTY; }
            }
            else
            {
                s_kick_act = 0U;                   /* 窗口过了：以后不再踹 */
            }
        }
        s_kick_tgt = (int16_t)(g_sc_ramp_x10 / 10);

        /* 踹完再夹一次上限（KICK_DUTY 可能比 DUTY_MAX 大，比如 40% vs 50% 现在没超，
           但这样写以后改参数也不会把限幅突破）*/
        if (out > CTRL_SPEED_DUTY_MAX) { out = CTRL_SPEED_DUTY_MAX; }
        if (out < 0)                   { out = 0; }
    }

    g_sc_duty = (int16_t)out;
    ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, g_sc_duty);

    g_sc_state = (g_sc_ramp_x10 < g_sc_target_x10) ? 1U : 2U;
}
