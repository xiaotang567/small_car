/*********************************************************************************************************************
* 文件名              protect_f4.c
* 适用平台            STM32F407
* 说明                电机保护：全局堵转判据（转速 < 400 持续 2 秒 ⇒ 关电机）+ 按键重启
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"


/* ★★★ 2026-11-07（FW=2112）：**电机闭环巡航 + 防堵转 + PC2 重启** ★★★
   用户要求：
     · 原来的"开环占空比"改成 **闭环 500 RPM**；
     · 转速连续 **2 秒**低于目标（这里取目标的 80%）⇒ 判堵转，**停机**；
     · 停机后 **PC2 低电平** 重新起转；
     · OLED 显示 RUN / STALL PC2 / STOP。
   为什么用 80% 而不是"低于目标"：PI 稳态本来就允许一点误差（比如稳在 495），
   严格按"低于目标"会把正常工作误判成堵转。真堵转时转速基本是 0，用 80% 足够灵敏。*/

static uint8_t  s_motor_started = 0U;      /* 0 = 还没起过转 */
volatile uint8_t g_protect_enable = 0U;     /* ★ 2217：**台架测 DRV 模式 ⇒ 默认关**（正常跑车改成 1U）*/
static uint8_t  s_motor_stalled = 0U;      /* 1 = 堵转停机，等按键 */
static uint32_t s_stall_tick    = 0U;      /* ★ 2191：进堵转的时刻（远程重启要冷却 2 秒）*/
static int16_t  s_tgt_prev      = 0;       /* ★ 2194：上一拍的目转速（判"刚起转"）*/
static uint32_t s_run_tick      = 0U;      /* ★ 2194：刚起转的时刻 */
#if CTRL_STALL_ENABLE
static uint16_t s_below_cnt     = 0U;      /* 连续低于阈值拍数 */
#endif
static uint8_t  s_btn_armed     = 0U;      /* 进堵转时已记住按键电平 */
static uint8_t  s_key_prev      = 0U;      /* 进堵转那一刻的按键电平（PC2/PC3）*/

void ctrl_protect_tick (void)
{
    /* ① 自动起转（仅闭环、只做一次）*/
    if ((0U == ctrl_is_open_loop()) && (0U == s_motor_started))
    {
        s_motor_started = 1U;
#if CTRL_RUN_AUTO_START
        ctrl_speed_set_target((int16_t)CTRL_RUN_RPM);
#endif
    }

    /* ② 堵转停机后：**任意按键（PC2 / PC3）新按下** ⇒ 回闭环重新起转
       ★ 2026-11-07（FW=2121）：实测遥测 `k1` **242 条全是 0** —— PC2 一直是低电平
         （按键卡住 / 常闭 / 板上有下拉电阻），"先松开再按下"的边沿永远等不到
         ⇒ 按 PC2 毫无反应。改成：
           · 进堵转时记住当时的电平；
           · 之后只要有任意一次电平变化、并且变成"有键按下"就重启；
           · PC2 卡死不动也没关系，按 PC3 一样能重启。*/
    if (0U != s_motor_stalled)
    {
        uint8_t k = (uint8_t)(((0U != ctrl_key(1U)) ? 1U : 0U) |
                              ((0U != ctrl_key(2U)) ? 2U : 0U));

        if (0U == s_btn_armed)
        {
            s_key_prev  = k;          /* 刚进堵转：记住当前电平（卡住的键不会误触发）*/
            s_btn_armed = 1U;
        }
        else if (k != s_key_prev)
        {
            s_key_prev = k;
            if (0U != k)              /* 变成"有键按下" ⇒ 重启 */
            {
                s_motor_stalled = 0U;
                s_btn_armed     = 0U;
#if CTRL_STALL_ENABLE
                s_below_cnt     = 0U;
#endif
                ctrl_open_loop(0U);                                /* 回闭环巡航 */
                ctrl_speed_set_target((int16_t)CTRL_RUN_RPM);
            }
        }
        return;
    }
    s_btn_armed = 0U;                 /* 没堵转时复位，下次堵转重新记住电平 */

    /* ③ ★★★ 全局堵转判据（FW=2120，用户要求，**不再分模式/不再看目标**）★★★
          规则只有一条：**实测转速 < CTRL_STALL_RPM(400) 持续 CTRL_STALL_MS(2 秒) ⇒ 关电机**
          · 闭环 500、开环占空比、甚至上位机没连 —— 全都一样判；
          · 唯一例外：电机本来就没驱动（占空比 0 且目标 0）⇒ 那就是"停着的"，不算堵转；
          · 关掉电机后进入 STALL 状态，等 PC2（先松开再按下）重新起转。*/
#if CTRL_STALL_ENABLE
    /* ★★ 2212：**保护总开关 + 开环模式自动不判堵转** ★★
       用户要拿 40% 占空比测新的 DRV888 驱动 —— 那种情况下电机可能转不起来/转速读数不对，
       堵转保护会立刻把占空比清零，反而妨碍测试。所以：
         · `g_protect_enable = 0`（命令 `pf 0` / 界面按钮）⇒ 整段跳过，完全不保护；
         · 开环模式（`w` 命令给的占空比）⇒ 也不判堵转（开环本来就没有"转速闭环"这回事）。
       ⚠️ 保护关掉以后，堵转时电机会一直给占空比 ⇒ **注意发热/别烧电机**。*/
    if (0U == g_protect_enable)
    {
        s_below_cnt = 0U;
        return;
    }
    if (0U != ctrl_is_open_loop())
    {
        s_below_cnt = 0U;
        return;
    }

    {
        int16_t duty  = ctrl_motor_get((uint8_t)CTRL_SPEED_MOTOR);
        int16_t tgt   = ctrl_speed_get_target();
        int16_t rpm   = ctrl_speed_get_rpm();
        int32_t a_rpm = (rpm < 0) ? -(int32_t)rpm : (int32_t)rpm;

        /* 根本没在驱动 ⇒ 不算堵转（这就是"已经停了"）*/
        if ((0 == duty) && (0 == tgt))
        {
            s_below_cnt = 0U;
            return;
        }

        /* ★★ 2194：**起转宽限**——目标由 0 变非 0 后 2.5 秒内不判堵转
           （低速起转时电机要 1~2 秒才转起来，之前 3 秒判据直接把它杀了）*/
        if ((0 != tgt) && (0 == s_tgt_prev)) { s_run_tick = HAL_GetTick(); }
        s_tgt_prev = tgt;

        if ((0 != tgt) && ((HAL_GetTick() - s_run_tick) < (uint32_t)CTRL_STALL_GRACE_MS))
        {
            s_below_cnt = 0U;
            return;
        }

        if (a_rpm < (int32_t)CTRL_STALL_RPM)
        {
            if (s_below_cnt < 0xFFFFU) { s_below_cnt ++; }

            if (s_below_cnt >= (uint16_t)CTRL_STALL_TICKS)
            {
                s_motor_stalled = 1U;
                s_stall_tick    = HAL_GetTick();      /* ★ 2191 */
                s_below_cnt     = 0U;
                ctrl_open_loop(0U);                             /* 退回闭环，便于 PC2 重启 */
                ctrl_speed_set_target(0);                       /* ★ 关电机 */
                ctrl_motor_set((uint8_t)CTRL_SPEED_MOTOR, 0);   /* 开环时也确保真的关 */
            }
        }
        else
        {
            s_below_cnt = 0U;
        }
    }
#endif  /* CTRL_STALL_ENABLE */
}

/* ★ 2026-11-07（FW=2121）：上位机发 `q <非零>` 时清掉"堵转停机"状态 ——
   这样即使按键坏了/没接，也能从面板上把电机重新起转。
   ★★ 2191：**冷却 2 秒** —— 实测上位机的 `q` 因为 ACK 丢失会自动重发（每 250ms），
      每次重发都会把堵转状态清掉 ⇒ 堵转保护形同虚设（电机 44% 占空比、0 转速一直转）。
      现在：进入堵转后 2 秒内，远程 `q` 不允许清除，必须在面板上再点一次（2 秒后）。*/
void ctrl_motor_clear_stall (void)
{
    if ((0U != s_motor_stalled) && ((HAL_GetTick() - s_stall_tick) < 2000U))
    {
        return;                                  /* 冷却中：不让重发把保护清掉 */
    }

    s_motor_stalled = 0U;
    s_btn_armed     = 0U;
#if CTRL_STALL_ENABLE
    s_below_cnt     = 0U;
#endif
}

uint8_t ctrl_motor_stalled (void) { return s_motor_stalled; }
uint8_t ctrl_motor_started (void) { return s_motor_started; }
