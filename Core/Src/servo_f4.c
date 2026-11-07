/*********************************************************************************************************************
* 文件名              servo_f4.c
* 适用平台            STM32F407
* 说明                幻尔 HX 总线舵机（UART4，1Mbps，同步轮询、不用中断）
*
* ★ 2026-11-07：从 ctrl_f4.c 拆出来的独立模块（用户要求按功能分文件）。
********************************************************************************************************************/

#include "ctrl_f4.h"


//===================================================★ 幻尔总线舵机（UART4，1Mbps，同步轮询）===================================================
//   ★★★ 2026-11-07（FW=2103）：**按 MSPM0 原工程 user/src/hx_servo.c 的写法整体重写** ★★★
//
//   为什么换掉原来那版（中断 + 环形缓冲 + 主循环状态机）：
//     把 PC10(TX) 和 PC11(RX) 一并起来（收发短接）后，板子会**把自己发的每一帧都收回来**
//     ⇒ RXNE 中断不断；而 2098 以前 SysTick 是最低优先级(15)、UART4 是 6
//     ⇒ 时间基准被饿死 ⇒ 所有 `while (HAL_GetTick() - t0 < N)` 永不超时 ⇒ 板子卡死。
//   原工程的做法**根本不用 UART 中断**：
//     · 发一帧 → **立刻轮询**把应答收回来（非阻塞查 RXNE）
//     · 超时用**循环次数**数出来（SERVO_RX_WAIT_LOOPS），和 HAL_GetTick 无关
//     · 收到的帧和"刚发出去那帧"一模一样 ⇒ 判定为回显 ⇒ 丢掉，继续等真正的应答
//   ⇒ "中断风暴"这条路从结构上消失；并把 PC10/PC11 短接，最多只是读不到舵机，不会卡板子。
//
//   帧格式： FF FF ID LEN INSTR P1..Pn CHK
//            LEN = 参数个数 + 2
//            CHK = ~(ID + LEN + INSTR + P1..Pn) & 0xFF
//   写位置： INSTR=0x03  addr=0x2A  2 字节小端（0~4095）
//   读位置： INSTR=0x02  addr=0x38  2 字节
//   速度  ： addr=0x2E，符号-幅值（负数 = 最高位 BIT15 置 1，不是补码）
//===================================================

#define SERVO_TX_BUF_MAX        (32U)       /* 发送缓冲上限 */
#define SERVO_RX_BUF_MAX        (32U)       /* 接收缓冲上限（LEN 最大 32）*/
#define SERVO_PARAM_MAX         (24U)       /* 单帧最多几个参数 */
#define SERVO_RX_GUARD_MAX      (64U)       /* 找包头时最多空转几轮，防死等 */

/* ★ 等一个字节要空转多少圈（**不用 HAL_GetTick**，和时间基准无关）
   168MHz 下 40000 圈约 1~2ms；1Mbps 一个字节只要 10µs，绰绰有余。
   没应答时也就是白等这一两毫秒，绝不卡死。*/
#define SERVO_RX_WAIT_LOOPS     (40000U)
/* TX 等 TXE 的圈数上限（防止 UART 没使能时死等）*/
#define SERVO_TX_WAIT_LOOPS     (100000U)

/* ---- 幻尔 HX 系列：指令码 / 寄存器地址 ---- */
#define SERVO_INST_PING         (0x01U)
#define SERVO_INST_READ         (0x02U)
#define SERVO_INST_WRITE        (0x03U)
#define SERVO_REG_MODE          (0x21U)
#define SERVO_REG_TORQUE        (0x28U)
#define SERVO_REG_GOAL_POS      (0x2AU)
#define SERVO_REG_GOAL_SPEED    (0x2EU)
#define SERVO_REG_CURPOS        (0x38U)

static uint8_t  g_servo_id       = CTRL_SERVO_ID;
static uint8_t  g_servo_err      = 0U;
static uint16_t g_servo_pos      = 0xFFFFU;         /* 最近一次**读到**的位置（0xFFFF=还没读到）*/
static uint16_t g_servo_target   = CTRL_SERVO_POS_CENTER;
static uint16_t g_servo_last_set = 0xFFFFU;          /* 0xFFFF = 还没下发过 ⇒ 开机第一次节拍会摆到中位 */
static uint8_t  g_servo_boot     = 0U;
static uint8_t  g_servo_speed_ok = 0U;
static uint8_t  g_servo_half     = 0U;              /* 兼容旧接口；同步轮询下不再切引脚 */
static uint32_t g_servo_baud     = CTRL_SERVO_BAUD;

static uint32_t g_srv_tx_bytes = 0U;
static uint32_t g_srv_rx_bytes = 0U;
static uint32_t g_srv_rx_bad   = 0U;
static uint32_t g_srv_tx_cnt   = 0U;
static uint32_t g_srv_rx_cnt   = 0U;

/* 刚发出去的那一帧（用来识别总线回显）*/
static uint8_t  g_srv_last_tx[SERVO_TX_BUF_MAX];
static uint8_t  g_srv_last_tx_len = 0U;

static uint8_t servo_checksum (const uint8_t *p, uint8_t n)
{
    uint32_t sum = 0U;
    uint8_t  i;

    for (i = 0U; i < n; i ++) { sum += (uint32_t)p[i]; }
    return (uint8_t)((~sum) & 0xFFU);
}

/* 改波特率 —— ★★★ 2026-11-07（FW=2108）：**这里原来算错了 16 倍，舵机一直没通过话** ★★★
   错的写法（沿用了很久）：
       div = fCK / bps;              // 42MHz / 1M = 42，以为这是"整数分频"
       UART4->BRR = div << 4;        // → 672
   实际上 OVER8=0 时：
       波特率 = fCK / (16 × USARTDIV)     ⇒   USARTDIV = fCK/(16×bps)
       而 BRR  = USARTDIV × 16           ⇒   **BRR = fCK / bps**（整数就是完整的 BRR，不用再移 4 位！）
   ⇒ 旧写法把 BRR 写成 672 ⇒ USARTDIV = 42 ⇒ 实际波特率 = 42e6/(16×42) = **62500**。
      1Mbps 的舵机收到的是一串 62.5k 的乱码，当然一辈子不应答；
      而 PC10/PC11 回环测试照样通（收发两边一样错）。
   正确：BRR = round(fCK / bps)。1M → 42 (=0x2A)，115200 → 365。两者都 ≤ 65535，
   所以根本用不着 OVER8（BRR 最大 65535 ⇒ 最低约 641bps）。*/
static void servo_set_baud (uint32_t bps)
{
    uint32_t brr;

    if (0U == bps) { return; }

    brr = (CTRL_APB1_HZ_LOCAL + (bps / 2U)) / bps;   /* ★ BRR 本身 = fCK/baud（OVER8=0）*/
    if (0U == brr) { brr = 1U; }

    UART4->CR1 &= (uint16_t)(~USART_CR1_UE);
    UART4->CR1 &= (uint16_t)(~USART_CR1_OVER8);      /* 16 倍过采样 */
    UART4->BRR  = (uint16_t)(brr & 0xFFFFU);         /* ★ 不再 << 4 */
    UART4->CR1 |= USART_CR1_UE;
    UART4->CR1 |= (uint16_t)(USART_CR1_TE | USART_CR1_RE);
    g_servo_baud = bps;
}

/* 非阻塞取一个字节：有数据返回 0，没有返回 1（只碰硬件寄存器） */
static uint8_t servo_hw_byte (uint8_t *d)
{
    if (0U != (UART4->SR & USART_SR_RXNE))
    {
        *d = (uint8_t)(UART4->DR & 0xFFU);       /* 读 DR：同时清 ORE/FE/NE/PE */
        g_srv_rx_bytes ++;
        return 0U;
    }
    return 1U;
}

/* ★★★ 2026-11-07（FW=2105）：**软件 FIFO** ★★★
   现场：把 PC10(TX) 和 PC11(RX) 短接做回环，结果是"发 1644B / 只收 549B"。
   原因不是引脚坏，而是 **F4 的 UART 只有 1 个字节的接收寄存器**：
   1Mbps 下字节间隔只有 **10µs**，而同步驱动在发送时 CPU 一直盯着 TXE 位、
   没空读 RX ⇒ 回显在发送过程中就一个个 **溢出（ORE）丢掉了**。
   （这跟 MSPM0 原工程注释里说的"要手动打开 FIFO，否则 1 个字节缓冲接不住"是同一件事。）
   ⇒ 这里补一个 64 字节软件 FIFO，并规定：**任何等待循环里都要顺手把硬件 RDR 搬进 FIFO**。
      这样"等 TXE 的那 10µs"也在收字节，回环就能发多少收多少。
   ⚠️ 真正的舵机是"收完整帧再回话"，本来不会和发送重叠 ⇒ 这条对真机没有副作用。*/
#define SERVO_FIFO_SIZE     (64U)

static uint8_t  g_srv_fifo[SERVO_FIFO_SIZE];
static uint16_t g_srv_fifo_head = 0U;
static uint16_t g_srv_fifo_tail = 0U;

/* 把硬件 RDR 里现有的字节全部搬进 FIFO（不等待、不阻塞）*/
static void servo_rx_pump (void)
{
    uint8_t  d;
    uint16_t next;

    while (0U == servo_hw_byte(&d))
    {
        next = (uint16_t)((g_srv_fifo_head + 1U) % SERVO_FIFO_SIZE);
        if (next == g_srv_fifo_tail) { return; }         /* FIFO 满：丢（和硬件溢出等效）*/
        g_srv_fifo[g_srv_fifo_head] = d;
        g_srv_fifo_head = next;
    }
}

/* 从 FIFO 里取一个字节：有返回 0，没有返回 1 */
static uint8_t servo_fifo_get (uint8_t *d)
{
    if (g_srv_fifo_tail == g_srv_fifo_head) { return 1U; }
    *d = g_srv_fifo[g_srv_fifo_tail];
    g_srv_fifo_tail = (uint16_t)((g_srv_fifo_tail + 1U) % SERVO_FIFO_SIZE);
    return 0U;
}

/* 读一个字节，**自己数圈数超时**（原工程 hx_rx_byte 的写法）
   ★ 等待期间必须不停 servo_rx_pump()，否则 10µs 就溢出了 */
static uint8_t servo_rx_byte (uint8_t *d)
{
    uint32_t guard = SERVO_RX_WAIT_LOOPS;

    while (guard --)
    {
        servo_rx_pump();                        /* ★ 每圈都把硬件 RDR 搬进 FIFO */
        if (0U == servo_fifo_get(d)) { return 0U; }
    }
    return 1U;
}

/* 把 RX 里残留的字节清掉（发之前清一下，别把上一帧的尾巴当成应答）*/
static void servo_rx_flush (void)
{
    uint8_t d;
    uint8_t n = 0U;

    servo_rx_pump();                            /* 先把硬件里的搬进 FIFO */
    while ((n < 200U) && (0U == servo_fifo_get(&d))) { n ++; }
    g_srv_fifo_head = 0U;
    g_srv_fifo_tail = 0U;
}

/* ★ 总线方向控制（单线半双工；对应老版本里 TC 中断里那段"放开 PC10"）
   PC10 是 UART4_TX，MODER 的 [21:20] 两位：10 = 复用推挽，00 = 输入(高阻)。
   推挽的时候总线由板子驱动；放开以后靠外部上拉（和 PC11 的内部上拉）维持空闲高，
   舵机才有机会把线拉低回话。*/
static void servo_bus_drive (void)
{
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << 20)) | (2UL << 20);   /* 推挽 AF */
}

static void servo_bus_release (void)
{
    GPIOC->MODER &= ~(3UL << 20);                                  /* 高阻输入 */
}

/* 组一帧发出去（原工程 hx_tx_frame）*/
static void servo_tx_frame (uint8_t id, uint8_t inst, const uint8_t *params, uint8_t n)
{
    uint8_t buf[SERVO_TX_BUF_MAX];
    uint8_t i;

    buf[0] = 0xFFU;
    buf[1] = 0xFFU;
    buf[2] = id;
    buf[3] = (uint8_t)(n + 2U);
    buf[4] = inst;

    for (i = 0U; i < n; i ++) { buf[5U + i] = params[i]; }

    buf[5U + n] = servo_checksum(&buf[2], (uint8_t)(n + 3U));

    /* 存一份，供接收端识别回显 */
    g_srv_last_tx_len = (uint8_t)(6U + n);
    for (i = 0U; i < g_srv_last_tx_len; i ++) { g_srv_last_tx[i] = buf[i]; }

    servo_rx_flush();

    /* ★★★ 2026-11-07（FW=2106）：**单线总线：说话之前先把 PC10 切回推挽 AF** ★★★
       官方的 F103 例程能直接推挽通，是因为中间有幻尔的**调试板**（半双工转换在那里做）。
       我们是把舵机直接并在 PC10/PC11 上：如果不主动释放，PC10 会一直推挽拉高，
       舵机想回话时就被顶住 ⇒ 永远收不到应答。*/
    if (0U != g_servo_half) { servo_bus_drive(); }

    for (i = 0U; i < g_srv_last_tx_len; i ++)
    {
        uint32_t guard = SERVO_TX_WAIT_LOOPS;

        /* ★★★ 2026-11-07（FW=2105）：**等 TXE 的空隙里也要搬 RX** ★★★
           1Mbps 下一个字节只有 10µs，而 F4 的接收寄存器只有 1 个字节。
           如果发送期间只顾着盯 TXE，回显就会一个个溢出丢掉 ——
           现场"发 1644B / 只收 549B"就是这个。这里顺手搬进软件 FIFO，
           回环就能做到"发多少收多少"。*/
        while ((0U == (UART4->SR & USART_SR_TXE)) && (0U != (-- guard)))
        {
            servo_rx_pump();
        }
        if (0U == guard) { break; }
        UART4->DR = buf[i];
        g_srv_tx_bytes ++;
    }

    /* ★ 发完**等最后一个位真的移出去（TC）**再放开总线 —— 早放会截断最后一个字节 */
    if (0U != g_servo_half)
    {
        uint32_t guard = SERVO_TX_WAIT_LOOPS;

        while ((0U == (UART4->SR & USART_SR_TC)) && (0U != (-- guard)))
        {
            servo_rx_pump();
        }
        servo_bus_release();            /* PC10 → 高阻，总线交给舵机 */
    }
    g_srv_tx_cnt ++;
}

/* 在字节流里找包头 FF FF（带空转上限，绝不死等）*/
static uint8_t servo_find_header (void)
{
    uint8_t b;
    uint8_t guard = 0U;

    for (;;)
    {
        if (0U != servo_rx_byte(&b)) { return 1U; }

        if (0xFFU == b)
        {
            if (0U != servo_rx_byte(&b)) { return 1U; }
            if (0xFFU == b) { return 0U; }              /* 连续两个 0xFF，包头确认 */
        }

        if (++ guard > SERVO_RX_GUARD_MAX) { return 1U; }
    }
}

/* 收一帧应答（最多试 4 次：前面可能是自己发出去的回显）*/
static uint8_t servo_recv_frame (uint8_t *rid, uint8_t *rerr, uint8_t *rdata, uint8_t *rlen)
{
    uint8_t  id, len, i, attempt;
    uint8_t  tmp[SERVO_RX_BUF_MAX];
    uint8_t  raw[SERVO_RX_BUF_MAX + 4U];                /* 整帧原样，用来和刚发那帧比对 */
    uint32_t sum;

    for (attempt = 0U; attempt < 4U; attempt ++)
    {
        if (0U != servo_find_header()) { return 1U; }

        raw[0] = 0xFFU;
        raw[1] = 0xFFU;

        if (0U != servo_rx_byte(&id))  { return 1U; }
        if (0U != servo_rx_byte(&len)) { return 1U; }
        raw[2] = id;
        raw[3] = len;

        if ((0U == len) || (len > SERVO_RX_BUF_MAX)) { continue; }   /* 长度不合理，重找 */

        for (i = 0U; i < len; i ++)
        {
            if (0U != servo_rx_byte(&tmp[i])) { return 1U; }
            raw[4U + i] = tmp[i];
        }

        /* ★★★ 回显过滤：和刚发出去那帧一模一样 ⇒ 是我们自己的回声，丢掉继续等 */
        if ((uint8_t)(len + 4U) == g_srv_last_tx_len)
        {
            uint8_t same = 1U;

            for (i = 0U; i < (uint8_t)(len + 4U); i ++)
            {
                if (raw[i] != g_srv_last_tx[i]) { same = 0U; break; }
            }
            if (0U != same) { continue; }
        }

        /* 校验：~(ID + LEN + tmp[0..len-2]) == tmp[len-1] */
        sum = (uint32_t)id + (uint32_t)len;
        for (i = 0U; i < (uint8_t)(len - 1U); i ++) { sum += (uint32_t)tmp[i]; }

        if ((uint8_t)((~sum) & 0xFFU) != tmp[len - 1U])
        {
            g_srv_rx_bad ++;
            continue;
        }

        if (NULL != rid)  { *rid  = id; }
        if (NULL != rerr) { *rerr = tmp[0]; }                    /* ERROR 是第一个字节 */
        if (NULL != rlen) { *rlen = (uint8_t)(len - 2U); }       /* 数据 = 去掉 ERROR 和校验 */

        if ((NULL != rdata) && (len > 2U))
        {
            for (i = 1U; i < (uint8_t)(len - 1U); i ++) { rdata[i - 1U] = tmp[i]; }
        }

        g_srv_rx_cnt ++;
        return 0U;
    }

    return 1U;
}

/* 写若干字节到舵机内存表 */
static uint8_t servo_write (uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t p[SERVO_PARAM_MAX];
    uint8_t err = 0xFFU;
    uint8_t i;

    if ((0U == n) || (n > (SERVO_PARAM_MAX - 1U))) { return 1U; }

    p[0] = addr;
    for (i = 0U; i < n; i ++) { p[1U + i] = data[i]; }

    servo_tx_frame(id, SERVO_INST_WRITE, p, (uint8_t)(n + 1U));

    if (0U != servo_recv_frame(NULL, &err, NULL, NULL)) { return 1U; }

    g_servo_err = err;
    return 0U;
}

/* 从舵机内存表读若干字节 */
static uint8_t servo_read (uint8_t id, uint8_t addr, uint8_t n, uint8_t *out)
{
    uint8_t p[2];
    uint8_t d[SERVO_RX_BUF_MAX];
    uint8_t dlen = 0U;
    uint8_t err  = 0xFFU;
    uint8_t i;

    if ((NULL == out) || (0U == n) || (n > SERVO_RX_BUF_MAX)) { return 1U; }

    p[0] = addr;
    p[1] = n;

    servo_tx_frame(id, SERVO_INST_READ, p, 2U);

    if (0U != servo_recv_frame(NULL, &err, d, &dlen)) { return 1U; }
    if (dlen < n) { return 1U; }

    for (i = 0U; i < n; i ++) { out[i] = d[i]; }

    g_servo_err = err;
    return 0U;
}

/* PING 一下（只读不写，验证"接线 + 波特率 + ID"） */
static uint8_t servo_ping (uint8_t id, uint8_t *err)
{
    uint8_t e   = 0xFFU;
    uint8_t rid = 0U;

    servo_tx_frame(id, SERVO_INST_PING, NULL, 0U);

    /* ★ 发完**立刻**开始收，中间不能加延时（1Mbps 下 6 字节约 60µs）*/
    if (0U != servo_recv_frame(&rid, &e, NULL, NULL)) { return 1U; }

    if (NULL != err) { *err = e; }
    return (0U != e) ? 2U : 0U;                 /* 2 = 应答了但舵机自己报故障 */
}

/* 读当前位置（同步，成功才更新 g_servo_pos）*/
static void servo_read_pos (void)
{
    uint8_t d[2];

    if (0U == servo_read(g_servo_id, SERVO_REG_CURPOS, 2U, d))
    {
        g_servo_pos = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));   /* 小端 */
    }
}

/* 设目标速度（0x2E，符号-幅值：负数最高位置 1）*/
static uint8_t servo_set_speed (int16_t speed)
{
    uint8_t  d[2];
    uint16_t w;

    if (speed >  (int16_t)CTRL_SERVO_SPEED) { speed =  (int16_t)CTRL_SERVO_SPEED; }
    if (speed < -(int16_t)CTRL_SERVO_SPEED) { speed = -(int16_t)CTRL_SERVO_SPEED; }

    w = (0 > speed) ? (uint16_t)((uint16_t)(-speed) | 0x8000U) : (uint16_t)speed;
    d[0] = (uint8_t)(w & 0xFFU);
    d[1] = (uint8_t)((w >> 8) & 0xFFU);

    return servo_write(g_servo_id, SERVO_REG_GOAL_SPEED, d, 2U);
}

//===================================================对外接口===================================================

uint32_t ctrl_servo_rx_bytes (void) { return g_srv_rx_bytes; }
uint32_t ctrl_servo_tx_bytes (void) { return g_srv_tx_bytes; }
uint32_t ctrl_servo_rx_bad   (void) { return g_srv_rx_bad; }
uint32_t ctrl_servo_baud     (void) { return g_servo_baud; }
uint32_t ctrl_servo_tx_cnt   (void) { return g_srv_tx_cnt; }
uint32_t ctrl_servo_rx_cnt   (void) { return g_srv_rx_cnt; }
uint8_t  ctrl_servo_id       (void) { return g_servo_id; }
uint8_t  ctrl_servo_get_err  (void) { return g_servo_err; }
uint16_t ctrl_servo_get_pos  (void) { return g_servo_pos; }
uint8_t  ctrl_servo_is_half  (void) { return g_servo_half; }

/* PC11(RX) 现在的电平：1 = 空闲高（正常），0 = 被拉低 */
uint8_t ctrl_servo_rx_pin (void)
{
    return (0U != (GPIOC->IDR & GPIO_PIN_11)) ? 1U : 0U;
}

/* 运行时切换"单线/全双工"
   单线（half=1）：PC10 发完立刻放成高阻，总线和 PC11 一起交给舵机（默认）；
   全双工（half=0）：PC10 一直是推挽 TX、PC11 是 RX（中间有调试板时才这么接）。*/
void ctrl_servo_set_half (uint8_t half)
{
    g_servo_half = (0U != half) ? 1U : 0U;

    if (0U == g_servo_half) { servo_bus_drive(); }     /* 全双工：一直由板子驱动 */
}

void ctrl_servo_init (void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC10 = UART4_TX (AF8)，PC11 = UART4_RX (AF8)，上拉 */
    gi.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gi);

    UART4->CR1 = 0U;
    UART4->CR1 = USART_CR1_UE;
    servo_set_baud(CTRL_SERVO_BAUD);
    UART4->CR1 |= (uint16_t)(USART_CR1_TE | USART_CR1_RE);

    /* ★★★ 关键改动：**不打开 RXNE 中断、不让 NVIC 管 UART4** ★★★
       整个驱动改成同步轮询（和 MSPM0 原工程一样）⇒ 没有任何中断，
       "中断风暴把时间基准饿死"这条路从结构上消失。*/
    UART4->CR1 &= (uint16_t)(~USART_CR1_RXNEIE);
    UART4->CR3 &= (uint16_t)(~USART_CR3_EIE);
    HAL_NVIC_DisableIRQ(UART4_IRQn);

    g_srv_tx_bytes = 0U;
    g_srv_rx_bytes = 0U;
    g_srv_rx_bad   = 0U;
    g_srv_tx_cnt   = 0U;
    g_srv_rx_cnt   = 0U;
    g_servo_pos      = 0xFFFFU;
    g_servo_err      = 0U;
    g_servo_boot     = 0U;
    g_servo_speed_ok = 0U;

    /* ★ 默认走"单线半双工"（发完放开 PC10），和现场把 PC10/PC11 并到一根线上的接法一致 */
    ctrl_servo_set_half((uint8_t)CTRL_SERVO_HALF_DUPLEX_DEFAULT);

    /* 等舵机上电初始化（原工程 system_delay_ms(300)；用 DWT 延时，不依赖 SysTick）*/
    ctrl_delay_us(300000UL);

    ctrl_servo_scan();
}

/* ★★★ 2026-11-07（FW=2107）：**ID 固定 1，波特率自动试两档** ★★★
   舵机型号 HX-10HM。它的手册前后不一致（第 1 页写 115200、规格表和协议文档写 1Mbps）——
   其实 115200 是**调试板到电脑**那一路，**舵机总线那一路是 1000000**。
   为了不再靠猜：开机把 1M → 115200 各 PING 一次（每次最多 1~2ms，总共几毫秒），
   谁应答就用谁；两个都不应答就停在 1M，运行中还会继续换档重试（见 board_ctrl_tick）。*/
void ctrl_servo_scan (void)
{
    static const uint32_t bauds[] = { CTRL_SERVO_BAUD, CTRL_SERVO_BAUD_ALT };
    uint8_t  bi;
    uint8_t  err = 0xFFU;

    ctrl_oled_boot(9U);

    g_servo_id = CTRL_SERVO_ID;                 /* ★ 固定 = 1 */

    for (bi = 0U; bi < (uint8_t)(sizeof(bauds) / sizeof(bauds[0])); bi ++)
    {
        servo_set_baud(bauds[bi]);

        if (0U == servo_ping(g_servo_id, &err))     /* 0 = 应答正常；2 = 应答但报故障 */
        {
            g_servo_err = err;
            return;                                 /* 这一档能通 ⇒ 定下来 */
        }
    }

    servo_set_baud(CTRL_SERVO_BAUD);            /* 都不应答：回默认档 */
    g_servo_err = 0xFFU;                        /* 留着给上位机显示 */
}

/* 走到目标位置（写 0x2A 两个字节，同步等应答）*/
uint8_t ctrl_servo_set_pos (uint16_t pos)
{
    uint8_t d[2];

    if (pos < CTRL_SERVO_POS_MIN) { pos = CTRL_SERVO_POS_MIN; }
    if (pos > CTRL_SERVO_POS_MAX) { pos = CTRL_SERVO_POS_MAX; }

    g_servo_target = pos;

    d[0] = (uint8_t)(pos & 0xFFU);
    d[1] = (uint8_t)((pos >> 8) & 0xFFU);

    if (0U != servo_write(g_servo_id, SERVO_REG_GOAL_POS, d, 2U)) { return 1U; }

    g_servo_pos      = pos;
    g_servo_last_set = pos;         /* 记下来 ⇒ 主循环节拍不会重复下发同一目标 */
    return 0U;
}

/* UART4 中断：同步轮询驱动已经把它关掉了，这里只保留符号（stm32f4xx_it.c 还在调）*/
void ctrl_uart4_irq (void)
{
    UART4->CR1 &= (uint16_t)(~USART_CR1_RXNEIE);
        (void)UART4->DR;
}

/* ★ 2026-11-07：舵机的"节拍"从 board_ctrl_tick() 搬到这里 ——
   它要用的 g_servo_boot / g_servo_target / g_srv_rx_cnt 等都是本文件的私有状态。 */
void ctrl_servo_tick (void)
{
    static uint8_t srv_div = 0U;

    if ((0U == g_srv_rx_cnt) && (g_servo_boot >= 50U))
    {
        static uint32_t t_baud = 0U;
        static uint8_t  alt    = 0U;

        if ((HAL_GetTick() - t_baud) > 3000U)
        {
            uint8_t err = 0xFFU;

            t_baud = HAL_GetTick();
            alt    = (uint8_t)(alt ^ 1U);
            servo_set_baud((0U != alt) ? CTRL_SERVO_BAUD_ALT : CTRL_SERVO_BAUD);
            (void)servo_ping(g_servo_id, &err);
            g_servo_err = err;
        }
    }

    if (g_servo_boot < 50U)
    {
        g_servo_boot ++;
    }
    else if (0U == g_servo_speed_ok)
    {
        if (0U == servo_set_speed((int16_t)CTRL_SERVO_SPEED)) { g_servo_speed_ok = 1U; }
    }
    else if (g_servo_target != g_servo_last_set)
    {
        (void)ctrl_servo_set_pos(g_servo_target);
        g_servo_last_set = g_servo_target;
    }
    else
    {
        srv_div ++;
        if (srv_div >= 10U) { srv_div = 0U; servo_read_pos(); }
    }
}
