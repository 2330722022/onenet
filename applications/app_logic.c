#include "app_logic.h"
#include <board.h>
#include <rtdevice.h>

/*
 * 模块：业务逻辑中心 (app_logic.c)
 * 版本：v2.0 稳定版
 * 日期：2026-05-13
 * 功能：统一管理共享资源（阈值、告警状态、蜂鸣器），通过互斥量实现临界区保护。
 * OS 概念体现：
 *   1. 互斥量 (Mutex)：data_lock 保护 g_app_state，防止多线程数据竞争。
 *      例如 Android HTTP 修改阈值和硬件按键修改阈值可能同时发生，必须互斥。
 *   2. 事件集 (Event)：逻辑线程阻塞等待传感器线程发送的告警事件，
 *      实现线程间异步通知，避免轮询浪费 CPU。
 *   3. 共享资源封装：将分散的全局变量封装到 struct app_state 中统一管理。
 */

/* ==================== 引脚定义 ==================== */
#define BEEP_PIN        GET_PIN(B, 0)       // 蜂鸣器 (高电平响)
#define KEY_WK_UP       GET_PIN(C, 5)       // WK_UP按键 - 增加阈值
#define KEY_DOWN        GET_PIN(C, 4)       // DOWN按键 - 减少阈值

/* ==================== 舵机驱动 ==================== */
#define SERVO_PWM_DEV      "pwm2"
#define SERVO_PWM_CHANNEL  4
#define SERVO_PERIOD_NS    20000000
#define SERVO_MIN_PULSE_NS 500000
#define SERVO_MAX_PULSE_NS 2500000
#define SERVO_ANGLE_MIN    0
#define SERVO_ANGLE_MAX    180

static struct rt_device_pwm *servo_pwm_dev = RT_NULL;
static int servo_last_angle = -1;

/* ==================== 全局 RT-Thread 对象 ==================== */
/*
 * data_lock:
 *   互斥量 — 保护 g_app_state（阈值、beep、alarm_state）的临界区。
 *   所有读/写 g_app_state 的线程（逻辑线程、按键处理、HTTP 服务、OneNET 下行）
 *   必须先持有此锁，防止数据竞争（Data Race）。
 */
rt_mutex_t data_lock = RT_NULL;

/*
 * g_app_state:
 *   全局业务状态结构体 — 将分散的全局变量（temp_threshold, beep_status, alarm_state）
 *   封装为单一结构体，便于互斥量统一保护。
 */
struct app_state g_app_state = {
    .temp_threshold = 40.0f,
    .beep_status    = 0,
    .alarm_state    = 0,
};

/*
 * evt_alarm:
 *   事件集 — 传感器线程检测到异常时发送事件，逻辑线程阻塞等待。
 *   实现生产者-消费者模式，避免轮询。
 */
struct rt_event evt_alarm;

/* 按键事件标志位（ISR 快速置位，线程轮询处理）*/
static volatile uint8_t key_event_flags = 0;
#define KEY_EVENT_WKUP_SHORT    (1 << 0)
#define KEY_EVENT_DOWN          (1 << 2)

/* ==================== 舵机操作 ==================== */
static int servo_init(void)
{
    servo_pwm_dev = (struct rt_device_pwm *)rt_device_find(SERVO_PWM_DEV);
    if (servo_pwm_dev == RT_NULL)
    {
        rt_kprintf("[logic] Servo: find %s failed!\n", SERVO_PWM_DEV);
        return -1;
    }

    rt_pwm_set(servo_pwm_dev, SERVO_PWM_CHANNEL, SERVO_PERIOD_NS, SERVO_MIN_PULSE_NS);
    rt_pwm_enable(servo_pwm_dev, SERVO_PWM_CHANNEL);

    servo_last_angle = SERVO_ANGLE_MIN;
    rt_kprintf("[logic] Servo: initialized on %s channel %d\n", SERVO_PWM_DEV, SERVO_PWM_CHANNEL);
    return 0;
}

static void servo_set_angle(int angle)
{
    if (servo_pwm_dev == RT_NULL) return;
    if (angle < SERVO_ANGLE_MIN || angle > SERVO_ANGLE_MAX) return;
    if (angle == servo_last_angle) return;

    uint32_t pulse_ns = SERVO_MIN_PULSE_NS + (uint32_t)((uint32_t)angle * (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS) / SERVO_ANGLE_MAX);
    rt_pwm_set(servo_pwm_dev, SERVO_PWM_CHANNEL, SERVO_PERIOD_NS, pulse_ns);
    servo_last_angle = angle;
    rt_kprintf("[logic] Servo: set angle %d\n", angle);
}

/* ==================== BEEP 控制 ==================== */
/*
 * beep_set:
 *   互斥量保护的 BEEP 控制函数。
 *   临界区：写入 g_app_state.beep_status 和操作硬件引脚必须原子化，
 *   防止 Android 设置 BEEP 和按键设置 BEEP 同时发生时产生竞态。
 */
void beep_set(uint8_t on)
{
    rt_mutex_take(data_lock, RT_WAITING_FOREVER);   /* 进入临界区 */
    g_app_state.beep_status = on;
    rt_pin_write(BEEP_PIN, on ? PIN_HIGH : PIN_LOW);
    rt_mutex_release(data_lock);                     /* 退出临界区 */
}

/* ==================== 核心逻辑处理 ==================== */
/*
 * logic_handle:
 *   被传感器线程调用的入口函数。负责根据温度值判定是否需要告警，
 *   并通过事件集通知逻辑线程执行实际操作。
 *
 *   临界区保护：
 *     - 读取 g_app_state.temp_threshold 前必须持有 data_lock，
 *       防止在读取过程中被 Android/按键修改阈值。
 *     - 写入 g_app_state.alarm_state 同理。
 */
void logic_handle(float temperature)
{
    int should_alarm = 0;

    while (rt_mutex_take(data_lock, RT_WAITING_NO) != RT_EOK)
        rt_thread_mdelay(1);

    /* 临界区：原子读取阈值 */
    float threshold = g_app_state.temp_threshold;

    if (temperature > threshold)
    {
        should_alarm = 1;
        g_app_state.alarm_state = 1;
        rt_kprintf("[logic] Temp %d.%d > threshold %d.%d, ALARM\n",
                   (int)temperature, abs((int)((temperature - (int)temperature) * 10)),
                   (int)threshold, abs((int)((threshold - (int)threshold) * 10)));
    }
    else
    {
        g_app_state.alarm_state = 0;
    }
    rt_mutex_release(data_lock);                     /* 退出临界区 */

    /* 发送事件通知逻辑线程 */
    if (should_alarm)
    {
        rt_event_send(&evt_alarm, EVENT_TEMP_ALARM);
    }
    else
    {
        rt_event_send(&evt_alarm, EVENT_ALARM_CLEAR);
    }
}

/* ==================== 逻辑处理线程 ==================== */
/*
 * 逻辑线程：
 *   阻塞等待传感器线程发送的告警事件，收到事件后执行舵机/BEEP操作。
 *   这种事件驱动模型体现了 RT-Thread 线程间异步通信的能力。
 */
static void logic_thread_entry(void *parameter)
{
    rt_uint32_t events;

    rt_kprintf("[logic] Logic thread started, waiting for events...\n");

    while (1)
    {
        /*
         * 阻塞等待任意告警事件。RT_WAITING_FOREVER 使线程挂起，不消耗 CPU。
         * 当传感器线程发送事件时，内核唤醒本线程。
         */
        if (rt_event_recv(&evt_alarm,
                          EVENT_TEMP_ALARM | EVENT_TILT_ALARM |
                          EVENT_VIBRATION_ALARM | EVENT_ALARM_CLEAR,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &events) == RT_EOK)
        {
            rt_mutex_take(data_lock, RT_WAITING_FOREVER);   /* 进入临界区 */

            if (events & EVENT_TEMP_ALARM)
            {
                g_app_state.beep_status = 1;
                g_app_state.alarm_state = 1;
                rt_pin_write(BEEP_PIN, PIN_HIGH);
                servo_set_angle(90);
                rt_kprintf("[logic] EVENT: TEMP_ALARM -> beep ON, servo 90\n");
            }
            else if (events & EVENT_ALARM_CLEAR)
            {
                g_app_state.beep_status = 0;
                g_app_state.alarm_state = 0;
                rt_pin_write(BEEP_PIN, PIN_LOW);
                servo_set_angle(0);
            }

            rt_mutex_release(data_lock);                     /* 退出临界区 */
        }
    }
}

/* ==================== 按键回调 ==================== */
/*
 * 按键 ISR 回调 — 仅快速置位标志位，实际处理由传感器线程轮询完成。
 * 这种"ISR 生产 + 线程消费"模式是 RTOS 中断处理的经典范式。
 */
static void key_wkup_callback(void *args)
{
    key_event_flags |= KEY_EVENT_WKUP_SHORT;
}

static void key_down_callback(void *args)
{
    key_event_flags |= KEY_EVENT_DOWN;
}

/* ==================== 按键事件处理 ==================== */
/*
 * process_key_events:
 *   由传感器线程周期性调用，消费按键事件。
 *
 *   临界区保护：
 *     - 修改 g_app_state.temp_threshold 时持有 data_lock，
 *       确保与 Android HTTP/OneNET 下行互斥。
 */
void process_key_events(void)
{
    uint8_t flags = key_event_flags;
    if (flags == 0) return;
    key_event_flags = 0;

    rt_mutex_take(data_lock, RT_WAITING_FOREVER);           /* 进入临界区 */

    if (flags & KEY_EVENT_WKUP_SHORT)
    {
        g_app_state.temp_threshold += 0.5f;
        if (g_app_state.temp_threshold > 50.0f)
            g_app_state.temp_threshold = 50.0f;
        rt_kprintf("[key] WK_UP: threshold +0.5 -> %d.%d\n",
                   (int)g_app_state.temp_threshold,
                   abs((int)((g_app_state.temp_threshold - (int)g_app_state.temp_threshold) * 10)));
    }
    if (flags & KEY_EVENT_DOWN)
    {
        g_app_state.temp_threshold -= 0.5f;
        if (g_app_state.temp_threshold < 20.0f)
            g_app_state.temp_threshold = 20.0f;
        rt_kprintf("[key] DOWN: threshold -0.5 -> %d.%d\n",
                   (int)g_app_state.temp_threshold,
                   abs((int)((g_app_state.temp_threshold - (int)g_app_state.temp_threshold) * 10)));
    }

    rt_mutex_release(data_lock);                             /* 退出临界区 */

    update_threshold_display();
}

/* ==================== 阈值LCD显示更新（弱符号，由main.c重写）==================== */
RT_WEAK void update_threshold_display(void)
{
}

/* ==================== 按键初始化 ==================== */
static void app_key_init(void)
{
    rt_pin_mode(KEY_WK_UP, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(KEY_WK_UP, PIN_IRQ_MODE_FALLING, key_wkup_callback, RT_NULL);
    rt_pin_irq_enable(KEY_WK_UP, PIN_IRQ_ENABLE);

    rt_pin_mode(KEY_DOWN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(KEY_DOWN, PIN_IRQ_MODE_FALLING, key_down_callback, RT_NULL);
    rt_pin_irq_enable(KEY_DOWN, PIN_IRQ_ENABLE);

    rt_kprintf("[logic] Key init done (WK_UP=PC5, DOWN=PC4)\n");
}

/* ==================== 模块初始化 ==================== */
/*
 * app_logic_init:
 *   创建 RT-Thread 同步对象（互斥量、事件集），初始化硬件，
 *   并启动逻辑处理线程。
 *
 *   初始化序列体现了 RT-Thread 对象的创建顺序：
 *     1. 互斥量 — 最先创建，供后续所有线程使用
 *     2. 事件集 — 线程间通信基础
 *     3. 硬件外设 — BEEP、舵机、按键
 *     4. 逻辑线程 — 最后启动，开始等待事件
 */
void app_logic_init(void)
{
    /* 1. 创建互斥量 — 保护共享资源 g_app_state */
    data_lock = rt_mutex_create("data_lock", RT_IPC_FLAG_FIFO);
    if (data_lock == RT_NULL)
    {
        rt_kprintf("[logic] FATAL: mutex create failed!\n");
        return;
    }

    /* 2. 创建事件集 — 用于传感器→逻辑线程的异步通知 */
    rt_event_init(&evt_alarm, "evt_alarm", RT_IPC_FLAG_FIFO);

    /* 3. 初始化硬件 */
    rt_pin_mode(BEEP_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BEEP_PIN, PIN_LOW);

    servo_init();
    app_key_init();

    /* 4. 启动逻辑处理线程 */
    rt_thread_t tid_logic = rt_thread_create("logic",
                                              logic_thread_entry,
                                              RT_NULL,
                                              2048,
                                              8,
                                              10);
    if (tid_logic)
    {
        rt_thread_startup(tid_logic);
        rt_kprintf("[logic] Module initialized (threshold=%d.%d)\n",
                   (int)g_app_state.temp_threshold,
                   abs((int)((g_app_state.temp_threshold - (int)g_app_state.temp_threshold) * 10)));
    }
}
