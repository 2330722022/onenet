#ifndef APP_LOGIC_H__
#define APP_LOGIC_H__

#include <rtthread.h>

/* ==================== 业务状态结构体 ==================== */
struct app_state {
    float temp_threshold;       /* 温度阈值 */
    uint8_t beep_status;        /* 蜂鸣器状态 */
    int alarm_state;            /* 告警状态 */
};

/* ==================== 告警事件集 ==================== */
#define EVENT_TEMP_ALARM       (1 << 0)
#define EVENT_TILT_ALARM       (1 << 1)
#define EVENT_VIBRATION_ALARM  (1 << 2)
#define EVENT_ALARM_CLEAR      (1 << 3)

/* 全局互斥锁 — 保护 g_app_state 的临界区 */
extern rt_mutex_t data_lock;

/* 全局状态变量 */
extern struct app_state g_app_state;

/* 告警事件集控制块 */
extern struct rt_event evt_alarm;

/* ==================== 初始化 ==================== */
void app_logic_init(void);

/* ==================== 核心处理 — 临界区保护 ==================== */
void logic_handle(float temperature);

/* ==================== BEEP 控制 ==================== */
void beep_set(uint8_t on);

/* ==================== 按键事件（仅保留UP/DOWN调整阈值）==================== */
void process_key_events(void);

/* ==================== LCD显示阈值更新 ==================== */
void update_threshold_display(void);

#endif
