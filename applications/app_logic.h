#ifndef APP_LOGIC_H__
#define APP_LOGIC_H__

#include <rtthread.h>

/* 业务状态结构体 */
struct app_state {
    float temp_threshold;
    uint8_t beep_status;
    int alarm_state;
    int work_mode;
    int fan_status;
};

/* 全局状态变量 */
extern float temp_threshold;
extern uint8_t beep_status;
extern volatile int auto_mode;

/* ==================== 初始化 ==================== */
void app_logic_init(void);

/* ==================== 核心处理 ==================== */
void logic_process(float temperature);

/* ==================== BEEP 控制 ==================== */
void beep_set(uint8_t on);

/* ==================== 按键事件（仅保留UP/DOWN调整阈值）==================== */
void app_key_init(void);
void process_key_events(void);
void update_threshold_display(void);

#endif
