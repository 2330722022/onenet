#include "app_logic.h"
#include <board.h>
#include <rtdevice.h>

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

/* ==================== 全局状态变量 ==================== */
float temp_threshold = 30.0f;       // 温度阈值（默认30度）
uint8_t beep_status = 0;            // 蜂鸣器状态
volatile int auto_mode = 1;         // 工作模式

/* 按键事件标志位 */
static uint8_t key_event_flags = 0;
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
void beep_set(uint8_t on)
{
    beep_status = on;
    rt_pin_write(BEEP_PIN, on ? PIN_HIGH : PIN_LOW);
}

/* ==================== 核心逻辑处理 ==================== */
void logic_process(float temperature)
{
    if (temperature > temp_threshold)
    {
        /* 超温：报警 + 蜂鸣器 + 舵机 */
        beep_set(1);
        servo_set_angle(90);
        rt_kprintf("[logic] Temp %.1f > threshold %.1f, alarm ON, servo 90\n",
                   temperature, temp_threshold);
    }
    else
    {
        /* 正常：关报警 + 关蜂鸣器 + 舵机回正 */
        beep_set(0);
        servo_set_angle(0);
    }
}

/* ==================== 按键回调 ==================== */
static void key_wkup_callback(void *args)
{
    key_event_flags |= KEY_EVENT_WKUP_SHORT;
    rt_kprintf("[key_int] WK_UP pressed\n");
}

static void key_down_callback(void *args)
{
    key_event_flags |= KEY_EVENT_DOWN;
    rt_kprintf("[key_int] DOWN pressed\n");
}

/* ==================== 按键事件处理 ==================== */
void process_key_events(void)
{
    uint8_t flags = key_event_flags;
    if (flags == 0) return;
    key_event_flags = 0;

    if (flags & KEY_EVENT_WKUP_SHORT)
    {
        temp_threshold += 0.5f;
        if (temp_threshold > 50.0f) temp_threshold = 50.0f;
        update_threshold_display();
        rt_kprintf("[key] WK_UP: threshold +0.5 -> %.1f\n", temp_threshold);
    }
    if (flags & KEY_EVENT_DOWN)
    {
        temp_threshold -= 0.5f;
        if (temp_threshold < 20.0f) temp_threshold = 20.0f;
        update_threshold_display();
        rt_kprintf("[key] DOWN: threshold -0.5 -> %.1f\n", temp_threshold);
    }
}

/* 阈值LCD显示更新（由main.c实现具体LVGL操作）*/
RT_WEAK void update_threshold_display(void)
{
}

/* ==================== 按键初始化 ==================== */
void app_key_init(void)
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
void app_logic_init(void)
{
    /* 初始化BEEP引脚 */
    rt_pin_mode(BEEP_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BEEP_PIN, PIN_LOW);

    /* 初始化舵机 */
    servo_init();

    /* 初始化按键（仅WK_UP/DOWN）*/
    app_key_init();

    rt_kprintf("[logic] Module initialized (threshold=%.1f)\n", temp_threshold);
}
