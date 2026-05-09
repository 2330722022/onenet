#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <wlan_mgnt.h>
#include "aht10.h"
#include "ap3216c.h"
#include <drv_lcd.h>
#include <onenet.h>
#include <cJSON.h>
#include <string.h>
#include <stdlib.h>

/* ==================== 用户配置区 ==================== */
#define STUDENT_ID      "23001040215"
#define WLAN_SSID       "whu"
#define WLAN_PASSWORD   "99999999"

/* WiFi连接状态 */
static int wifi_connected = 0;
/* ==================== 引脚定义 ==================== */
#define LED_R_PIN       GET_PIN(F, 12)      // 红色LED (低电平亮)
#define BEEP_PIN        GET_PIN(B, 0)       // 蜂鸣器 (高电平响)
#define KEY_WK_UP       GET_PIN(C, 5)       // WK_UP按键

/* ==================== 舵机驱动 ==================== */
#define SERVO_PWM_DEV      "pwm3"
#define SERVO_PWM_CHANNEL  2
#define SERVO_PERIOD_NS    20000000         // 50Hz = 20ms周期 (20,000,000 ns)

#define SERVO_MIN_PULSE_NS 500000          // 0° = 500us = 500,000 ns
#define SERVO_MAX_PULSE_NS 2500000         // 180° = 2500us = 2,500,000 ns
#define SERVO_ANGLE_MIN    0
#define SERVO_ANGLE_MAX    180

static struct rt_device_pwm *servo_pwm_dev = RT_NULL;

int servo_init(void)
{
    servo_pwm_dev = (struct rt_device_pwm *)rt_device_find(SERVO_PWM_DEV);
    if (servo_pwm_dev == RT_NULL)
    {
        rt_kprintf("Servo: find %s failed!\n", SERVO_PWM_DEV);
        return -1;
    }
    
    rt_pwm_set(servo_pwm_dev, SERVO_PWM_CHANNEL, SERVO_PERIOD_NS, SERVO_MIN_PULSE_NS);
    rt_pwm_enable(servo_pwm_dev, SERVO_PWM_CHANNEL);
    
    rt_kprintf("Servo: initialized on %s channel %d\n", SERVO_PWM_DEV, SERVO_PWM_CHANNEL);
    return 0;
}

int servo_set_angle(int angle)
{
    if (servo_pwm_dev == RT_NULL)
    {
        rt_kprintf("Servo: not initialized!\n");
        return -1;
    }
    
    if (angle < SERVO_ANGLE_MIN || angle > SERVO_ANGLE_MAX)
    {
        rt_kprintf("Servo: angle %d out of range [%d, %d]!\n", angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        return -1;
    }
    
    uint32_t pulse_ns = SERVO_MIN_PULSE_NS + (uint32_t)((uint32_t)angle * (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS) / SERVO_ANGLE_MAX);
    
    rt_pwm_set(servo_pwm_dev, SERVO_PWM_CHANNEL, SERVO_PERIOD_NS, pulse_ns);
    
    return 0;
}

static void servo(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("Usage: servo <angle>\n");
        rt_kprintf("Example: servo 90\n");
        return;
    }
    servo_set_angle(atoi(argv[1]));
}
MSH_CMD_EXPORT(servo, set servo angle 0-180);

/* ==================== 传感器数据结构 ==================== */
struct sensor_data {
    float temperature;
    float humidity;
    float light;
    uint16_t proximity;
};

/* 全局变量：共享数据区及其互斥量 */
static struct sensor_data shared_data;
static rt_mutex_t data_mutex = RT_NULL;

/* 传感器设备句柄 */
static aht10_device_t aht20_dev = RT_NULL;
static ap3216c_device_t ap3216c_dev = RT_NULL;

/* 按键信号量 */
static rt_sem_t key_sem = RT_NULL;

/* ==================== 传感器采集线程 ==================== */
static void sensor_thread_entry(void *parameter)
{
    struct sensor_data data;
    while (1)
    {
        /* 读取传感器（两个不同 I2C，无需互斥） */
        data.temperature = aht10_read_temperature(aht20_dev);
        data.humidity    = aht10_read_humidity(aht20_dev);
        data.light       = ap3216c_read_ambient_light(ap3216c_dev);
        data.proximity   = ap3216c_read_ps_data(ap3216c_dev);

        /* 更新共享数据区（加锁） */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        shared_data = data;
        rt_mutex_release(data_mutex);

        rt_thread_mdelay(2000);
    }
}

/* ==================== LCD显示线程 ==================== */
static void display_thread_entry(void *parameter)
{
    struct sensor_data data;
    int light_int, temp_int, humi_int, temp_frac, humi_frac;
    char buf[32];

    lcd_clear(WHITE);
    lcd_set_color(WHITE, BLACK);

    /* 固定标题：学号 */
    lcd_show_string(20, 5, 16, "ID: ");
    lcd_show_string(60, 5, 16, STUDENT_ID);

    /* 绘制四分格边框（只绘制一次，后面只刷新内容） */
    lcd_draw_rectangle(0, 30, 120, 150);
    lcd_draw_rectangle(120, 30, 240, 150);
    lcd_draw_rectangle(0, 150, 120, 240);
    lcd_draw_rectangle(120, 150, 240, 240);

    while (1)
    {
        /* 读取共享数据（加锁） */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        data = shared_data;
        rt_mutex_release(data_mutex);

        /* 刷新 LCD 各格子内容 */
        /* 左上：光照 */
        light_int = (int)data.light;
        rt_sprintf(buf, "Lux:%d", light_int);
        lcd_fill(1, 31, 119, 149, WHITE);
        lcd_show_string(10, 50, 16, "Light");
        lcd_show_string(10, 80, 16, buf);

        /* 右上：接近 */
        rt_sprintf(buf, "PS:%d", data.proximity);
        lcd_fill(121, 31, 239, 149, WHITE);
        lcd_show_string(130, 50, 16, "Proximity");
        lcd_show_string(130, 80, 16, buf);

        /* 左下：温度 */
        temp_int = (int)data.temperature;
        temp_frac = (int)((data.temperature - temp_int) * 10);
        if (temp_frac < 0) temp_frac = -temp_frac;
        rt_sprintf(buf, "%d.%d C", temp_int, temp_frac);
        lcd_fill(1, 151, 119, 239, WHITE);
        lcd_show_string(10, 170, 16, "Temp");
        lcd_show_string(10, 200, 16, buf);

        /* 右下：湿度 */
        humi_int = (int)data.humidity;
        humi_frac = (int)((data.humidity - humi_int) * 10);
        if (humi_frac < 0) humi_frac = -humi_frac;
        rt_sprintf(buf, "%d.%d %%", humi_int, humi_frac);
        lcd_fill(121, 151, 239, 239, WHITE);
        lcd_show_string(130, 170, 16, "Humi");
        lcd_show_string(130, 200, 16, buf);

        /* 刷新间隔 1 秒（与采集周期 2 秒错开，保证数据不重复） */
        rt_thread_mdelay(1000);
    }
}

/* ==================== 按键处理 ==================== */
static void key_irq_callback(void *args)
{
    rt_sem_release(key_sem);   // 释放信号量，通知按键线程
}

static void key_thread_entry(void *parameter)
{
    int beep_on = 0;   // beep状态：0=关，1=开
    while (1)
    {
        rt_sem_take(key_sem, RT_WAITING_FOREVER);
        /* 消抖延时 20ms */
        rt_thread_mdelay(20);
        if (rt_pin_read(KEY_WK_UP) == PIN_LOW) {
            /* 切换状态 */
            beep_on = !beep_on;
            if (beep_on) {
                rt_pin_write(BEEP_PIN, PIN_HIGH);
                rt_kprintf("Key pressed: beep ON\n");
            } else {
                rt_pin_write(BEEP_PIN, PIN_LOW);
                rt_kprintf("Key pressed: beep OFF\n");
            }
            /* 等待按键松开 */
            while (rt_pin_read(KEY_WK_UP) == PIN_LOW) {
                rt_thread_mdelay(10);
            }
        }
    }
}

/* ==================== WiFi连接函数 ==================== */
static int wifi_connect(void)
{
    rt_kprintf("Connecting to WiFi: %s\n", WLAN_SSID);

    /* 注册WiFi事件回调 */
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED, wlan_event_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED, wlan_event_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_GOT_IP, wlan_event_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_START, wlan_event_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_STOP, wlan_event_callback, RT_NULL);

    /* 连接WiFi - 忽略返回值，因为它是异步的 */
    rt_wlan_connect(WLAN_SSID, WLAN_PASSWORD);

    /* 等待连接成功 - 使用简单的延时等待 */
    for (int i = 0; i < 60; i++)
    {
        /* 检查WiFi是否已连接 */
        if (rt_wlan_is_connected())
        {
            rt_kprintf("WiFi connected!\n");
            wifi_connected = 1;
            return 0;
        }
        rt_thread_mdelay(500);
    }

    rt_kprintf("WiFi connect timeout!\n");
    return -1;
}

/* ==================== OneNET下行控制回调函数 ==================== */
static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size)
{
    char resp_json[64];
    int request_id = 0;
    const char *data = (const char *)recv_data;
    
    /* 快速提取ID（字符串搜索） */
    const char *id_start = strstr(data, "\"id\":");
    if (id_start)
    {
        id_start += 5;
        while (*id_start == ' ' || *id_start == '\"') id_start++;
        request_id = atoi(id_start);
    }
    
    /* 最快响应：直接构造JSON字符串 */
    rt_sprintf(resp_json, "{\"id\":%d,\"code\":200,\"msg\":\"success\"}", request_id);
    
    /* 返回响应 */
    *resp_data = (uint8_t *)rt_malloc(strlen(resp_json) + 1);
    if (*resp_data)
    {
        strcpy((char *)*resp_data, resp_json);
        *resp_size = strlen(resp_json);
    }
    
    /* 控制硬件 */
    const char *led_start = strstr(data, "\"led\":");
    if (led_start)
    {
        led_start += 6;
        while (*led_start == ' ') led_start++;
        if (*led_start == 't' || *led_start == 'T' || *led_start == '1')
            rt_pin_write(LED_R_PIN, PIN_LOW);
        else
            rt_pin_write(LED_R_PIN, PIN_HIGH);
    }
    
    const char *beep_start = strstr(data, "\"beep\":");
    if (beep_start)
    {
        beep_start += 7;
        while (*beep_start == ' ') beep_start++;
        if (*beep_start == 't' || *beep_start == 'T' || *beep_start == '1')
            rt_pin_write(BEEP_PIN, PIN_HIGH);
        else
            rt_pin_write(BEEP_PIN, PIN_LOW);
    }
}

/* ==================== WiFi事件回调 ==================== */
static void wlan_event_callback(int event, struct rt_wlan_buff *buff, void *parameter)
{
    switch (event)
    {
        case RT_WLAN_EVT_STA_CONNECTED:
            rt_kprintf("WiFi: connected to AP\n");
            break;
        case RT_WLAN_EVT_STA_DISCONNECTED:
            rt_kprintf("WiFi: disconnected from AP\n");
            wifi_connected = 0;
            /* 尝试自动重连 */
            rt_wlan_connect(WIFI_SSID, WIFI_PASSWORD);
            break;
        case RT_WLAN_EVT_STA_GOT_IP:
            rt_kprintf("WiFi: got IP address\n");
            wifi_connected = 1;
            break;
        case RT_WLAN_EVT_STA_START:
            rt_kprintf("WiFi: STA started\n");
            break;
        case RT_WLAN_EVT_STA_STOP:
            rt_kprintf("WiFi: STA stopped\n");
            wifi_connected = 0;
            break;
        default:
            break;
    }
}

/* ==================== OneNET数据上传线程 ==================== */
static void onenet_upload_thread_entry(void *parameter)
{
    struct sensor_data data;
    rt_err_t result;
    int upload_count = 0;
    cJSON *root = RT_NULL;
    cJSON *params = RT_NULL;
    char *json_str = RT_NULL;

    rt_kprintf("OneNET upload thread: started\n");

    while (1)
    {
        /* 检查WiFi连接状态 */
        if (!wifi_connected)
        {
            rt_kprintf("OneNET: WiFi not connected\n");
            rt_thread_mdelay(2000);
            continue;
        }

        /* 读取共享数据（加锁） */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        data = shared_data;
        rt_mutex_release(data_mutex);

        /* 使用cJSON构建OneNET物模型标准格式JSON数据 */
        root = cJSON_CreateObject();
        if (root)
        {
            cJSON_AddStringToObject(root, "id", "123");
            cJSON_AddStringToObject(root, "version", "1.0");
            
            params = cJSON_CreateObject();
            if (params)
            {
                cJSON_AddNumberToObject(params, "humidity", data.humidity);
                cJSON_AddNumberToObject(params, "light", data.light);
                cJSON_AddItemToObject(root, "params", params);
            }
            
            cJSON_AddStringToObject(root, "method", "thing.property.post");
            
            json_str = cJSON_PrintUnformatted(root);
        }

        /* 在串口打印要发送的JSON */
        if (json_str)
        {
            rt_kprintf("OneNET: sending JSON: %s\n", json_str);

            /* 使用MQTT直接发布到物模型上传topic（跳过库的自动封装） */
            result = onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post", (uint8_t *)json_str, strlen(json_str));
            if (result == 0)
            {
                rt_kprintf("OneNET: upload success\n");
            }
            else
            {
                rt_kprintf("OneNET: upload failed, ret=%d\n", result);
            }
            
            /* 释放JSON字符串内存 */
            rt_free(json_str);
            json_str = RT_NULL;
        }
        
        /* 释放cJSON对象 */
        if (root)
        {
            cJSON_Delete(root);
            root = RT_NULL;
            params = RT_NULL;
        }

        upload_count++;
        rt_kprintf("OneNET: upload count = %d\n", upload_count);

        /* 每5秒上传一次 */
        rt_thread_mdelay(5000);
    }
}

/* ==================== 系统初始化 ==================== */
static void system_init(void)
{
    /* 1. 初始化 LED、蜂鸣器引脚 */
    rt_pin_mode(LED_R_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BEEP_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_R_PIN, PIN_HIGH);
    rt_pin_write(BEEP_PIN, PIN_LOW);

    /* 1.5. 初始化舵机 */
    if (servo_init() != 0)
    {
        rt_kprintf("Servo init failed!\n");
    }

    /* 2. 初始化按键（中断 + 信号量 + 线程） */
    key_sem = rt_sem_create("key_sem", 0, RT_IPC_FLAG_FIFO);
    rt_thread_t tid_key = rt_thread_create("key", key_thread_entry, RT_NULL,
                                            1024, 22, 10);
    if (tid_key) rt_thread_startup(tid_key);

    rt_pin_mode(KEY_WK_UP, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(KEY_WK_UP, PIN_IRQ_MODE_FALLING, key_irq_callback, RT_NULL);
    rt_pin_irq_enable(KEY_WK_UP, PIN_IRQ_ENABLE);

    /* 3. 初始化传感器 */
    aht20_dev = aht10_init("i2c3");
    ap3216c_dev = ap3216c_init("i2c2");
    if (aht20_dev == RT_NULL || ap3216c_dev == RT_NULL)
    {
        rt_kprintf("Sensor init failed!\n");
        return;
    }

    /* 4. 创建互斥量（保护共享数据）和消息队列（LCD -> MQTT） */
    data_mutex = rt_mutex_create("data_mutex", RT_IPC_FLAG_FIFO);
    if (data_mutex == RT_NULL)
    {
        rt_kprintf("IPC create failed!\n");
        return;
    }

    /* 5. 创建采集线程和显示线程 */
    rt_thread_t tid_sensor = rt_thread_create("sensor", sensor_thread_entry, RT_NULL,
                                               2048, 20, 10);
    rt_thread_t tid_display = rt_thread_create("display", display_thread_entry, RT_NULL,
                                                4096, 21, 10);
    if (tid_sensor && tid_display)
    {
        rt_thread_startup(tid_sensor);
        rt_thread_startup(tid_display);
    }
    else
    {
        rt_kprintf("Thread create failed!\n");
    }
}

int main(void)
{
    int ret;
    rt_thread_t tid_onenet;

    system_init();

    lcd_clear(WHITE);
    lcd_show_string(20, 100, 24, "System Ready");
    lcd_show_string(20, 130, 16, "Connecting WiFi...");

    /* 自动连接WiFi */
    rt_kprintf("Connecting to WiFi network...\n");
    ret = wifi_connect();
    if (ret != 0)
    {
        rt_kprintf("WiFi connection failed!\n");
        lcd_show_string(20, 160, 16, "WiFi Failed");
    }
    else
    {
        rt_kprintf("WiFi connected successfully!\n");
        lcd_show_string(20, 160, 16, "WiFi Connected");

        /* 初始化 OneNET */
        lcd_fill(0, 80, 240, 240, WHITE);
        lcd_show_string(20, 100, 24, "Init OneNET...");
        rt_kprintf("Initializing OneNET...\n");

        ret = onenet_mqtt_init();
        if (ret != 0)
        {
            rt_kprintf("OneNET init failed: %d\n", ret);
            lcd_show_string(20, 130, 16, "OneNET Init Failed");
        }
        else
        {
            rt_kprintf("OneNET init success\n");
            lcd_show_string(20, 130, 16, "OneNET Connected");

            /* 注册下行控制回调函数 */
            onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);
            rt_kprintf("OneNET callback registered\n");

            /* 创建 OneNET 数据上传线程 */
            tid_onenet = rt_thread_create("onenet", onenet_upload_thread_entry, RT_NULL,
                                          2048, 19, 10);
            if (tid_onenet)
            {
                rt_thread_startup(tid_onenet);
                rt_kprintf("OneNET upload thread started\n");
            }
            else
            {
                rt_kprintf("Failed to create onenet thread\n");
            }
        }
    }

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
