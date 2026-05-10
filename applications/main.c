#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <wlan_mgnt.h>
#include "aht10.h"
#include "ap3216c.h"
#include "icm20608.h"
#include <drv_lcd.h>
#include <onenet.h>
#include <cJSON.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* LVGL头文件 */
#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

/* 声明中文字库 */
LV_FONT_DECLARE(my_font_cn_16);

/* 图标资源声明（变量名与文件名一致）*/
extern const lv_img_dsc_t Environmental;   // 温度计图标
extern const lv_img_dsc_t waterprof;       // 水滴图标
extern const lv_img_dsc_t light;           // 光照图标
extern const lv_img_dsc_t Alarm;           // 红色警报图标
extern const lv_img_dsc_t tilt;            // 货架倾斜图标
extern const lv_img_dsc_t connected;       // WiFi状态图标
extern const lv_img_dsc_t onenet_upload;   // OneNET上传图标
extern const lv_img_dsc_t Megaphone;       // 蜂鸣器图标

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
    static int last_angle = -1;
    
    if (servo_pwm_dev == RT_NULL) return -1;
    
    if (angle < SERVO_ANGLE_MIN || angle > SERVO_ANGLE_MAX) return -1;

    if (angle == last_angle) return 0;
 
    uint32_t pulse_ns = SERVO_MIN_PULSE_NS + (uint32_t)((uint32_t)angle * (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS) / SERVO_ANGLE_MAX);
    
    rt_pwm_set(servo_pwm_dev, SERVO_PWM_CHANNEL, SERVO_PERIOD_NS, pulse_ns);
    last_angle = angle;
    
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

/* ==================== ICM20608倾斜监测配置 ==================== */
#define TILT_THRESHOLD     15      // 倾斜阈值（度）
#define TILT_SENSITIVITY   2       // 灵敏度（连续多少次超过阈值才触发报警）

/* ==================== 传感器数据结构 ==================== */
struct sensor_data {
    float temperature;
    float humidity;
    float light;
    uint16_t proximity;
    float tilt_angle_x;            // X轴倾斜角度
    float tilt_angle_y;            // Y轴倾斜角度
    int tilt_alarm;                // 倾斜报警状态：0=正常，1=报警
};

/* 全局变量：共享数据区及其互斥量 */
static struct sensor_data shared_data;
static rt_mutex_t data_mutex = RT_NULL;

/* 传感器设备句柄 */
static aht10_device_t aht20_dev = RT_NULL;
static ap3216c_device_t ap3216c_dev = RT_NULL;
static icm20608_device_t icm20608_dev = RT_NULL;

/* 按键信号量 */
static rt_sem_t key_sem = RT_NULL;

/* ==================== 倾斜角度计算函数 ==================== */
static void calculate_tilt_angle(rt_int16_t accel_x, rt_int16_t accel_y, rt_int16_t accel_z, 
                                  float *angle_x, float *angle_y)
{
    /* 计算重力加速度大小 */
    float g = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    
    if (g < 100)  // 防止除零
    {
        *angle_x = 0;
        *angle_y = 0;
        return;
    }
    
    /* 计算X轴倾斜角度：绕X轴旋转时，Y轴加速度变化 */
    *angle_x = atan2((float)accel_y, (float)accel_z) * 180 / 3.14159f;
    
    /* 计算Y轴倾斜角度：绕Y轴旋转时，X轴加速度变化 */
    *angle_y = atan2((float)accel_x, (float)accel_z) * 180 / 3.14159f;
}

/* ==================== ICM20608测试命令 ==================== */
static void icm20608_test(int argc, char **argv)
{
    rt_int16_t accel_x, accel_y, accel_z;
    rt_int16_t gyro_x, gyro_y, gyro_z;
    float angle_x, angle_y;
    
    if (icm20608_dev == RT_NULL)
    {
        rt_kprintf("ICM20608 not initialized!\n");
        return;
    }
    
    /* 读取加速度计数据 */
    if (icm20608_get_accel(icm20608_dev, &accel_x, &accel_y, &accel_z) == RT_EOK)
    {
        calculate_tilt_angle(accel_x, accel_y, accel_z, &angle_x, &angle_y);
        rt_kprintf("ICM20608 Accel: X=%d, Y=%d, Z=%d\n", accel_x, accel_y, accel_z);
        rt_kprintf("Tilt Angle: X=%d.%d deg, Y=%d.%d deg\n", 
                   (int)angle_x, abs((int)(angle_x * 10) % 10),
                   (int)angle_y, abs((int)(angle_y * 10) % 10));
    }
    else
    {
        rt_kprintf("Failed to read accelerometer!\n");
    }
    
    /* 读取陀螺仪数据 */
    if (icm20608_get_gyro(icm20608_dev, &gyro_x, &gyro_y, &gyro_z) == RT_EOK)
    {
        rt_kprintf("ICM20608 Gyro: X=%d, Y=%d, Z=%d\n", gyro_x, gyro_y, gyro_z);
    }
    else
    {
        rt_kprintf("Failed to read gyroscope!\n");
    }
}
MSH_CMD_EXPORT(icm20608_test, test ICM20608 sensor);

/* ==================== 传感器采集线程 ==================== */
static void sensor_thread_entry(void *parameter)
{
    struct sensor_data data;
    rt_int16_t accel_x, accel_y, accel_z;
    static int tilt_warning_count = 0;
    
    while (1)
    {
        /* 读取环境传感器 */
        data.temperature = aht10_read_temperature(aht20_dev);
        data.humidity    = aht10_read_humidity(aht20_dev);
        data.light       = ap3216c_read_ambient_light(ap3216c_dev);
        data.proximity   = ap3216c_read_ps_data(ap3216c_dev);
        
        /* 读取ICM20608加速度计数据 */
        if (icm20608_dev != RT_NULL)
        {
            if (icm20608_get_accel(icm20608_dev, &accel_x, &accel_y, &accel_z) == RT_EOK)
            {
                /* 计算倾斜角度 */
                calculate_tilt_angle(accel_x, accel_y, accel_z, 
                                     &data.tilt_angle_x, &data.tilt_angle_y);
                
                /* 检测倾斜是否超过阈值 */
                if (abs(data.tilt_angle_x) > TILT_THRESHOLD || 
                    abs(data.tilt_angle_y) > TILT_THRESHOLD)
                {
                    tilt_warning_count++;
                    if (tilt_warning_count >= TILT_SENSITIVITY)
                    {
                        data.tilt_alarm = 1;
                        rt_kprintf("ALERT: Shelf tilted! X:%d.%d Y:%d.%d\n", 
                                   (int)data.tilt_angle_x, abs((int)(data.tilt_angle_x * 10) % 10),
                                   (int)data.tilt_angle_y, abs((int)(data.tilt_angle_y * 10) % 10));
                    }
                }
                else
                {
                    tilt_warning_count = 0;
                    data.tilt_alarm = 0;
                }
                
                rt_kprintf("ICM20608: X:%d.%d Y:%d.%d Alarm=%d\n", 
                           (int)data.tilt_angle_x, abs((int)(data.tilt_angle_x * 10) % 10),
                           (int)data.tilt_angle_y, abs((int)(data.tilt_angle_y * 10) % 10),
                           data.tilt_alarm);
            }
            else
            {
                data.tilt_angle_x = 0;
                data.tilt_angle_y = 0;
                data.tilt_alarm = 0;
            }
        }
        
        /* 更新共享数据区（加锁） */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        shared_data = data;
        rt_mutex_release(data_mutex);

        rt_thread_mdelay(2000);
    }
}

/* ==================== LVGL UI 全局变量 ==================== */
static lv_obj_t *img_main_status; // 动态切换的警报图标
static lv_obj_t *label_temp_val;  // 温度数值标签
static lv_obj_t *label_humi_val;  // 湿度数值标签
static lv_obj_t *label_light_val; // 光照数值标签
static lv_obj_t *label_tilt_x_val; // X轴倾斜角度标签
static lv_obj_t *label_tilt_y_val; // Y轴倾斜角度标签
static lv_obj_t *label_alarm_status; // 报警状态标签
static lv_obj_t *img_beep_status;   // 蜂鸣器状态图标
static lv_obj_t *img_wifi_status;   // WiFi状态图标
static lv_obj_t *img_cloud_status;  // OneNET状态图标

/* ==================== UI静态布局实现 ==================== */
static void warehouse_ui_init(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);

    /* 1. 顶部状态栏：显示 WiFi 和 OneNET 上传图标 */
    img_wifi_status = lv_img_create(scr);
    lv_img_set_src(img_wifi_status, &connected);
    lv_obj_align(img_wifi_status, LV_ALIGN_TOP_RIGHT, -5, 5);

    img_cloud_status = lv_img_create(scr);
    lv_img_set_src(img_cloud_status, &onenet_upload);
    lv_obj_align_to(img_cloud_status, img_wifi_status, LV_ALIGN_OUT_LEFT_MID, -5, 0);

    /* 2. 中文标题：使用 my_font_cn_16 字库 */
    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &my_font_cn_16, 0);
    lv_obj_set_style_text_color(title, lv_color_black(), 0);
    lv_label_set_text(title, "仓库环境监测");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 10);

    /* 3. 数据展示区：温度 */
    lv_obj_t *img_temp = lv_img_create(scr);
    lv_img_set_src(img_temp, &Environmental);
    lv_obj_align(img_temp, LV_ALIGN_LEFT_MID, 15, -60);

    label_temp_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_temp_val, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
    lv_label_set_text(label_temp_val, "--.-- C");
    lv_obj_align_to(label_temp_val, img_temp, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* 4. 数据展示区：湿度 */
    lv_obj_t *img_humi = lv_img_create(scr);
    lv_img_set_src(img_humi, &waterprof);
    lv_obj_align(img_humi, LV_ALIGN_RIGHT_MID, -15, -60);

    label_humi_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_humi_val, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label_humi_val, lv_color_black(), 0);
    lv_label_set_text(label_humi_val, "--.-- %");
    lv_obj_align_to(label_humi_val, img_humi, LV_ALIGN_OUT_LEFT_MID, -10, 0);

    /* 5. 数据展示区：光照 */
    lv_obj_t *img_light = lv_img_create(scr);
    lv_img_set_src(img_light, &light);
    lv_obj_align(img_light, LV_ALIGN_LEFT_MID, 15, 0);

    label_light_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_light_val, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_light_val, lv_color_black(), 0);
    lv_label_set_text(label_light_val, "--- Lux");
    lv_obj_align_to(label_light_val, img_light, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* 6. 倾斜角度显示区 */
    lv_obj_t *img_tilt = lv_img_create(scr);
    lv_img_set_src(img_tilt, &tilt);
    lv_obj_align(img_tilt, LV_ALIGN_RIGHT_MID, -15, 0);

    label_tilt_x_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_tilt_x_val, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(label_tilt_x_val, lv_color_black(), 0);
    lv_label_set_text(label_tilt_x_val, "X: --.-");
    lv_obj_align_to(label_tilt_x_val, img_tilt, LV_ALIGN_OUT_LEFT_MID, -5, -10);

    label_tilt_y_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_tilt_y_val, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(label_tilt_y_val, lv_color_black(), 0);
    lv_label_set_text(label_tilt_y_val, "Y: --.-");
    lv_obj_align_to(label_tilt_y_val, label_tilt_x_val, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* 7. 底部状态区 */
    label_alarm_status = lv_label_create(scr);
    lv_obj_set_style_text_font(label_alarm_status, &my_font_cn_16, 0);
    lv_obj_set_style_text_color(label_alarm_status, lv_color_black(), 0);
    lv_label_set_text(label_alarm_status, "正常");
    lv_obj_align(label_alarm_status, LV_ALIGN_BOTTOM_LEFT, 10, -10);

    img_main_status = lv_img_create(scr);
    lv_img_set_src(img_main_status, &Environmental);
    lv_obj_align(img_main_status, LV_ALIGN_BOTTOM_MID, 0, -10);

    img_beep_status = lv_img_create(scr);
    lv_img_set_src(img_beep_status, &Megaphone);
    lv_obj_add_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(img_beep_status, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
}

/* ==================== LCD显示线程（LVGL版本） ==================== */
static void display_thread_entry(void *parameter)
{
    struct sensor_data data;
    char buf[32];
    static int beep_on = 0;
    static int blink_count = 0;

    /* 初始化LVGL界面 */
    warehouse_ui_init();

    while (1)
    {
        /* 读取共享数据（加锁） */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        data = shared_data;
        rt_mutex_release(data_mutex);

        /* 更新温度显示 */
        rt_sprintf(buf, "%d.%d C", (int)data.temperature, 
                   abs((int)((data.temperature - (int)data.temperature) * 10)));
        lv_label_set_text(label_temp_val, buf);

        /* 更新湿度显示 */
        rt_sprintf(buf, "%d.%d %%", (int)data.humidity, 
                   abs((int)((data.humidity - (int)data.humidity) * 10)));
        lv_label_set_text(label_humi_val, buf);

        /* 更新光照显示 */
        rt_sprintf(buf, "%d Lux", (int)data.light);
        lv_label_set_text(label_light_val, buf);

        /* 更新倾斜角度显示 */
        rt_sprintf(buf, "X:%d.%d", (int)data.tilt_angle_x, 
                   abs((int)((data.tilt_angle_x - (int)data.tilt_angle_x) * 10)));
        lv_label_set_text(label_tilt_x_val, buf);
        rt_sprintf(buf, "Y:%d.%d", (int)data.tilt_angle_y, 
                   abs((int)((data.tilt_angle_y - (int)data.tilt_angle_y) * 10)));
        lv_label_set_text(label_tilt_y_val, buf);

        /* 更新WiFi状态图标 */
        if (wifi_connected) {
            lv_img_set_src(img_wifi_status, &connected);
        }

        /* 更新蜂鸣器状态图标闪烁 */
        beep_on = (rt_pin_read(BEEP_PIN) == PIN_HIGH) ? 1 : 0;
        blink_count++;
        if (beep_on && (blink_count % 10) < 5) {
            lv_obj_clear_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);
        }

        /* 闭环逻辑：基于温湿度或 ICM20608 姿态数据的 UI 反馈 */
        if (data.temperature > 35.0f) {
            /* 温度过高，显示警报图标并文字变红 */
            lv_img_set_src(img_main_status, &Alarm);
            lv_obj_set_style_text_color(label_temp_val, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(label_alarm_status, "温度过高");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_RED), 0);
        } else if (data.tilt_alarm) {
            /* 检测到货架失稳，显示倾斜图标 */
            lv_img_set_src(img_main_status, &tilt);
            lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
            lv_label_set_text(label_alarm_status, "货架倾斜");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_ORANGE), 0);
        } else {
            /* 恢复正常 */
            lv_img_set_src(img_main_status, &Environmental);
            lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
            lv_label_set_text(label_alarm_status, "正常");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_GREEN), 0);
        }

        /* LVGL任务处理 */
        lv_task_handler();
        
        rt_thread_mdelay(500);
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

/* WiFi事件回调函数前向声明 */
static void wlan_event_callback(int event, struct rt_wlan_buff *buff, void *parameter);

/* ==================== WiFi连接函数 ==================== */
static int wifi_connect(void)
{
    rt_kprintf("Connecting to WiFi: %s\n", WLAN_SSID);

    /* 注册WiFi事件回调 */
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED, wlan_event_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED, wlan_event_callback, RT_NULL);

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
    rt_sprintf(resp_json, "{\"id\":\"%d\",\"code\":200,\"msg\":\"success\"}", request_id);
    
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
            rt_wlan_connect(WLAN_SSID, WLAN_PASSWORD);
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
    char json_buf[128];

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

        /* 使用rt_sprintf构建OneNET物模型标准格式JSON数据 */
        int humidity_int = (int)(data.humidity * 10);
        int light_int = (int)data.light;
        rt_sprintf(json_buf, "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{\"humidity\":{\"value\":%d.%d},\"light\":{\"value\":%d}},\"method\":\"thing.property.post\"}",
                   humidity_int / 10, humidity_int % 10, light_int);

        /* 在串口打印要发送的JSON */
        rt_kprintf("OneNET: sending JSON: %s\n", json_buf);

        /* 使用MQTT直接发布到物模型上传topic（跳过库的自动封装） */
        result = onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post", (uint8_t *)json_buf, rt_strlen(json_buf));
        if (result == 0)
        {
            rt_kprintf("OneNET: upload success\n");
        }
        else
        {
            rt_kprintf("OneNET: upload failed, ret=%d\n", result);
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
    
    /* 初始化ICM20608加速度计 */
    icm20608_dev = icm20608_init("i2c2");
    if (icm20608_dev != RT_NULL)
    {
        /* 校准ICM20608 */
        icm20608_calib_level(icm20608_dev, 100);
        rt_kprintf("ICM20608 initialized and calibrated\n");
    }
    else
    {
        rt_kprintf("ICM20608 init failed!\n");
    }
    
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

    /* 4.5. 初始化 LVGL */
    rt_kprintf("Initializing LVGL...\n");
    lv_init();
    lv_port_disp_init();
    rt_kprintf("LVGL initialized successfully!\n");

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

    /* 自动连接WiFi */
    rt_kprintf("Connecting to WiFi network...\n");
    ret = wifi_connect();
    if (ret != 0)
    {
        rt_kprintf("WiFi connection failed!\n");
    }
    else
    {
        rt_kprintf("WiFi connected successfully!\n");

        /* 初始化 OneNET */
        rt_kprintf("Initializing OneNET...\n");

        ret = onenet_mqtt_init();
        if (ret != 0)
        {
            rt_kprintf("OneNET init failed: %d\n", ret);
        }
        else
        {
            rt_kprintf("OneNET init success\n");

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


