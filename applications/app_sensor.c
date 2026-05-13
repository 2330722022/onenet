#include "app_sensor.h"
#include "app_logic.h"
#include <board.h>
#include <rtdevice.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "aht10.h"
#include "ap3216c.h"
#include "icm20608.h"

/* ==================== ICM20608倾斜监测配置 ==================== */
#define TILT_THRESHOLD     15
#define TILT_SENSITIVITY   2

/* ==================== 全局变量定义 ==================== */
struct sensor_data shared_data;
rt_mutex_t data_mutex = RT_NULL;

char status_json[128];
char data_json[128];

void *aht20_dev = RT_NULL;
void *ap3216c_dev = RT_NULL;
void *icm20608_dev = RT_NULL;

/* ==================== 倾斜角度计算 ==================== */
static void calculate_tilt_angle(rt_int16_t accel_x, rt_int16_t accel_y, rt_int16_t accel_z,
                                  float *angle_x, float *angle_y)
{
    float g = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (g < 100)
    {
        *angle_x = 0;
        *angle_y = 0;
        return;
    }
    *angle_x = atan2((float)accel_y, (float)accel_z) * 180 / 3.14159f;
    *angle_y = atan2((float)accel_x, (float)accel_z) * 180 / 3.14159f;
}

/* ==================== 传感器采集线程 ==================== */
static void sensor_thread_entry(void *parameter)
{
    struct sensor_data data;
    rt_int16_t accel_x, accel_y, accel_z;
    rt_int16_t gyro_x, gyro_y, gyro_z;
    static int tilt_warning_count = 0;
    #define VIBRATION_THRESHOLD 1000

    while (1)
    {
        rt_memset(&data, 0, sizeof(struct sensor_data));

        data.temperature = aht10_read_temperature((aht10_device_t)aht20_dev);
        data.humidity    = aht10_read_humidity((aht10_device_t)aht20_dev);
        data.light       = ap3216c_read_ambient_light((ap3216c_device_t)ap3216c_dev);
        data.proximity   = ap3216c_read_ps_data((ap3216c_device_t)ap3216c_dev);

        if (icm20608_dev != RT_NULL)
        {
            if (icm20608_get_accel((icm20608_device_t)icm20608_dev, &accel_x, &accel_y, &accel_z) == RT_EOK)
            {
                calculate_tilt_angle(accel_x, accel_y, accel_z,
                                     &data.tilt_angle_x, &data.tilt_angle_y);

                if (abs(data.tilt_angle_x) > TILT_THRESHOLD ||
                    abs(data.tilt_angle_y) > TILT_THRESHOLD)
                {
                    tilt_warning_count++;
                    if (tilt_warning_count >= TILT_SENSITIVITY)
                    {
                        data.tilt_alarm = 1;
                        rt_kprintf("[sensor] ALERT: Shelf tilted! X:%d Y:%d\n",
                                   (int)data.tilt_angle_x, (int)data.tilt_angle_y);
                    }
                }
                else
                {
                    tilt_warning_count = 0;
                    data.tilt_alarm = 0;
                }
            }
            else
            {
                data.tilt_angle_x = 0;
                data.tilt_angle_y = 0;
                data.tilt_alarm = 0;
            }

            if (icm20608_get_gyro((icm20608_device_t)icm20608_dev, &gyro_x, &gyro_y, &gyro_z) == RT_EOK)
            {
                data.vibration_detected = (abs(gyro_x) > VIBRATION_THRESHOLD ||
                                           abs(gyro_y) > VIBRATION_THRESHOLD ||
                                           abs(gyro_z) > VIBRATION_THRESHOLD) ? 1 : 0;
            }
            else
            {
                data.vibration_detected = 0;
            }
        }

        /* 处理按键事件 */
        process_key_events();

        /* 调用业务逻辑进行阈值判定 */
        logic_process(data.temperature);

        /* 同步状态 */
        data.actuator_status = beep_status;

        /* 更新共享数据 */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        shared_data = data;

        /* 构建静态JSON：/get_status 响应 */
        int alarm_state = (data.temperature > temp_threshold) || data.tilt_alarm || data.vibration_detected;
        int _th_i = (int)temp_threshold, _th_d = (int)((temp_threshold - _th_i) * 10);
        if (_th_d < 0) _th_d = -_th_d;
        rt_snprintf(status_json, sizeof(status_json),
                    "{\"alarm_state\":%d,\"beep_status\":%u,\"fan_status\":%u,\"work_mode\":%d,\"temp_threshold\":%d.%d}",
                    alarm_state, beep_status, data.fan_status, auto_mode, _th_i, _th_d);

        /* 构建静态JSON：Android轮询响应 */
        int temp_int = (int)data.temperature;
        int temp_dec = abs((int)((data.temperature - temp_int) * 10));
        int humi_int = (int)data.humidity;
        int humi_dec = abs((int)((data.humidity - humi_int) * 10));
        rt_snprintf(data_json, sizeof(data_json),
                    "{\"temp\":%d.%d,\"humi\":%d.%d,\"threshold\":%d.%d}",
                    temp_int, temp_dec, humi_int, humi_dec, _th_i, _th_d);

        rt_mutex_release(data_mutex);

        rt_thread_mdelay(5000);
    }
}

/* ==================== 传感器初始化 ==================== */
void app_sensor_init(void)
{
    /* 创建互斥量 */
    data_mutex = rt_mutex_create("data_mutex", RT_IPC_FLAG_FIFO);

    /* 初始化传感器硬件 */
    aht20_dev = (void *)aht10_init("i2c3");
    ap3216c_dev = (void *)ap3216c_init("i2c2");
    icm20608_dev = (void *)icm20608_init("i2c2");

    if (icm20608_dev != RT_NULL)
    {
        icm20608_calib_level((icm20608_device_t)icm20608_dev, 100);
        rt_kprintf("[sensor] ICM20608 calibrated\n");
    }

    if (aht20_dev == RT_NULL || ap3216c_dev == RT_NULL)
    {
        rt_kprintf("[sensor] FATAL: AHT10 or AP3216C init failed!\n");
        return;
    }

    /* 创建采集线程 */
    rt_thread_t tid = rt_thread_create("sensor", sensor_thread_entry, RT_NULL,
                                        2048, 20, 10);
    if (tid)
    {
        rt_thread_startup(tid);
        rt_kprintf("[sensor] Collection thread started (interval=5s)\n");
    }
}
