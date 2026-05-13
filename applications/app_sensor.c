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

/*
 * 模块：传感器采集 (app_sensor.c)
 * 版本：v2.0 稳定版
 * 日期：2026-05-13
 * OS 概念体现：
 *   1. 多线程调度 — 采集线程设为优先级25（低优先级），让出 CPU 给显示(21)
 *      和逻辑处理(16)线程。OS 按优先级抢占调度，高优先级就绪时低优先级自动挂起。
 *   2. 事件集 — 采用生产者-消费者模式：传感器线程(生产者)检测到温度/倾斜/振动异常时
 *      发送 EVENT_ALARM，逻辑线程(消费者)阻塞等待，实现异步解耦。
 *   3. 总线驱动管理 — AHT10 挂载 I2C3，ICM20608 挂载 I2C2，RT-Thread 设备框架
 *      通过统一的 rt_device 接口管理不同总线设备，上层无需关心底层时序。
 */

/*
 * ==================== 硬件选型说明 ====================
 *
 * [AHT10 温湿度传感器 — I2C3 总线]
 *   选型依据：
 *     - 数字输出，无需额外 ADC，精度 ±0.3°C / ±2%RH
 *     - I2C 接口仅需 2 根线（SCL/SDA），节省引脚
 *     - RT-Thread AHT10 驱动已适配，直接调用 aht10_read_temperature/humidity
 *   为何 I2C3：
 *     - I2C1 被 AP3216C 和 ICM20608 共享（同总线多设备）
 *     - I2C2 留给扩展
 *     - I2C3 独立挂载 AHT10，避免总线冲突
 *
 * [AP3216C 环境光/接近传感器 — I2C2 总线]
 *   选型依据：
 *     - 集成 ALS（环境光）+ PS（接近检测），单芯片双功能
 *     - I2C 地址 0x1E，与 ICM20608(0x68) 不冲突，可共享 I2C2
 *
 * [ICM20608 6轴IMU — I2C2 总线]
 *   选型依据：
 *     - 集成 3轴加速度计 + 3轴陀螺仪，适合姿态检测
 *     - 加速度计用于计算倾斜角（atan2），陀螺仪用于检测振动
 *     - I2C 地址 0x68，与 AP3216C(0x1E) 不冲突
 *   为何 I2C 而非 SPI：
 *     - SPI2 已被 RW007 WiFi 模块独占，不可复用
 *     - ICM20608 支持 I2C 接口，共享 I2C2 总线减少引脚占用
 *     - I2C 轮询速率（100kHz）足够满足环境监测的 5 秒采样周期
 *
 * [总线资源分配总结]
 *   I2C1: 保留
 *   I2C2: AP3216C (0x1E) + ICM20608 (0x68) — 同总线双设备，地址隔离
 *   I2C3: AHT10 (0x38) — 独立总线
 *   SPI2: RW007 WiFi — 独占
 */

/* ==================== ICM20608倾斜监测配置 ==================== */
#define TILT_THRESHOLD     15
#define TILT_SENSITIVITY   2

/* ==================== 全局变量定义 ==================== */
struct sensor_data shared_data;
rt_mutex_t data_mutex = RT_NULL;

char status_json[256];
char data_json[256];

void *aht20_dev = RT_NULL;
void *ap3216c_dev = RT_NULL;
void *icm20608_dev = RT_NULL;

/* ==================== 倾斜角度计算 ==================== */
/*
 * calculate_tilt_angle:
 *   利用加速度计的三轴分量计算 X/Y 轴倾角。
 *   公式：angle = atan2(垂直分量, 重力方向分量) * 180/π
 *   当 g < 100（接近自由落体）时返回 0，避免异常值。
 */
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
/*
 * 优先级说明：
 *   采集线程优先级设为 25（最低），确保不干扰：
 *     - 显示线程(21) — LVGL 需要及时刷新屏幕
 *     - 逻辑线程(16) — 告警响应需要低延迟
 *     - Web Server(18) — 网络 I/O 需要及时处理
 *   RT-Thread 抢占式调度确保高优先级线程就绪时立即抢占 CPU。
 */
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

        /* 读取温度 & 湿度 (I2C3 — AHT10) */
        data.temperature = aht10_read_temperature((aht10_device_t)aht20_dev);
        data.humidity    = aht10_read_humidity((aht10_device_t)aht20_dev);

        /* 读取光照 & 接近 (I2C2 — AP3216C) */
        data.light       = ap3216c_read_ambient_light((ap3216c_device_t)ap3216c_dev);
        data.proximity   = ap3216c_read_ps_data((ap3216c_device_t)ap3216c_dev);

        /* 读取加速度计 + 陀螺仪 (I2C2 — ICM20608) */
        if (icm20608_dev != RT_NULL)
        {
            if (icm20608_get_accel((icm20608_device_t)icm20608_dev,
                                   &accel_x, &accel_y, &accel_z) == RT_EOK)
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
                        /*
                         * 检测到倾斜告警 → 发送事件通知逻辑线程。
                         * 事件驱动模型：传感器线程不直接操作 BEEP/舵机，
                         * 而是通过事件集异步通知专用的逻辑线程处理。
                         */
                        rt_event_send(&evt_alarm, EVENT_TILT_ALARM);
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

            if (icm20608_get_gyro((icm20608_device_t)icm20608_dev,
                                  &gyro_x, &gyro_y, &gyro_z) == RT_EOK)
            {
                int vib = (abs(gyro_x) > VIBRATION_THRESHOLD ||
                           abs(gyro_y) > VIBRATION_THRESHOLD ||
                           abs(gyro_z) > VIBRATION_THRESHOLD) ? 1 : 0;
                data.vibration_detected = vib;

                if (vib)
                {
                    /*
                     * 检测到振动告警 → 发送事件通知逻辑线程。
                     */
                    rt_event_send(&evt_alarm, EVENT_VIBRATION_ALARM);
                    rt_kprintf("[sensor] ALERT: Vibration detected!\n");
                }
            }
            else
            {
                data.vibration_detected = 0;
            }
        }

        /* 处理按键事件（WK_UP/DOWN 调整阈值）*/
        process_key_events();

        /* 调用业务逻辑中心进行温度阈值判定 */
        logic_handle(data.temperature);

        /* 同步 actuator 状态 */
        rt_mutex_take(data_lock, RT_WAITING_FOREVER);
        data.actuator_status = g_app_state.beep_status;
        float local_threshold = g_app_state.temp_threshold;
        rt_mutex_release(data_lock);

        /* 更新共享数据 — 临界区保护 */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        shared_data = data;

        /* 构建 /get_status JSON — 供 HTTP/OneNET 零拷贝引用 */
        {
            int alarm_state = (data.temperature > local_threshold)
                              || data.tilt_alarm || data.vibration_detected;
            int _th_i = (int)local_threshold;
            int _th_d = (int)((local_threshold - _th_i) * 10);
            if (_th_d < 0) _th_d = -_th_d;
            rt_snprintf(status_json, sizeof(status_json),
                        "{\"alarm_state\":%d,\"beep\":%u,\"fan_status\":%u,"
                        "\"work_mode\":%d,\"temp_threshold\":%d.%d}",
                        alarm_state, g_app_state.beep_status, data.fan_status, 1,
                        _th_i, _th_d);
        }

        /* 构建 data JSON — Android 轮询响应 */
        {
            int temp_int = (int)data.temperature;
            int temp_dec = abs((int)((data.temperature - temp_int) * 10));
            int humi_int = (int)data.humidity;
            int humi_dec = abs((int)((data.humidity - humi_int) * 10));
            int _th_i = (int)local_threshold;
            int _th_d = (int)((local_threshold - _th_i) * 10);
            if (_th_d < 0) _th_d = -_th_d;
            rt_snprintf(data_json, sizeof(data_json),
                        "{\"temp\":%d.%d,\"humi\":%d.%d,\"threshold\":%d.%d}",
                        temp_int, temp_dec, humi_int, humi_dec, _th_i, _th_d);
        }

        rt_mutex_release(data_mutex);

        /* 5 秒采样周期 — 环境监测无需高速采集 */
        rt_thread_mdelay(5000);
    }
}

/* ==================== 传感器初始化 ==================== */
/*
 * app_sensor_init:
 *   创建互斥量保护共享数据，初始化传感器硬件，启动采集线程。
 *
 *   初始化序列：
 *     1. 创建 data_mutex — 保护 shared_data、status_json、data_json
 *     2. 初始化 I2C 总线传感器（AHT10、AP3216C、ICM20608）
 *     3. 校准 ICM20608（消除零偏）
 *     4. 启动采集线程（优先级 25，低优先级）
 */
void app_sensor_init(void)
{
    /* 1. 创建互斥量 */
    data_mutex = rt_mutex_create("data_mutex", RT_IPC_FLAG_FIFO);
    if (data_mutex == RT_NULL)
    {
        rt_kprintf("[sensor] FATAL: data_mutex create failed!\n");
        return;
    }

    /* 2. 初始化传感器硬件 */
    aht20_dev = (void *)aht10_init("i2c3");
    if (aht20_dev == RT_NULL)
        rt_kprintf("[sensor] WARN: AHT10 init failed\n");

    ap3216c_dev = (void *)ap3216c_init("i2c2");
    if (ap3216c_dev == RT_NULL)
        rt_kprintf("[sensor] WARN: AP3216C init failed\n");

    icm20608_dev = (void *)icm20608_init("i2c2");
    if (icm20608_dev != RT_NULL)
    {
        /* 3. 校准 — 采集 100 个样本计算零偏 */
        icm20608_calib_level((icm20608_device_t)icm20608_dev, 100);
        rt_kprintf("[sensor] ICM20608 calibrated\n");
    }
    else
    {
        rt_kprintf("[sensor] WARN: ICM20608 init failed\n");
    }

    /* 4. 启动采集线程 — 优先级25（低优先级）*/
    /*
     * 优先级 25：低于显示线程(21)、Web Server(18)、逻辑线程(16)，
     * 确保传感器采集不阻塞 UI 刷新和网络响应。
     */
    rt_thread_t tid = rt_thread_create("sensor",
                                        sensor_thread_entry,
                                        RT_NULL,
                                        3072,
                                        25,     /* 优先级 25 — 体现低优先级调度 */
                                        10);
    if (tid)
    {
        rt_thread_startup(tid);
        rt_kprintf("[sensor] Thread started (prio=25, interval=5s)\n");
    }
}
