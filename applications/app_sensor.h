#ifndef APP_SENSOR_H__
#define APP_SENSOR_H__

#include <rtthread.h>

/* 传感器数据结构 */
struct sensor_data {
    float temperature;
    float humidity;
    float light;
    uint16_t proximity;
    float tilt_angle_x;
    float tilt_angle_y;
    int tilt_alarm;
    int vibration_detected;
    int actuator_status;
    int fan_status;
};

/* 全局JSON缓冲区（零拷贝，供HTTP模块直接引用）*/
extern char status_json[128];
extern char data_json[128];

/* 共享数据与互斥锁 */
extern struct sensor_data shared_data;
extern rt_mutex_t data_mutex;

/* 传感器设备句柄 */
extern void *aht20_dev;
extern void *ap3216c_dev;
extern void *icm20608_dev;

/* 模块初始化 */
void app_sensor_init(void);

#endif
