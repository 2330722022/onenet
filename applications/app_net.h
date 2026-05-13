#ifndef APP_NET_H__
#define APP_NET_H__

#include <rtthread.h>

/*
 * net_ready_sem:
 *   信号量 — WiFi 获取 IP 后释放，通知 OneNET 和 HTTP 服务启动。
 *   初始化值 = 0，WiFi 连接成功后 release，消费者 wait 获取。
 *   体现 RT-Thread 线程间同步关系：网络线程（生产者）→ 业务线程（消费者）。
 */
extern rt_sem_t net_ready_sem;

/* WiFi连接状态 */
extern int wifi_connected;

/* 模块初始化 */
void app_net_init(void);

/* WiFi连接 */
int wifi_connect(void);

/* OneNET初始化（由main在WiFi连接后调用）*/
void app_net_onenet_start(void);

#endif
