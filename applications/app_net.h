#ifndef APP_NET_H__
#define APP_NET_H__

#include <rtthread.h>

/* WiFi连接状态 */
extern int wifi_connected;

/* 模块初始化 */
void app_net_init(void);

/* WiFi连接 */
int wifi_connect(void);

/* OneNET初始化（由main在WiFi连接后调用）*/
void app_net_onenet_start(void);

#endif
