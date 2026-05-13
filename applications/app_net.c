#include "app_net.h"
#include "app_sensor.h"
#include "app_logic.h"
#include <board.h>
#include <rtdevice.h>
#include <wlan_mgnt.h>
#include <onenet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

/* ==================== 配置 ==================== */
#define WLAN_SSID       "whu"
#define WLAN_PASSWORD   "99999999"
#define HTTP_PORT       80
#define RATE_LIMIT_MS   5000
#define ENV_UPLOAD_INTERVAL  30000
#define ALARM_UPLOAD_INTERVAL 1000

/* RW007复位引脚（PG15 = GET_PIN(G, 15)）*/
#define RW007_RST_PIN   111

/* LED引脚 */
#define LED_R_PIN       GET_PIN(F, 12)

/* ==================== 全局变量 ==================== */
int wifi_connected = 0;
static char json_buf[2048];
static char heartbeat[64];
static int mqtt_reconnect_count = 0;

/* HTTP响应头 */
static const char http_header_200[] = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: %d\r\n\r\n";
static const char http_header_503[] = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";

/* ==================== 发布温度阈值到云端 ==================== */
static rt_err_t publish_temp_threshold(void)
{
    int int_part = (int)temp_threshold;
    int dec_part = abs((int)((temp_threshold - int_part) * 10));

    sprintf(json_buf,
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"temp_threshold\":{\"value\":%d.%d}"
        "},\"method\":\"thing.property.post\"}",
        (int)rt_tick_get(), int_part, dec_part);

    rt_kprintf("[net] Publish threshold: %d.%d\n", int_part, dec_part);
    return onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                               (uint8_t *)json_buf, strlen(json_buf));
}

/* ==================== HTTP请求处理 ==================== */
static void http_client_handler(int client_fd)
{
    char buffer[256];
    int bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read <= 0) { close(client_fd); return; }
    buffer[bytes_read] = '\0';

    if (strstr(buffer, "GET /get_status") != NULL)
    {
        int len = strlen(status_json);
        char header[64];
        rt_snprintf(header, sizeof(header), http_header_200, len);
        send(client_fd, header, strlen(header), 0);
        send(client_fd, status_json, len, 0);
    }
    else if (strstr(buffer, "GET /api/set") != NULL)
    {
        const char *type_param = strstr(buffer, "type=");
        const char *val_param = strstr(buffer, "val=");

        if (type_param != NULL && val_param != NULL)
        {
            type_param += 5;
            val_param += 4;

            char type_buf[16] = {0};
            char val_buf[32] = {0};
            int i = 0;
            while (*type_param != '&' && *type_param != ' ' && *type_param != '\0' && i < 15)
                type_buf[i++] = *type_param++;
            i = 0;
            while (*val_param != '&' && *val_param != ' ' && *val_param != '\0' && i < 31)
                val_buf[i++] = *val_param++;

            if (strcmp(type_buf, "threshold") == 0)
            {
                float new_threshold = atof(val_buf);
                if (new_threshold >= 20.0f && new_threshold <= 50.0f)
                {
                    temp_threshold = new_threshold;
                    update_threshold_display();
                    publish_temp_threshold();

                    int _th_i = (int)temp_threshold, _th_d = (int)((temp_threshold - _th_i) * 10);
                    if (_th_d < 0) _th_d = -_th_d;
                    char resp[64];
                    rt_snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"type\":\"threshold\",\"value\":%d.%d}", _th_i, _th_d);
                    char header[64];
                    rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
                    send(client_fd, header, strlen(header), 0);
                    send(client_fd, resp, strlen(resp), 0);
                    rt_kprintf("[net] /api/set threshold -> %.1f\n", new_threshold);
                }
                else
                {
                    const char *resp = "{\"status\":\"error\",\"msg\":\"invalid threshold range (20-50)\"}";
                    char header[64];
                    rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
                    send(client_fd, header, strlen(header), 0);
                    send(client_fd, resp, strlen(resp), 0);
                }
            }
            else if (strcmp(type_buf, "beep") == 0)
            {
                int new_beep = atoi(val_buf);
                if (new_beep == 0 || new_beep == 1)
                {
                    beep_set((uint8_t)new_beep);
                    char resp[64];
                    rt_snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"type\":\"beep\",\"value\":%u}", new_beep);
                    char header[64];
                    rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
                    send(client_fd, header, strlen(header), 0);
                    send(client_fd, resp, strlen(resp), 0);
                    rt_kprintf("[net] /api/set beep -> %s\n", new_beep ? "ON" : "OFF");
                }
                else
                {
                    const char *resp = "{\"status\":\"error\",\"msg\":\"invalid beep value (0 or 1)\"}";
                    char header[64];
                    rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
                    send(client_fd, header, strlen(header), 0);
                    send(client_fd, resp, strlen(resp), 0);
                }
            }
            else
            {
                const char *resp = "{\"status\":\"error\",\"msg\":\"unknown type\"}";
                char header[64];
                rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
                send(client_fd, header, strlen(header), 0);
                send(client_fd, resp, strlen(resp), 0);
            }
        }
        else
        {
            const char *resp = "{\"status\":\"error\",\"msg\":\"missing parameters\"}";
            char header[64];
            rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
            send(client_fd, header, strlen(header), 0);
            send(client_fd, resp, strlen(resp), 0);
        }
    }
    else
    {
        const char *not_found = "HTTP/1.1 404 Not Found\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
        send(client_fd, not_found, strlen(not_found), 0);
    }
    close(client_fd);
}

/* Web Server 线程 */
static void web_server_thread_entry(void *parameter)
{
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len;
    int opt = 1;

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) { rt_kprintf("[net] socket failed\n"); return; }
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(HTTP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    { rt_kprintf("[net] bind failed\n"); close(server_fd); return; }
    if (listen(server_fd, 5) < 0)
    { rt_kprintf("[net] listen failed\n"); close(server_fd); return; }

    rt_kprintf("[net] HTTP Server started on port %d\n", HTTP_PORT);

    while (1)
    {
        client_len = sizeof(client_addr);
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd >= 0)
            http_client_handler(client_fd);
        rt_thread_mdelay(10);
    }
}

/* ==================== OneNET数据上传 ==================== */
static rt_err_t upload_env_data(struct sensor_data *data)
{
    int temp_int = (int)data->temperature;
    int temp_dec = abs((int)((data->temperature - temp_int) * 10));
    int humi_int = (int)data->humidity;
    int humi_dec = abs((int)((data->humidity - humi_int) * 10));
    int light_int = (int)data->light;
    int light_dec = abs((int)((data->light - light_int) * 10));
    int th_int = (int)temp_threshold;
    int th_dec = abs((int)((temp_threshold - th_int) * 10));

    sprintf(json_buf,
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"temperature\":{\"value\":%d.%d},"
        "\"humidity\":{\"value\":%d.%d},"
        "\"light\":{\"value\":%d.%d},"
        "\"temp_threshold\":{\"value\":%d.%d}"
        "},\"method\":\"thing.property.post\"}",
        (int)rt_tick_get(),
        temp_int, temp_dec, humi_int, humi_dec, light_int, light_dec, th_int, th_dec);

    return onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                               (uint8_t *)json_buf, strlen(json_buf));
}

static rt_err_t upload_alarm_data(struct sensor_data *data, int alarm_state)
{
    int v_int = data->vibration_detected ? 1 : 0;
    float tilt_sq = data->tilt_angle_x * data->tilt_angle_x + data->tilt_angle_y * data->tilt_angle_y;
    int tilt_int = (tilt_sq >= 0 && tilt_sq < 1000000) ? (int)sqrt(tilt_sq) : 0;

    sprintf(json_buf,
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"vibration\":{\"value\":%d.%d},"
        "\"tilt_angle\":{\"value\":%d},"
        "\"alarm_state\":{\"value\":%d},"
        "\"fan_status\":{\"value\":%s}"
        "},\"method\":\"thing.property.post\"}",
        (int)rt_tick_get(),
        v_int, 0, tilt_int, alarm_state,
        data->actuator_status ? "true" : "false");

    return onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                               (uint8_t *)json_buf, strlen(json_buf));
}

/* ==================== OneNET上传线程 ==================== */
static void onenet_upload_thread_entry(void *parameter)
{
    struct sensor_data data;
    rt_err_t result;
    static int last_alarm_state = 0;
    int current_alarm_state;
    rt_tick_t last_env_upload_time = 0;
    rt_tick_t current_time;

    rt_kprintf("[net] OneNET upload thread started\n");

    while (1)
    {
        current_time = rt_tick_get();

        if (!wifi_connected)
        {
            rt_thread_mdelay(2000);
            continue;
        }

        rt_snprintf(heartbeat, sizeof(heartbeat),
                    "{\"id\":\"hb\",\"version\":\"1.0\",\"params\":{},\"method\":\"thing.property.post\"}");
        int mqtt_result = onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                                              (uint8_t *)heartbeat, rt_strlen(heartbeat));

        if (mqtt_result != 0)
        {
            mqtt_reconnect_count++;
            if (mqtt_reconnect_count <= 3)
            {
                rt_thread_mdelay(1000);
                if (onenet_mqtt_init() == 0)
                {
                    mqtt_reconnect_count = 0;
                }
            }
            else
            {
                mqtt_reconnect_count = 0;
                rt_thread_mdelay(5000);
            }
            rt_thread_mdelay(ALARM_UPLOAD_INTERVAL);
            continue;
        }
        mqtt_reconnect_count = 0;

        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        data = shared_data;
        rt_mutex_release(data_mutex);

        if (data.temperature > temp_threshold)
            current_alarm_state = 1;
        else if (data.vibration_detected)
            current_alarm_state = 2;
        else if (data.tilt_alarm)
            current_alarm_state = 3;
        else
            current_alarm_state = 0;

        if (current_time - last_env_upload_time >= ENV_UPLOAD_INTERVAL)
        {
            result = upload_env_data(&data);
            if (result == 0)
            {
                last_env_upload_time = current_time;
                rt_kprintf("[net] ENV upload OK\n");
            }
        }

        if (current_alarm_state != last_alarm_state)
        {
            result = upload_alarm_data(&data, current_alarm_state);
            if (result == 0)
            {
                last_alarm_state = current_alarm_state;
                rt_kprintf("[net] ALARM state %d uploaded\n", current_alarm_state);
            }
        }

        rt_thread_mdelay(ALARM_UPLOAD_INTERVAL);
    }
}

/* ==================== OneNET下行控制回调 ==================== */
static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size)
{
    const char *data = (const char *)recv_data;
    char resp_json[64];
    int request_id = 0;

    const char *id_start = strstr(data, "\"id\":");
    if (id_start)
    {
        id_start += 5;
        while (*id_start == ' ' || *id_start == '\"') id_start++;
        request_id = atoi(id_start);
    }

    rt_sprintf(resp_json, "{\"id\":\"%d\",\"code\":200,\"msg\":\"success\"}", request_id);
    *resp_data = (uint8_t *)rt_malloc(strlen(resp_json) + 1);
    if (*resp_data)
    {
        strcpy((char *)*resp_data, resp_json);
        *resp_size = strlen(resp_json);
    }

    const char *led_start = strstr(data, "\"led\":");
    if (led_start)
    {
        led_start += 6;
        while (*led_start == ' ') led_start++;
        if (*led_start == 't' || *led_start == 'T' || *led_start == '1')
        {
            rt_pin_write(LED_R_PIN, PIN_LOW);
            rt_kprintf("[net] OneNET cmd: LED ON\n");
        }
        else
        {
            rt_pin_write(LED_R_PIN, PIN_HIGH);
            rt_kprintf("[net] OneNET cmd: LED OFF\n");
        }
    }

    const char *beep_start = strstr(data, "\"beep\":");
    if (beep_start)
    {
        beep_start += 7;
        while (*beep_start == ' ') beep_start++;
        beep_set((*beep_start == 't' || *beep_start == 'T' || *beep_start == '1') ? 1 : 0);
        rt_kprintf("[net] OneNET cmd: beep %s\n", beep_status ? "ON" : "OFF");
    }

    const char *threshold_start = strstr(data, "\"temp_threshold\":");
    if (threshold_start)
    {
        threshold_start += 17;
        while (*threshold_start == ' ') threshold_start++;
        float new_threshold = atof(threshold_start);
        if (new_threshold >= 20.0f && new_threshold <= 50.0f)
        {
            temp_threshold = new_threshold;
            update_threshold_display();
            rt_kprintf("[net] OneNET cmd: threshold=%.1f\n", new_threshold);
        }
    }
}

/* ==================== WiFi事件回调 ==================== */
static void wlan_event_callback(int event, struct rt_wlan_buff *buff, void *parameter)
{
    switch (event)
    {
        case RT_WLAN_EVT_STA_CONNECTED:
            rt_kprintf("[net] WiFi connected to AP\n");
            break;
        case RT_WLAN_EVT_STA_DISCONNECTED:
            rt_kprintf("[net] WiFi disconnected, reconnecting...\n");
            wifi_connected = 0;
            rt_wlan_connect(WLAN_SSID, WLAN_PASSWORD);
            break;
        default:
            break;
    }
}

/* ==================== WiFi连接 ==================== */
int wifi_connect(void)
{
    rt_kprintf("[net] Connecting to WiFi: %s\n", WLAN_SSID);

    /* RW007硬件复位：拉低200ms再拉高 */
    rt_pin_mode(RW007_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RW007_RST_PIN, PIN_LOW);
    rt_thread_mdelay(200);
    rt_pin_write(RW007_RST_PIN, PIN_HIGH);
    rt_kprintf("[net] RW007 reset done, waiting 3s for stabilization...\n");
    rt_thread_mdelay(3000);

    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED, wlan_event_callback, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED, wlan_event_callback, RT_NULL);

    rt_wlan_connect(WLAN_SSID, WLAN_PASSWORD);

    for (int i = 0; i < 60; i++)
    {
        if (rt_wlan_is_connected())
        {
            rt_kprintf("[net] WiFi connected!\n");
            wifi_connected = 1;
            return 0;
        }
        rt_thread_mdelay(500);
    }

    rt_kprintf("[net] WiFi connect timeout!\n");
    return -1;
}

/* ==================== 模块初始化 ==================== */
void app_net_init(void)
{
    rt_thread_t tid_web = rt_thread_create("web", web_server_thread_entry, RT_NULL,
                                           2048, 18, 10);
    if (tid_web)
        rt_thread_startup(tid_web);
}

/* ==================== OneNET初始化（由main在WiFi连接后调用）==================== */
void app_net_onenet_start(void)
{
    int ret = onenet_mqtt_init();
    if (ret != 0)
    {
        rt_kprintf("[net] OneNET init failed: %d\n", ret);
        return;
    }

    onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);
    rt_kprintf("[net] OneNET OK, starting upload thread...\n");

    rt_thread_t onenet_thread = rt_thread_create("onenet", onenet_upload_thread_entry, RT_NULL,
                                                  3072, 25, 10);
    if (onenet_thread)
    {
        rt_thread_startup(onenet_thread);
        rt_kprintf("[net] OneNET upload thread started\n");
    }
}
