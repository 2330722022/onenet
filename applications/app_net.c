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

/*
 * 模块：网络通信 (app_net.c)
 * 版本：v2.0 稳定版
 * 日期：2026-05-13
 * OS 概念体现：
 *   1. 信号量 (Semaphore) — net_ready_sem 实现线程间同步。
 *      WiFi 连接线程（生产者）获取 IP 后释放信号量，HTTP Server 和 OneNET
 *      线程（消费者）在启动服务前等待信号量，确保网络就绪后才开始通信。
 *   2. ISR安全命令处理 — onenet_cmd_rsp_cb 仅写 volatile 标志位，
 *      由上传线程在线程上下文中安全消费，避免在不可调度上下文调用阻塞API。
 *   3. 异步 HTTP 处理 — 每个 HTTP 请求创建独立线程处理，主 acceptor 线程
 *      不被阻塞，可同时接受多个连接，体现 OS 处理并发异步事件的能力。
 */

/* ==================== 配置 ==================== */
#define WLAN_SSID       "whu"
#define WLAN_PASSWORD   "99999999"
#define HTTP_PORT       80
#define ENV_UPLOAD_INTERVAL  30000
#define ALARM_UPLOAD_INTERVAL 1000

/* RW007复位引脚（PG15 = GET_PIN(G, 15)）*/
#define RW007_RST_PIN   111

/* LED引脚 */
#define LED_R_PIN       GET_PIN(F, 12)

/* 蜂鸣器引脚（与 app_logic.c 一致）*/
#define BEEP_PIN        GET_PIN(B, 0)

/* ==================== 全局变量 ==================== */
/*
 * net_ready_sem:
 *   信号量 — WiFi 就绪同步原语。
 *   初始值 = 0：消费者（HTTP/OneNET）调用 rt_sem_take() 将阻塞，
 *   直到生产者（WiFi 连接成功）调用 rt_sem_release() 唤醒。
 */
rt_sem_t net_ready_sem = RT_NULL;

int wifi_connected = 0;
static char json_buf[2048];
static char heartbeat[64];
static int mqtt_reconnect_count = 0;

/* HTTP响应头 */
static const char http_header_200[] = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: %d\r\n\r\n";
static const char http_header_503[] = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";

/*
 * 下行命令标志位（ISR安全 — volatile存储，回调只写、线程只读）
 *   说明：OneNET 命令回调可能从 MQTT 内部线程上下文调用，
 *   不能使用 rt_mutex_take (RT_WAITING_FOREVER) 等阻塞操作。
 *   解决办法：回调仅置 volatile 标志位，由上传线程在主循环中消费。
 *
 *   ⚠ 使用独立 uint8_t 而非位域 — 位域共享内存字节，多线程读-改-写会互相覆盖。
 *      ARM Cortex-M 单字节 volatile 读/写是原子的，无竞态。
 */
static volatile uint8_t net_led_pending;
static volatile uint8_t net_led_on;
static volatile uint8_t net_beep_pending;
static volatile uint8_t net_beep_on;
static volatile uint8_t net_threshold_pending;
static volatile float   net_threshold_val;

/* ==================== 发布温度阈值到云端 ==================== */
static rt_err_t publish_temp_threshold(void)
{
    /*
     * 临界区保护：读取 g_app_state.temp_threshold 前持有 data_lock，
     * 确保与按键线程/HTTP接口互斥，读取到一致的阈值快照。
     */
    rt_mutex_take(data_lock, RT_WAITING_FOREVER);
    int int_part = (int)g_app_state.temp_threshold;
    int dec_part = abs((int)((g_app_state.temp_threshold - int_part) * 10));
    rt_mutex_release(data_lock);

    sprintf(json_buf,
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"temp_threshold\":{\"value\":%d.%d}"
        "},\"method\":\"thing.property.post\"}",
        (int)rt_tick_get(), int_part, dec_part);

    rt_kprintf("[net] Publish threshold: %d.%d\n", int_part, dec_part);

    rt_err_t ret = onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                                        (uint8_t *)json_buf, strlen(json_buf));

    return ret;
}

/* ==================== HTTP请求结构体 ==================== */
/*
 * http_request:
 *   封装异步 HTTP 请求的上下文。
 *   主 acceptor 线程接收连接后，将此结构体传递给新创建的 worker 线程，
 *   主线程立即返回继续 accept，不会阻塞。
 */
struct http_request {
    int client_fd;
};

/* ==================== HTTP请求处理器（异步线程） ==================== */
/*
 * http_handler_thread:
 *   每个 HTTP 请求由独立线程处理，体现 OS 的异步事件处理能力。
 *   主 acceptor 线程不阻塞，可同时处理多个并发 HTTP 请求。
 */
static void http_handler_thread(void *parameter)
{
    struct http_request *req = (struct http_request *)parameter;
    int client_fd = req->client_fd;
    rt_free(req);

    char buffer[256];
    int bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read <= 0) { close(client_fd); return; }
    buffer[bytes_read] = '\0';

    if (strstr(buffer, "GET /get_status") != NULL)
    {
        /*
         * 临界区保护：读取 status_json 和 g_app_state 前持有 data_mutex，
         * 防止传感器线程同时写入造成读到不完整的数据。
         */
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        int len = strlen(status_json);
        char header[64];
        rt_snprintf(header, sizeof(header), http_header_200, len);
        send(client_fd, header, strlen(header), 0);
        send(client_fd, status_json, len, 0);
        rt_mutex_release(data_mutex);
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
                    /*
                     * 临界区保护：修改 g_app_state.temp_threshold 时持有 data_lock，
                     * 确保与按键事件处理互斥，防止数据竞争。
                     * 例如：Android 将阈值设为 35.0 的同时按键增加 0.5，
                     * 无锁时可能导致竞态（最终值不确定）。
                     */
                    rt_mutex_take(data_lock, RT_WAITING_FOREVER);
                    g_app_state.temp_threshold = new_threshold;
                    rt_mutex_release(data_lock);

                    update_threshold_display();
                    publish_temp_threshold();

                    int _th_i = (int)new_threshold;
                    int _th_d = abs((int)((new_threshold - _th_i) * 10));
                    char resp[64];
                    rt_snprintf(resp, sizeof(resp),
                                "{\"status\":\"ok\",\"type\":\"threshold\",\"value\":%d.%d}",
                                _th_i, _th_d);
                    char header[64];
                    rt_snprintf(header, sizeof(header), http_header_200, strlen(resp));
                    send(client_fd, header, strlen(header), 0);
                    send(client_fd, resp, strlen(resp), 0);
                    rt_kprintf("[net] /api/set threshold -> %d.%d\n",
                               (int)new_threshold,
                               abs((int)((new_threshold - (int)new_threshold) * 10)));
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
                    /*
                     * 通过 beep_set() 操作 BEEP，内部已持有 data_lock 互斥量，
                     * 确保与逻辑线程/OneNET 下行互斥。
                     */
                    beep_set((uint8_t)new_beep);
                    char resp[64];
                    rt_snprintf(resp, sizeof(resp),
                                "{\"status\":\"ok\",\"type\":\"beep\",\"value\":%u}",
                                new_beep);
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

/* ==================== Web Server 线程 ==================== */
/*
 * web_server_thread_entry:
 *   主 acceptor 线程 — 等待 net_ready_sem 后启动监听。
 *
 *   异步处理模式：
 *     每次 accept 新连接后，创建独立 worker 线程处理请求，
 *     主线程立即返回 accept 下一个连接，不被阻塞。
 *     这体现了 RTOS 处理并发异步事件的核心能力。
 */
static void web_server_thread_entry(void *parameter)
{
    /*
     * 等待信号量 — 阻塞直到 WiFi 获取 IP。
     * rt_sem_take 使本线程挂起，不消耗 CPU，内核在信号量可用时自动唤醒。
     */
    rt_kprintf("[net] HTTP Server waiting for net_ready_sem...\n");
    rt_sem_take(net_ready_sem, RT_WAITING_FOREVER);
    rt_kprintf("[net] net_ready_sem acquired, starting HTTP server...\n");

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

    rt_kprintf("[net] HTTP Server started on port %d (async mode)\n", HTTP_PORT);

    while (1)
    {
        client_len = sizeof(client_addr);
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd >= 0)
        {
            /*
             * 异步处理：为每个 HTTP 请求创建独立线程。
             * 主 acceptor 线程不阻塞，立即返回继续 accept。
             * 这确保了多个并发请求互不干扰。
             */
            struct http_request *req = rt_malloc(sizeof(struct http_request));
            if (req)
            {
                req->client_fd = client_fd;
                rt_thread_t tid = rt_thread_create("http",
                                                     http_handler_thread,
                                                     req,
                                                     2048,
                                                     23,
                                                     10);
                if (tid)
                {
                    rt_thread_startup(tid);
                }
                else
                {
                    rt_free(req);
                    close(client_fd);
                }
            }
            else
            {
                close(client_fd);
            }
        }
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
    int prox = (int)data->proximity;
    int tilt_abs = abs((int)data->tilt_angle_x) > abs((int)data->tilt_angle_y)
                   ? abs((int)data->tilt_angle_x) : abs((int)data->tilt_angle_y);

    rt_mutex_take(data_lock, RT_WAITING_FOREVER);
    int th_int = (int)g_app_state.temp_threshold;
    int th_dec = abs((int)((g_app_state.temp_threshold - th_int) * 10));
    int alarm = g_app_state.alarm_state;
    int beep  = g_app_state.beep_status;
    rt_mutex_release(data_lock);

    sprintf(json_buf,
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"temperature\":{\"value\":%d.%d},"
        "\"humidity\":{\"value\":%d.%d},"
        "\"light\":{\"value\":%d.%d},"
        "\"proximity\":{\"value\":%d},"
        "\"tilt_angle\":{\"value\":%d},"
        "\"alarm_state\":{\"value\":%d},"
        "\"beep\":{\"value\":%s},"
        "\"temp_threshold\":{\"value\":%d.%d}"
        "},\"method\":\"thing.property.post\"}",
        (int)rt_tick_get(),
        temp_int, temp_dec, humi_int, humi_dec, light_int, light_dec,
        prox,
        tilt_abs,
        alarm,
        beep ? "true" : "false",
        th_int, th_dec);

    rt_err_t ret = onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                                        (uint8_t *)json_buf, strlen(json_buf));

    return ret;
}

static rt_err_t upload_alarm_data(struct sensor_data *data, int alarm_state)
{
    int v_int = data->vibration_detected ? 1 : 0;
    int tilt_abs = abs((int)data->tilt_angle_x) > abs((int)data->tilt_angle_y)
                   ? abs((int)data->tilt_angle_x) : abs((int)data->tilt_angle_y);

    rt_mutex_take(data_lock, RT_WAITING_FOREVER);
    int beep_val = g_app_state.beep_status;
    rt_mutex_release(data_lock);

    sprintf(json_buf,
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"vibration\":{\"value\":%d.%d},"
        "\"tilt_angle\":{\"value\":%d},"
        "\"alarm_state\":{\"value\":%d},"
        "\"beep\":{\"value\":%s},"
        "\"fan_status\":{\"value\":%s}"
        "},\"method\":\"thing.property.post\"}",
        (int)rt_tick_get(),
        v_int, 0,
        tilt_abs,
        alarm_state,
        beep_val ? "true" : "false",
        data->actuator_status ? "true" : "false");

    rt_err_t ret = onenet_mqtt_publish("$sys/67k36rzgOO/test1/thing/property/post",
                                        (uint8_t *)json_buf, strlen(json_buf));

    return ret;
}

static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size,
                               uint8_t **resp_data, size_t *resp_size);

/* ==================== OneNET上传线程 ==================== */
/*
 * onenet_upload_thread_entry:
 *   等待 net_ready_sem 信号量 → 初始化 MQTT → 启动数据上传。
 *
 *   同步关系：
 *     WiFi 线程（生产者）释放信号量 → 本线程（消费者）获取信号量后开始工作。
 *
 *   命令处理：
 *     主循环中异步消费 net_xxx 标志位（由 ISR 安全的 onenet_cmd_rsp_cb 生产），
 *     使用 data_lock 互斥量保护 g_app_state 的写入。
 */
static void onenet_upload_thread_entry(void *parameter)
{
    rt_kprintf("[net] OneNET upload waiting for net_ready_sem...\n");
    rt_sem_take(net_ready_sem, RT_WAITING_FOREVER);
    rt_kprintf("[net] net_ready_sem acquired, starting MQTT init...\n");

    /* MQTT 初始化 — 在独立线程上下文中调用，避免 scheduler 断言 */
    int ret = onenet_mqtt_init();
    if (ret != 0)
    {
        rt_kprintf("[net] OneNET MQTT init failed: %d, thread exit\n", ret);
        return;
    }

    onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);
    rt_kprintf("[net] OneNET MQTT OK, entering upload loop\n");

    struct sensor_data data;
    rt_err_t result;
    static int last_alarm_state = 0;
    int current_alarm_state;
    rt_tick_t last_env_upload_time = 0;
    rt_tick_t current_time;

    while (1)
    {
        /* ——— 异步消费下行命令标志位（ISR安全） ——— */
        if (net_led_pending)
        {
            net_led_pending = 0;
            rt_pin_write(LED_R_PIN, net_led_on ? PIN_LOW : PIN_HIGH);
            rt_kprintf("[net] Deferred cmd: LED %s\n", net_led_on ? "ON" : "OFF");
        }

        if (net_beep_pending)
        {
            net_beep_pending = 0;
            uint8_t on = net_beep_on;
            rt_mutex_take(data_lock, RT_WAITING_FOREVER);
            g_app_state.beep_status = on;
            rt_pin_write(BEEP_PIN, on ? PIN_HIGH : PIN_LOW);
            rt_mutex_release(data_lock);
            rt_kprintf("[net] Deferred cmd: beep %s\n", on ? "ON" : "OFF");
        }

        if (net_threshold_pending)
        {
            net_threshold_pending = 0;
            float new_val = net_threshold_val;
            if (new_val >= 20.0f && new_val <= 50.0f)
            {
                rt_mutex_take(data_lock, RT_WAITING_FOREVER);
                g_app_state.temp_threshold = new_val;
                rt_mutex_release(data_lock);
                update_threshold_display();
                rt_kprintf("[net] Deferred cmd: threshold=%d.%d\n",
                           (int)new_val,
                           abs((int)((new_val - (int)new_val) * 10)));
            }
        }
        /* ——— 命令消费结束 ——— */

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
                rt_kprintf("[net] MQTT publish fail #%d, waiting PAHO reconnect...\n",
                           mqtt_reconnect_count);
            }
            if (mqtt_reconnect_count > 5)
            {
                mqtt_reconnect_count = 0;
                rt_kprintf("[net] MQTT long offline, try hard reinit...\n");
                onenet_mqtt_init();
                onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);
                rt_thread_mdelay(3000);
            }
            rt_thread_mdelay(ALARM_UPLOAD_INTERVAL);
            continue;
        }
        mqtt_reconnect_count = 0;

        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        data = shared_data;
        rt_mutex_release(data_mutex);

        rt_mutex_take(data_lock, RT_WAITING_FOREVER);
        float threshold = g_app_state.temp_threshold;
        rt_mutex_release(data_lock);

        if (data.temperature > threshold)
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
/*
 * onenet_cmd_rsp_cb:
 *   处理 OneNET 云端下发的控制命令（LED、BEEP、阈值）。
 *   ⚠ 必须 ISR 安全 — 此回调可能被 paho_mqtt 内部线程调用，
 *     该线程可能在 SPI 中断上下文中处理网络数据包。
 *     因此禁止调用 rt_mutex_take / rt_sem_take 等阻塞 API，
 *     仅通过 volatile 标志位异步通知上传线程。
 */
static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size,
                               uint8_t **resp_data, size_t *resp_size)
{
    const char *data_str = (const char *)recv_data;
    char resp_json[64];
    int request_id = 0;

    const char *id_start = strstr(data_str, "\"id\":");
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

    /* LED 控制 — 纯 volatile 写，ISR 安全 */
    const char *led_start = strstr(data_str, "\"led\":");
    if (led_start)
    {
        led_start += 6;
        while (*led_start == ' ') led_start++;
        net_led_on = (*led_start == 't' || *led_start == 'T' || *led_start == '1') ? 1 : 0;
        net_led_pending = 1;
        rt_kprintf("[net] OneNET cmd: LED %s (deferred)\n", net_led_on ? "ON" : "OFF");
    }

    /* BEEP 控制 — 纯 volatile 写，ISR 安全 */
    const char *beep_start = strstr(data_str, "\"beep\":");
    if (beep_start)
    {
        beep_start += 7;
        while (*beep_start == ' ') beep_start++;
        net_beep_on = (*beep_start == 't' || *beep_start == 'T' || *beep_start == '1') ? 1 : 0;
        net_beep_pending = 1;
        rt_kprintf("[net] OneNET cmd: beep %s (deferred)\n", net_beep_on ? "ON" : "OFF");
    }

    /* 阈值控制 — 纯 volatile 写，ISR 安全 */
    const char *threshold_start = strstr(data_str, "\"temp_threshold\":");
    if (threshold_start)
    {
        threshold_start += 17;
        while (*threshold_start == ' ') threshold_start++;
        float new_threshold = atof(threshold_start);
        if (new_threshold >= 20.0f && new_threshold <= 50.0f)
        {
            net_threshold_val = new_threshold;
            net_threshold_pending = 1;
            rt_kprintf("[net] OneNET cmd: threshold=%d.%d (deferred)\n",
                       (int)new_threshold,
                       abs((int)((new_threshold - (int)new_threshold) * 10)));
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
/*
 * wifi_connect:
 *   连接 WiFi 并释放 net_ready_sem 信号量，通知所有等待的消费者线程。
 *
 *   同步关系：
 *     生产者：本函数在 WiFi 获取 IP 后释放信号量。
 *     消费者：HTTP Server 线程、OneNET 上传线程阻塞等待此信号量。
 */
int wifi_connect(void)
{
    rt_kprintf("[net] Connecting to WiFi: %s\n", WLAN_SSID);

    /*
     * RW007 硬件复位：
     *   拉低 RESET 引脚 200ms → 拉高，确保模块上电稳定。
     *   rt_thread_mdelay 使用系统延时，不阻塞其他线程。
     */
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

            /*
             * 释放信号量 — 唤醒所有等待 net_ready_sem 的消费者线程。
             * 包括：HTTP Server 线程、OneNET 上传线程。
             * 释放2次：每个消费者各取一次，确保两个线程都能被唤醒。
             */
            rt_sem_release(net_ready_sem);  /* 消费者1: HTTP Server */
            rt_sem_release(net_ready_sem);  /* 消费者2: OneNET 上传 */

            return 0;
        }
        rt_thread_mdelay(500);
    }

    rt_kprintf("[net] WiFi connect timeout!\n");
    return -1;
}

/* ==================== 模块初始化 ==================== */
/*
 * app_net_init:
 *   创建信号量并启动 Web Server 线程。
 *   线程启动后立即阻塞等待 net_ready_sem，WiFi 连接成功后自动唤醒。
 */
void app_net_init(void)
{
    /* 创建信号量 — 初始值 0，WiFi 连接成功后 release */
    net_ready_sem = rt_sem_create("net_ready", 0, RT_IPC_FLAG_FIFO);
    if (net_ready_sem == RT_NULL)
    {
        rt_kprintf("[net] FATAL: semaphore create failed!\n");
        return;
    }

    rt_kprintf("[net] net_ready_sem created (value=0)\n");

    /* 启动 Web Server 线程 */
    rt_thread_t tid_web = rt_thread_create("web", web_server_thread_entry, RT_NULL,
                                           2048, 18, 10);
    if (tid_web)
        rt_thread_startup(tid_web);
}

/* ==================== OneNET初始化（由main在WiFi连接后调用）==================== */
/*
 * app_net_onenet_start:
 *   仅启动 OneNET 上传线程。MQTT 初始化、回调注册由线程自身完成。
 *   线程启动后先阻塞等待 net_ready_sem（WiFi 就绪），然后执行 init。
 *   这确保了 onenet_mqtt_init() 在正确的线程上下文（而非 main 线程）中运行。
 */
void app_net_onenet_start(void)
{
    /*
     * 启动 OneNET 上传线程。
     * 线程内部会先等待 net_ready_sem 信号量，
     * 然后初始化 MQTT、注册回调、进入上传循环。
     * 优先级 15 — 高于传感器(25)和HTTP worker(23)，
     * 确保下行命令及时消费，同时不抢占逻辑处理(16)。
     */
    rt_thread_t onenet_thread = rt_thread_create("onenet",
                                                   onenet_upload_thread_entry,
                                                   RT_NULL,
                                                   3072,
                                                   15,
                                                   10);
    if (onenet_thread)
    {
        rt_thread_startup(onenet_thread);
        rt_kprintf("[net] OneNET upload thread started (MQTT init deferred)\n");
    }
}
