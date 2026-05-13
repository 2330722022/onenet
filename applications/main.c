#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*
 * main.c — 项目入口
 *
 * 版本：v2.0 稳定版
 * 日期：2026-05-13
 * 说明：本版本基于 RT-Thread 操作系统，使用多线程、互斥量、信号量、事件集
 *       等 OS 原语实现各功能模块的协同工作，系统稳定运行。
 *
 * RT-Thread 作为资源调度者的入口职能：
 *   - 不包含业务逻辑代码
 *   - 仅负责 RT-Thread 同步/互斥对象的创建（由各模块 init 函数内部完成）
 *   - 按正确顺序启动各功能模块线程
 *   - 内存监测：记录系统启动前后的内存使用情况
 *
 * ⚠ LVGL 线程安全：
 *   BSP 的 lv_port_disp_init() 内部创建了专用的 "LVGL" 线程，唯一负责调用
 *   lv_timer_handler()。我们通过 lv_timer_create() 注册周期性回调，
 *   回调运行在 LVGL 线程上下文中，确保所有 LVGL 操作单线程安全。
 *
 * 线程优先级规划：
 *   8  — 逻辑处理线程 (app_logic)     【紧急：温度/倾斜告警立即触发BEEP+舵机】
 *   10 — paho_mqtt 线程 (PAHO内部)
 *   15 — OneNET 上传线程 (app_net)
 *   18 — Web Server 线程 (app_net)
 *   20 — LVGL 线程 (BSP内部)
 *   23 — HTTP worker 线程 (app_net)
 *   25 — 传感器采集线程 (app_sensor)
 */

/* 模块头文件 */
#include "app_logic.h"
#include "app_sensor.h"
#include "app_net.h"

/* LVGL */
#include "lv_conf.h"
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

void lv_port_disp_init(void);
LV_FONT_DECLARE(my_font_cn_16);

extern const lv_img_dsc_t Environmental;
extern const lv_img_dsc_t waterprof;
extern const lv_img_dsc_t light;
extern const lv_img_dsc_t Alarm;
extern const lv_img_dsc_t tilt;
extern const lv_img_dsc_t connected;
extern const lv_img_dsc_t onenet_upload;
extern const lv_img_dsc_t Megaphone;

/* ==================== 引脚定义 ==================== */
#define LED_R_PIN       GET_PIN(F, 12)

/* ==================== 内存监测 ==================== */
static void print_memory(const char *tag)
{
    rt_size_t total, used, max_used;
    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("[MEM] %s - Total: %u KB, Used: %u KB (%u%%), Max Used: %u KB (%u%%)\n",
               tag, total / 1024, used / 1024, (used * 100) / total,
               max_used / 1024, (max_used * 100) / total);
}

/* ==================== LVGL UI 对象 ==================== */
static lv_obj_t *img_main_status;
static lv_obj_t *label_temp_val;
static lv_obj_t *label_humi_val;
static lv_obj_t *label_light_val;
static lv_obj_t *label_tilt_x_val;
static lv_obj_t *label_tilt_y_val;
static lv_obj_t *label_alarm_status;
static lv_obj_t *img_beep_status;
static lv_obj_t *img_wifi_status;
static lv_obj_t *img_cloud_status;
static lv_obj_t *label_threshold_val = RT_NULL;
static volatile uint8_t threshold_dirty = 1;

/* ==================== 阈值LCD显示更新（仅设脏标志，LVGL操作由lv_timer回调安全执行）==================== */
void update_threshold_display(void)
{
    threshold_dirty = 1;
}

/* ==================== lv_timer 回调：唯一更新LVGL对象的入口 ==================== */
/*
 * 此回调由 BSP 的 LVGL 线程在 lv_timer_handler() 内调用，是唯一操作 LVGL 对象的地方。
 * 所有其他线程（传感器/HTTP/OneNET/按键）只通过 volatile 标志位 + 共享数据间接影响。
 */
static void lv_ui_refresh_task(lv_timer_t *timer)
{
    struct sensor_data local_data;
    char buf[32];

    /* 读取传感器数据 — 非阻塞尝试，传感器线程持有锁时跳过本周期 */
    if (rt_mutex_take(data_mutex, 1) == RT_EOK) {
        local_data = shared_data;
        rt_mutex_release(data_mutex);
    } else {
        return; /* 数据未就绪，下一轮再刷新 */
    }

    /* 温度 */
    rt_sprintf(buf, "%d.%d C", (int)local_data.temperature,
               abs((int)((local_data.temperature - (int)local_data.temperature) * 10)));
    lv_label_set_text(label_temp_val, buf);

    /* 湿度 */
    rt_sprintf(buf, "%d.%d %%", (int)local_data.humidity,
               abs((int)((local_data.humidity - (int)local_data.humidity) * 10)));
    lv_label_set_text(label_humi_val, buf);

    /* 光照 */
    rt_sprintf(buf, "%d Lux", (int)local_data.light);
    lv_label_set_text(label_light_val, buf);

    /* 倾斜角 X/Y */
    rt_sprintf(buf, "X:%d.%d", (int)local_data.tilt_angle_x,
               abs((int)((local_data.tilt_angle_x - (int)local_data.tilt_angle_x) * 10)));
    lv_label_set_text(label_tilt_x_val, buf);
    rt_sprintf(buf, "Y:%d.%d", (int)local_data.tilt_angle_y,
               abs((int)((local_data.tilt_angle_y - (int)local_data.tilt_angle_y) * 10)));
    lv_label_set_text(label_tilt_y_val, buf);

    /* WiFi状态 */
    if (wifi_connected)
        lv_img_set_src(img_wifi_status, &connected);

    /* 读取业务状态 — 短超时尝试（数据锁由高优先级逻辑线程持有，短暂等待） */
    if (rt_mutex_take(data_lock, 2) == RT_EOK) {
        uint8_t beep = g_app_state.beep_status;
        float threshold = g_app_state.temp_threshold;

        /* 阈值LCD刷新 */
        if (threshold_dirty) {
            threshold_dirty = 0;
            if (label_threshold_val != RT_NULL) {
                int th_int = (int)threshold;
                int th_dec = (int)((threshold - th_int) * 10);
                if (th_dec < 0) th_dec = -th_dec;
                rt_snprintf(buf, sizeof(buf), "%d.%dC", th_int, th_dec);
                lv_label_set_text(label_threshold_val, buf);
            }
        }

        /* BEEP 状态图标 */
        if (beep)
            lv_obj_clear_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);

        /* 告警状态 & 主图 */
        if (local_data.temperature > threshold) {
            lv_img_set_src(img_main_status, &Alarm);
            lv_obj_set_style_text_color(label_temp_val, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(label_alarm_status, "温度过高");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_RED), 0);
        } else if (local_data.tilt_alarm) {
            lv_img_set_src(img_main_status, &tilt);
            lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
            lv_label_set_text(label_alarm_status, "货架倾斜");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_ORANGE), 0);
        } else {
            lv_img_set_src(img_main_status, &Environmental);
            lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
            lv_label_set_text(label_alarm_status, "正常");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_GREEN), 0);
        }

        rt_mutex_release(data_lock);
    }
}

/* ==================== UI初始化 ==================== */
static void warehouse_ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);

    img_wifi_status = lv_img_create(scr);
    lv_img_set_src(img_wifi_status, &connected);
    lv_obj_align(img_wifi_status, LV_ALIGN_TOP_RIGHT, -5, 5);

    img_cloud_status = lv_img_create(scr);
    lv_img_set_src(img_cloud_status, &onenet_upload);
    lv_obj_align_to(img_cloud_status, img_wifi_status, LV_ALIGN_OUT_LEFT_MID, -5, 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &my_font_cn_16, 0);
    lv_obj_set_style_text_color(title, lv_color_black(), 0);
    lv_label_set_text(title, "仓库环境监测");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *img_temp = lv_img_create(scr);
    lv_img_set_src(img_temp, &Environmental);
    lv_obj_align(img_temp, LV_ALIGN_TOP_LEFT, 15, 45);

    label_temp_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_temp_val, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
    lv_label_set_text(label_temp_val, "--.-- C");
    lv_obj_align_to(label_temp_val, img_temp, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lv_obj_t *img_humi = lv_img_create(scr);
    lv_img_set_src(img_humi, &waterprof);
    lv_obj_align(img_humi, LV_ALIGN_RIGHT_MID, -15, -60);

    label_humi_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_humi_val, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label_humi_val, lv_color_black(), 0);
    lv_label_set_text(label_humi_val, "--.-- %");
    lv_obj_align_to(label_humi_val, img_humi, LV_ALIGN_OUT_LEFT_MID, -10, 0);

    lv_obj_t *img_light = lv_img_create(scr);
    lv_img_set_src(img_light, &light);
    lv_obj_align(img_light, LV_ALIGN_LEFT_MID, 15, 0);

    label_light_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_light_val, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_light_val, lv_color_black(), 0);
    lv_label_set_text(label_light_val, "--- Lux");
    lv_obj_align_to(label_light_val, img_light, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

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

    label_threshold_val = lv_label_create(scr);
    lv_obj_set_style_text_font(label_threshold_val, &my_font_cn_16, 0);
    lv_obj_set_style_text_color(label_threshold_val, lv_color_hex(0x0000FF), 0);
    update_threshold_display();
    lv_obj_align_to(label_threshold_val, label_light_val, LV_ALIGN_OUT_BOTTOM_MID, 0, 25);

    /*
     * 注册 lv_timer 周期性刷新回调 — 200ms 周期，运行在 LVGL 线程上下文。
     * LVGL v8 API: lv_timer_create(callback, period_ms, user_data)
     * 这是唯一操作 LVGL 对象的地方，确保线程安全。
     */
    lv_timer_create(lv_ui_refresh_task, 200, NULL);
}

/* ==================== main 函数 ==================== */
int main(void)
{
    print_memory("boot_start");

    /* ——— 硬件底层初始化 ——— */
    rt_pin_mode(LED_R_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_R_PIN, PIN_HIGH);

    /*
     * LVGL 初始化 + UI 创建 — 在 scheduler 启动前完成，单线程安全。
     * lv_port_disp_init() 内部创建 "LVGL" 线程(prio 20, 8KB) 负责 lv_timer_handler()。
     * warehouse_ui_init() 注册 lv_timer 回调，后续由 LVGL 线程安全调用。
     * ⚠ 不再创建独立的 display 线程 — 避免双线程 LVGL 竞态导致 Hard Fault。
     */
    rt_kprintf("[main] Initializing LVGL...\n");
    lv_init();
    lv_port_disp_init();
    warehouse_ui_init();
    rt_kprintf("[main] LVGL OK\n");

    /*
     * ——— RT-Thread 对象创建 & 模块启动 ———
     */
    app_logic_init();
    app_sensor_init();
    app_net_init();

    print_memory("modules_started");

    /* ——— 连接 WiFi ——— */
    rt_kprintf("[main] Delaying 3s for system stabilization...\n");
    rt_thread_mdelay(3000);

    int ret = wifi_connect();
    if (ret != 0)
    {
        rt_kprintf("[main] WiFi connection failed!\n");
    }
    else
    {
        rt_kprintf("[main] WiFi connected, starting OneNET...\n");
        rt_thread_mdelay(2000);
        app_net_onenet_start();
    }

    print_memory("boot_done");

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
