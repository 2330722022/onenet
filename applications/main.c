#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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
static void print_memory_info(const char *tag)
{
    rt_size_t total, used, max_used;
    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("[MEM] %s - Total: %u KB, Used: %u KB (%u%%), Max Used: %u KB (%u%%)\n",
               tag,
               total / 1024, used / 1024, (used * 100) / total,
               max_used / 1024, (max_used * 100) / total);
}

/* ==================== LVGL UI ==================== */
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

/* 重写弱符号：阈值LCD显示更新 */
void update_threshold_display(void)
{
    if (label_threshold_val != RT_NULL) {
        char buf[16];
        int th_int = (int)temp_threshold;
        int th_dec = (int)((temp_threshold - th_int) * 10);
        if (th_dec < 0) th_dec = -th_dec;
        rt_snprintf(buf, sizeof(buf), "%d.%dC", th_int, th_dec);
        lv_label_set_text(label_threshold_val, buf);
    }
}

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
}

/* ==================== 显示线程 ==================== */
static void display_thread_entry(void *parameter)
{
    struct sensor_data local_data;
    struct sensor_data *data_ptr = &local_data;
    char buf[32];
    rt_err_t mutex_ret;

    warehouse_ui_init();

    while (1)
    {
        mutex_ret = rt_mutex_take(data_mutex, 10);
        if (mutex_ret == RT_EOK) {
            local_data = shared_data;
            rt_mutex_release(data_mutex);
            data_ptr = &local_data;
        }

        rt_sprintf(buf, "%d.%d C", (int)data_ptr->temperature,
                   abs((int)((data_ptr->temperature - (int)data_ptr->temperature) * 10)));
        lv_label_set_text(label_temp_val, buf);

        rt_sprintf(buf, "%d.%d %%", (int)data_ptr->humidity,
                   abs((int)((data_ptr->humidity - (int)data_ptr->humidity) * 10)));
        lv_label_set_text(label_humi_val, buf);

        rt_sprintf(buf, "%d Lux", (int)data_ptr->light);
        lv_label_set_text(label_light_val, buf);

        rt_sprintf(buf, "X:%d.%d", (int)data_ptr->tilt_angle_x,
                   abs((int)((data_ptr->tilt_angle_x - (int)data_ptr->tilt_angle_x) * 10)));
        lv_label_set_text(label_tilt_x_val, buf);
        rt_sprintf(buf, "Y:%d.%d", (int)data_ptr->tilt_angle_y,
                   abs((int)((data_ptr->tilt_angle_y - (int)data_ptr->tilt_angle_y) * 10)));
        lv_label_set_text(label_tilt_y_val, buf);

        if (wifi_connected)
            lv_img_set_src(img_wifi_status, &connected);

        if (beep_status)
            lv_obj_clear_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(img_beep_status, LV_OBJ_FLAG_HIDDEN);

        if (data_ptr->temperature > temp_threshold)
        {
            lv_img_set_src(img_main_status, &Alarm);
            lv_obj_set_style_text_color(label_temp_val, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(label_alarm_status, "温度过高");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_RED), 0);
        }
        else if (data_ptr->tilt_alarm)
        {
            lv_img_set_src(img_main_status, &tilt);
            lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
            lv_label_set_text(label_alarm_status, "货架倾斜");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_ORANGE), 0);
        }
        else
        {
            lv_img_set_src(img_main_status, &Environmental);
            lv_obj_set_style_text_color(label_temp_val, lv_color_black(), 0);
            lv_label_set_text(label_alarm_status, "正常");
            lv_obj_set_style_text_color(label_alarm_status, lv_palette_main(LV_PALETTE_GREEN), 0);
        }

        lv_task_handler();
        rt_thread_mdelay(200);
    }
}

/* ==================== main 函数 ==================== */
int main(void)
{
    print_memory_info("start");

    /* 硬件底层初始化 */
    rt_pin_mode(LED_R_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_R_PIN, PIN_HIGH);

    /* 初始化LVGL */
    rt_kprintf("[main] Initializing LVGL...\n");
    lv_init();
    lv_port_disp_init();
    rt_kprintf("[main] LVGL OK\n");

    /* 初始化业务逻辑模块（BEEP、舵机、按键）*/
    app_logic_init();

    /* 初始化传感器模块（创建采集线程）*/
    app_sensor_init();

    /* 初始化网络服务模块（创建Web Server线程）*/
    app_net_init();

    /* 创建显示线程 */
    rt_thread_t tid_display = rt_thread_create("display", display_thread_entry, RT_NULL,
                                                4096, 21, 10);
    if (tid_display) rt_thread_startup(tid_display);

    print_memory_info("modules_started");

    /* 等待系统稳定，再连接WiFi */
    rt_kprintf("[main] Delaying 3s for system stabilization...\n");
    rt_thread_mdelay(3000);

    /* 连接WiFi */
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

    print_memory_info("done");

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
