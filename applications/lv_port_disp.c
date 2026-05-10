/*
 * LVGL Display Port for ST7789 LCD
 */

#include <lvgl.h>
#include <drv_lcd.h>

/* LCD buffer for LVGL (1/10 screen size) */
static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf1[LCD_W * 24];  /* 240 * 24 = 5760 pixels */

/* Flush callback: copy buffer to LCD */
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    /* Set the drawing window */
    lcd_address_set(area->x1, area->y1, area->x2, area->y2);
    
    /* Copy color data to LCD */
    lcd_fill_array(area->x1, area->y1, area->x2, area->y2, color_p);
    
    /* Tell LVGL we are ready with the flushing */
    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    /* Initialize LCD hardware */
    rt_device_t lcd_dev = rt_device_find("lcd");
    if (lcd_dev != RT_NULL)
    {
        rt_device_open(lcd_dev, RT_DEVICE_OFLAG_RDWR);
        rt_kprintf("LCD device opened successfully\n");
    }
    
    /* Clear LCD screen to white (match LVGL background) */
    lcd_clear(WHITE);
    rt_kprintf("LCD cleared to white\n");
    
    /* Initialize display buffer */
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LCD_W * 24);
    rt_kprintf("LVGL display buffer initialized\n");
    
    /* Initialize display driver */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    
    /* Set basic parameters */
    disp_drv.hor_res = LCD_W;
    disp_drv.ver_res = LCD_H;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.flush_cb = disp_flush;
    
    /* Register the display driver */
    lv_disp_drv_register(&disp_drv);
    rt_kprintf("LVGL display driver registered\n");
}

/* Empty input driver (no touch) */
void lv_port_indev_init(void)
{
    /* No touch screen */
}

/* Empty GUI init */
void lv_user_gui_init(void)
{
    /* GUI initialized elsewhere */
}
