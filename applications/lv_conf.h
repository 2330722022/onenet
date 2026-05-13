/**
 * @file lv_conf.h
 * Configuration file for LVGL
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   Memory Manager Settings
 *====================*/

/* Memory configuration options (see lv_malloc.c for details) */
#define LV_MEM_CUSTOM 1    /* 1: �� (rt_malloc/rt_free) */
#define LV_MEM_SIZE (32U * 1024U)  /* ����?*/
#define LV_MEM_AUTO_DEFRAG 1  /* �� */

/* ��?*/
#if LV_MEM_CUSTOM == 1
#include <rtthread.h>
#define lv_malloc(size)       rt_malloc(size)
#define lv_free(p)            rt_free(p)
#define lv_realloc(p, size)   rt_realloc(p, size)
#define lv_calloc(num, size)  rt_calloc(num, size)
#endif

/*========================
   Color Settings
 *========================*/

/* Color depth:
 * 1:  1 byte per pixel (monochrome)
 * 8:  RGB332
 * 16: RGB565
 * 32: ARGB8888
 */
#define LV_COLOR_DEPTH     16

/* Swap the 2 bytes of RGB565 color
 * Useful if the display has SPI interface (some controllers send high byte first)
 */
#define LV_COLOR_16_SWAP   0

/*========================
   Feature Usage
 *========================*/

#define LV_USE_ANIMATION        0  /* ���?*/
#define LV_USE_SHADOW           0  /* ��?*/
#define LV_USE_GROUP            1

/*========================
   Drawing Settings
 *========================*/

#define LV_USE_DRAW_SW 1           /* �� */
#define LV_DRAW_SW_STRIDE_ALIGN 1  /* ��?*/
#define LV_DRAW_SW_FILL_STYLE_SIMPLE 1  /* ���?*/

/*=================
   Debug Settings
 *================*/

#define LV_USE_LOG      0
#if LV_USE_LOG
    /* How important log should be added:
     * LV_LOG_LEVEL_TRACE       A lot of logs to give detailed information
     * LV_LOG_LEVEL_INFO        Log important events
     * LV_LOG_LEVEL_WARN        Log if something unwanted happened but didn't cause a problem
     * LV_LOG_LEVEL_ERROR       Only critical issue, when the system may fail
     * LV_LOG_LEVEL_USER        Only logs added by the user
     * LV_LOG_LEVEL_NONE        Do not log anything
     */
    #define LV_LOG_LEVEL    LV_LOG_LEVEL_WARN

    /* 1: Print the log with printf*/
    #define LV_LOG_PRINTF   0
#endif

/*================
   FONT USAGE
 *================*/

/* Montserrat fonts with bpp = 4
 * https://fonts.google.com/specimen/Montserrat  */
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_18  1

/*================
   WIDGET USAGE
 *================*/

#define LV_USE_ANIMIMG    1
#define LV_USE_ARC        1
#define LV_USE_BAR        1
#define LV_USE_BTN        1
#define LV_USE_BTNMATRIX  1
#define LV_USE_CALENDAR   1
#define LV_USE_CANVAS     1
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1
#define LV_USE_IMG        1
#define LV_USE_LABEL      1
#define LV_USE_LINE       1
#define LV_USE_LIST       1
#define LV_USE_OBJ_TABVIEW   1
#define LV_USE_ROLLER     1
#define LV_USE_SLIDER     1
#define LV_USE_SWITCH     1
#define LV_USE_TABLE      1
#define LV_USE_TEXTAREA   1
#define LV_USE_MSGBOX     1
#define LV_USE_COLORWHEEL 1

/*================
   COMPONENT USAGE
 *================*/

#define LV_USE_EXTRA 1
#if LV_USE_EXTRA
    #define LV_USE_FLEX 1
    #define LV_USE_GRID 1
#endif

#endif /* LV_CONF_H */
