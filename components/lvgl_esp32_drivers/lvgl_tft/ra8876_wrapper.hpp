/**
 * @file ra8876.h
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

	/*********************
	 *      INCLUDES
	 *********************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "lvgl/lvgl.h"
	#include "../lvgl_helpers.h"

	void ra8876_init(void);
	void ra8876_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
	void ra8876_enable_backlight(bool backlight);

#ifdef __cplusplus
} /* extern "C" */
#endif

/*********************
 *      DEFINES
 *********************/
#define ILI9488_RST  CONFIG_LVGL_DISP_PIN_RST
#define ILI9488_BCKL CONFIG_LVGL_DISP_PIN_BCKL

#define ILI9488_ENABLE_BACKLIGHT_CONTROL CONFIG_LVGL_ENABLE_BACKLIGHT_CONTROL


