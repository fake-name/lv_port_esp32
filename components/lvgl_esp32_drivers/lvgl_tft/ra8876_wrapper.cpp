/**
 * @file RA8876.c
 */

/*********************
 *      INCLUDES
 *********************/
#include "RA8876.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "RA8876"

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void RA8876_set_orientation(uint8_t orientation);

static void RA8876_send_cmd(uint8_t cmd);
static void RA8876_send_data(void * data, uint16_t length);
static void RA8876_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
// From github.com/jeremyjh/ESP32_TFT_library
// From github.com/mvturnho/RA8876-lvgl-ESP32-WROVER-B
void RA8876_init(void)
{
	lcd_init_cmd_t ili_init_cmds[]={
                {RA8876_CMD_SLEEP_OUT, {0x00}, 0x80},
		{RA8876_CMD_POSITIVE_GAMMA_CORRECTION, {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15},
		{RA8876_CMD_NEGATIVE_GAMMA_CORRECTION, {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15},
		{RA8876_CMD_POWER_CONTROL_1, {0x17, 0x15}, 2},
		{RA8876_CMD_POWER_CONTROL_2, {0x41}, 1},
		{RA8876_CMD_VCOM_CONTROL_1, {0x00, 0x12, 0x80}, 3},
		{RA8876_CMD_MEMORY_ACCESS_CONTROL, {(0x20 | 0x08)}, 1},
		{RA8876_CMD_COLMOD_PIXEL_FORMAT_SET, {0x66}, 1},
		{RA8876_CMD_INTERFACE_MODE_CONTROL, {0x00}, 1},
		{RA8876_CMD_FRAME_RATE_CONTROL_NORMAL, {0xA0}, 1},
		{RA8876_CMD_DISPLAY_INVERSION_CONTROL, {0x02}, 1},
		{RA8876_CMD_DISPLAY_FUNCTION_CONTROL, {0x02, 0x02}, 2},
		{RA8876_CMD_SET_IMAGE_FUNCTION, {0x00}, 1},
		{RA8876_CMD_WRITE_CTRL_DISPLAY, {0x28}, 1},
		{RA8876_CMD_WRITE_DISPLAY_BRIGHTNESS, {0x7F}, 1},
		{RA8876_CMD_ADJUST_CONTROL_3, {0xA9, 0x51, 0x2C, 0x02}, 4},
		{RA8876_CMD_DISPLAY_ON, {0x00}, 0x80},
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
	gpio_set_direction(RA8876_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(RA8876_RST, GPIO_MODE_OUTPUT);

	gpio_set_direction(RA8876_BCKL, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(RA8876_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(RA8876_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

	ESP_LOGI(TAG, "RA8876 initialization.");

	// Exit sleep
	RA8876_send_cmd(0x01);	/* Software reset */
	vTaskDelay(100 / portTICK_RATE_MS);

	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		RA8876_send_cmd(ili_init_cmds[cmd].cmd);
		RA8876_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	RA8876_enable_backlight(true);

        RA8876_set_orientation(CONFIG_LVGL_DISPLAY_ORIENTATION);
}

// Flush function based on mvturnho repo
void RA8876_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    lv_color16_t *buffer_16bit = (lv_color16_t *) color_map;
    uint8_t *mybuf;
    do {
        mybuf = (uint8_t *) heap_caps_malloc(3 * size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if (mybuf == NULL)  ESP_LOGW(TAG, "Could not allocate enough DMA memory!");
    } while (mybuf == NULL);

    uint32_t LD = 0;
    uint32_t j = 0;

    for (uint32_t i = 0; i < size; i++) {
        LD = buffer_16bit[i].full;
        mybuf[j] = (uint8_t) ((LD & 0xF800) >> 8);
        j++;
        mybuf[j] = (uint8_t) ((LD & 0x07E0) >> 3);
        j++;
        mybuf[j] = (uint8_t) ((LD & 0x001F) << 3);
        j++;
    }

	/* Column addresses  */
	uint8_t xb[] = {
	    (uint8_t) (area->x1 >> 8) & 0xFF,
	    (uint8_t) (area->x1) & 0xFF,
	    (uint8_t) (area->x2 >> 8) & 0xFF,
	    (uint8_t) (area->x2) & 0xFF,
	};

	/* Page addresses  */
	uint8_t yb[] = {
	    (uint8_t) (area->y1 >> 8) & 0xFF,
	    (uint8_t) (area->y1) & 0xFF,
	    (uint8_t) (area->y2 >> 8) & 0xFF,
	    (uint8_t) (area->y2) & 0xFF,
	};

	/*Column addresses*/
	RA8876_send_cmd(RA8876_CMD_COLUMN_ADDRESS_SET);
	RA8876_send_data(xb, 4);

	/*Page addresses*/
	RA8876_send_cmd(RA8876_CMD_PAGE_ADDRESS_SET);
	RA8876_send_data(yb, 4);

	/*Memory write*/
	RA8876_send_cmd(RA8876_CMD_MEMORY_WRITE);

	RA8876_send_color((void *) mybuf, size * 3);
	heap_caps_free(mybuf);
}

void RA8876_enable_backlight(bool backlight)
{
    ESP_LOGI(TAG, "%s backlight.", backlight ? "Enabling" : "Disabling");
    uint32_t tmp = 0;

    tmp = backlight ? 1 : 0;

    gpio_set_level(RA8876_BCKL, tmp);
}
