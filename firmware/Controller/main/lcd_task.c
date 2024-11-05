#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"
#include "remote_ctrl_task.h"
#include "esp_camera.h"
#include "lcd_task.h"
#include "touch_utils.h"
#include "user_interface_utils.h"
#include "esp_log.h"
#include "main.h"

static const char *TAG = "lcd";

// PCLK frequency can't go too high as the limitation of PSRAM bandwidth
#define LCD_PIXEL_CLOCK_HZ     (2 * 1000 * 1000)
// Bit number used to represent command and parameter for i80 controller to ILLI9341
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8
// Supported alignment: 16, 32, 64. A higher alignment can enables higher burst transfer size, thus a higher i80 bus throughput.
#define PSRAM_DATA_ALIGNMENT   	64

#define LCD_BK_LIGHT_ON_LEVEL  		1
#define LCD_BK_LIGHT_OFF_LEVEL 		!LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIN_NUM_DATA0          	4
#define LCD_PIN_NUM_DATA1          	5
#define LCD_PIN_NUM_DATA2          	6
#define LCD_PIN_NUM_DATA3          	7
#define LCD_PIN_NUM_DATA4          	15
#define LCD_PIN_NUM_DATA5          	16
#define LCD_PIN_NUM_DATA6          	17
#define LCD_PIN_NUM_DATA7          	18
#define LCD_PIN_NUM_DATA8          	8
#define LCD_PIN_NUM_DATA9          	42
#define LCD_PIN_NUM_DATA10         	41
#define LCD_PIN_NUM_DATA11         	40
#define LCD_PIN_NUM_DATA12         	39
#define LCD_PIN_NUM_DATA13         	38
#define LCD_PIN_NUM_DATA14         	3
#define LCD_PIN_NUM_DATA15         	45
#define LCD_PIN_NUM_PCLK           	2
#define LCD_PIN_NUM_CS             	48
#define LCD_PIN_NUM_DC             	1
#define LCD_PIN_NUM_RST            	47
#define LCD_PIN_NUM_BK_LIGHT       	21

#define LCD_EVENT_QUEUE_SIZE		6
QueueHandle_t lcd_event_queue;

esp_lcd_panel_handle_t panel_handle = NULL;
uint8_t* lcd_frame_bitmap = NULL;
remote_ctrl__turret_data_t* recv_td = NULL;

static bool lcd_notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);

#define TASK_SCHEDULED_PERIOD	10
static void lcd_task(void *pvParameter);

esp_err_t lcd_init(void){
		
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .wr_gpio_num = LCD_PIN_NUM_PCLK,
        .data_gpio_nums = {
            LCD_PIN_NUM_DATA0,
            LCD_PIN_NUM_DATA1,
            LCD_PIN_NUM_DATA2,
            LCD_PIN_NUM_DATA3,
            LCD_PIN_NUM_DATA4,
            LCD_PIN_NUM_DATA5,
            LCD_PIN_NUM_DATA6,
            LCD_PIN_NUM_DATA7,
            LCD_PIN_NUM_DATA8,
            LCD_PIN_NUM_DATA9,
            LCD_PIN_NUM_DATA10,
            LCD_PIN_NUM_DATA11,
            LCD_PIN_NUM_DATA12,
            LCD_PIN_NUM_DATA13,
            LCD_PIN_NUM_DATA14,
            LCD_PIN_NUM_DATA15,
        },
        .bus_width = 16,
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
        .psram_trans_align = PSRAM_DATA_ALIGNMENT,
        .sram_trans_align = 4,
    };		
	ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
	
	esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = 0,
        },
        .on_color_trans_done = lcd_notify_flush_ready,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));	
	
	
    ESP_LOGI(TAG, "Install LCD driver of ILI9341");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));	
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	
	// ILI9341 initial code provided by module vendor
	esp_lcd_panel_io_tx_param(io_handle, 0xCF, (uint8_t[]) { 0x00, 0xC1, 0x30 }, 3);
	esp_lcd_panel_io_tx_param(io_handle, 0xED, (uint8_t[]) { 0x64, 0x03, 0x12, 0x81 }, 4);
	esp_lcd_panel_io_tx_param(io_handle, 0xE8, (uint8_t[]) { 0x85, 0x10, 0x7A }, 3);
	esp_lcd_panel_io_tx_param(io_handle, 0xCB, (uint8_t[]) { 0x39, 0x2C, 0x00, 0x34, 0x02 }, 5);
	esp_lcd_panel_io_tx_param(io_handle, 0xF7, (uint8_t[]) { 0x20 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0xEA, (uint8_t[]) { 0x00, 0x00 }, 2);
	esp_lcd_panel_io_tx_param(io_handle, 0xC0, (uint8_t[]) { 0x1B }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0xC1, (uint8_t[]) { 0x01 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0xC5, (uint8_t[]) { 0x30, 0x30 }, 2);
	esp_lcd_panel_io_tx_param(io_handle, 0xC7, (uint8_t[]) { 0xB7 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]) { 0x48 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0x3A, (uint8_t[]) { 0x55 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0xB1, (uint8_t[]) { 0x00, 0x1A }, 2);
	esp_lcd_panel_io_tx_param(io_handle, 0xB6, (uint8_t[]) { 0x0A, 0xA2 }, 2);
	esp_lcd_panel_io_tx_param(io_handle, 0xF2, (uint8_t[]) { 0x00 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0x26, (uint8_t[]) { 0x01 }, 1);
	esp_lcd_panel_io_tx_param(io_handle, 0xE0, (uint8_t[]) {          
        0x0F, 0x2A, 0x28, 0x08, 0x0E, 0x08, 0x54, 0xA9, 0x43, 0x0A, 0x0F, 0x00, 0x00, 0x00, 0x00 }, 15);
    esp_lcd_panel_io_tx_param(io_handle, 0xE1, (uint8_t[]) {       
        0x00, 0x15, 0x17, 0x07, 0x11, 0x06, 0x2B, 0x56, 0x3C, 0x05, 0x10, 0x0F, 0x3F, 0x3F, 0x0F }, 15);
	esp_lcd_panel_io_tx_param(io_handle, 0x2B, (uint8_t[]) { 0x00, 0x00, 0x01, 0x3F }, 4);
	esp_lcd_panel_io_tx_param(io_handle, 0x2A, (uint8_t[]) { 0x00, 0x00, 0x00, 0xEF }, 4);
	esp_lcd_panel_io_tx_param(io_handle, 0x11, NULL, 0);
	vTaskDelay(pdMS_TO_TICKS(120));
	esp_lcd_panel_io_tx_param(io_handle, 0x29, NULL, 0);	
	
	ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
	ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
	ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false)); 
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);	
	
	lcd_frame_bitmap = heap_caps_aligned_alloc(PSRAM_DATA_ALIGNMENT, LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if(lcd_frame_bitmap == NULL){
        ESP_LOGE(TAG, "Allocate frame buffer fail");
        return ESP_FAIL;		
	}
	
    lcd_event_queue = xQueueCreate(LCD_EVENT_QUEUE_SIZE, sizeof(lcd_event_t));
    if (lcd_event_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }	

	xTaskCreatePinnedToCore(lcd_task, "lcd_task", 4096, NULL, TASK_PRIORITY__LCD, NULL, TASK_CPU_CORE__LCD);
	
	return ESP_OK;
}

static void lcd_task(void *pvParameter){		
	lcd_event_t lcd_evt;
	remote_ctrl_event_t rc_evt;
	
	bool lcd_flush_is_done = true;	
	int lcd_frame_count = 0;
	
	while(1){
		if((xQueueReceive(lcd_event_queue, &lcd_evt, 0)) == pdTRUE) {		
			switch (lcd_evt.id){
				case FLUSH_WITH_TURRET_DATA: // Receive a jpg frame from remote control task
					recv_td	= (remote_ctrl__turret_data_t*)lcd_evt.info;
					if(recv_td){
						if(lcd_flush_is_done == true){					
							if(jpg2rgb565(recv_td->frame_buf, recv_td->frame_len, lcd_frame_bitmap, JPG_SCALE_NONE) == true){		
								lcd_flush_is_done = false;	
								ui__draw_turret_status(lcd_frame_bitmap, recv_td);								
								esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_frame_bitmap);	
							}else{
								rc_evt.id = LCD_FRAME_REQ;
								if(xQueueSend(remote_ctrl_event_queue, &rc_evt, 0) != pdTRUE){
									ESP_LOGW(TAG, "Pushing LCD_FRAME_REQ to remote_ctrl_event_queue fail");
								}								
							}								
						}else{
							ESP_LOGW(TAG, "LCD flushing is on-going.");
						}					
					}
				break;
				case FLUSH_WITH_BITMAP:
					esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_frame_bitmap);
				break;
				case FLUSH_DONE: // 
					if(recv_td){															
						rc_evt.id = LCD_FRAME_REQ;
						if(xQueueSend(remote_ctrl_event_queue, &rc_evt, 0) != pdTRUE){
							ESP_LOGW(TAG, "Pushing LCD_FRAME_REQ to remote_ctrl_event_queue fail");
						}

						recv_td = NULL;
						lcd_flush_is_done = true;
						lcd_frame_count++;							
					}
				break;
				case GET_FRAME_RATE:
					ESP_LOGI(TAG, "lcd_frame count = %d", lcd_frame_count);
					lcd_frame_count = 0;
				break;
				default:
				break;
			}		
		}		
		vTaskDelay(pdMS_TO_TICKS(TASK_SCHEDULED_PERIOD));	
	}
}

static bool lcd_notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
	lcd_event_t evt;

	if(recv_td){
		evt.id = FLUSH_DONE;
		evt.info = NULL;
		if (xQueueSend(lcd_event_queue, &evt, 0) != pdTRUE) {
			ESP_LOGW(TAG, "Pushing FLUSH_DONE to lcd_event_queue fail");
		}			
	}
    return false;
}


 


