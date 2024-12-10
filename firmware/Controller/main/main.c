#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "remote_ctrl_task.h"
#include "lcd_task.h"
#include "touch_utils.h"

static const char *TAG = "main";

#define BOOT_BTN_PIN_NUM	0
#define BOOT_BTN_PIN_SEL	(1ULL << BOOT_BTN_PIN_NUM)

void app_main(void)
{
    gpio_config_t io_conf = {};	
	uint8_t boot_btn_hold_count = 0;
	uint16_t z_threshold = 0; // for touch calibration task to reset the pressing thresshold of the touch panel.
	
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BOOT_BTN_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);	
	
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
			
	// Initialize touch and LCD 
	ESP_ERROR_CHECK(touch_utils_init());
	ESP_ERROR_CHECK(lcd_init());
	
	// Initialize ESPNOW for remote control
	ESP_ERROR_CHECK(remote_ctrl_init());	
	
	while(1){
		if(remote_ctrl_state == REMOTE_CTRL_STATE__LISTENING){	
			if(!gpio_get_level(BOOT_BTN_PIN_NUM)){
				if(boot_btn_hold_count < 3){
					if(++boot_btn_hold_count == 3){
						ESP_LOGI(TAG, "Suspend remote_ctrl_task and initiate touch calibration.");
						vTaskSuspend(remote_ctrl_task_handle);
						touch_is_pressed(&z_threshold); // Get the current z_threshold as the thresshold of touch_is_pressed.
						touch_calibration_task_init(&z_threshold);
					}
				}
			}else{
				boot_btn_hold_count = 0;
			}					
		}
		
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
