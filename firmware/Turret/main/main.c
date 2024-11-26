#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "main.h"
#include "remote_ctrl_task.h"
#include "camera_utils.h"
#include "orientation_ctrl_task.h"
#include "fire_ctrl_task.h"
#include "speaker_ctrl_task.h"
#include "range_finder_utils.h"

static const char *TAG = "main";

static esp_err_t i2c_init(i2c_config_t* conf, int pin_sda, int pin_scl){   
    esp_err_t ret;

    memset(conf, 0, sizeof(i2c_config_t));  
	conf->mode = I2C_MODE_MASTER;
    conf->sda_io_num = pin_sda;
    conf->sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf->scl_io_num = pin_scl;
    conf->scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf->master.clk_speed = 100000;

    if ((ret = i2c_param_config(I2C_PORT_DEFAULT, conf)) != ESP_OK) {
        return ret;
    }

    return i2c_driver_install(I2C_PORT_DEFAULT, conf->mode, 0, 0, 0);	
}

uint32_t global_timer_us_count = 0;
static void global_timer_isr(void *pvParameter){
	global_timer_us_count += GLOBAL_TIMER_INTR_PEROID;
	if(global_timer_us_count >= GLOBAL_TIMER_INTR_PEROID * 400){global_timer_us_count = 0;}
	
	if(global_timer_us_count % STEPPER_MOTOR_STEP_HALF_PERIOD_US == 0){stepper_motor_steps_ctrl_isr();}
	if(global_timer_us_count % FIRE_CTRL_ISR_PERIOD_US == 0){fire_ctrl__isr();}	
}

static esp_err_t global_timer_intr_init(esp_timer_handle_t* handle){
    const esp_timer_create_args_t motor_timer_args = {
        .callback = &global_timer_isr,
        .name = "global_timer_isr"
    };		
	esp_timer_create(&motor_timer_args, handle);
	global_timer_us_count = 0;
	esp_timer_start_periodic(*handle, GLOBAL_TIMER_INTR_PEROID);
	return ESP_OK;
}

void app_main(void)
{	
	esp_timer_handle_t global_timer_intr = NULL;
	i2c_config_t global_i2c_config;
	
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
	// Initialize i2c interface used by:
	// 1. PCF8574 i2c-to-GPIO expander, orientation_ctrl_task.
	// 2. OV5640, camera_utils.
	ESP_ERROR_CHECK(i2c_init(&global_i2c_config, I2C_PIN_SDA, I2C_PIN_SCL));
	
	// Initialize fire control
	ESP_ERROR_CHECK(fire_ctrl__init());	
	
	// Initialize speaker 
	ESP_ERROR_CHECK(speaker_ctrl_task_init());	
	
	// Initialize global timer interrpt used by:
	// 1. Step motor tick, orientation_ctrl_task.
	// 2. fire_ctrl_task
	ESP_ERROR_CHECK(global_timer_intr_init(&global_timer_intr));
				
	// Initialize turret orientation
	ESP_ERROR_CHECK(orientation_ctrl_init());
	
	// Initialize cmaera
	ESP_ERROR_CHECK(camera_init());
	
	// Initialize remote control 
	ESP_ERROR_CHECK(remote_ctrl_init());
	
	while(1){
		vTaskDelay(1000 / portTICK_PERIOD_MS);		
	}
}
