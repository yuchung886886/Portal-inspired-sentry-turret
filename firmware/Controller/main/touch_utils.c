#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "touch_utils.h"
#include "lcd_task.h"
#include "user_interface_utils.h"
#include "remote_ctrl_task.h"
#include "esp_log.h"
#include "main.h"

static const char *TAG = "touch";

#define TOUCH_SPI_CLK_FREQ         		(1 * 1000 * 1000)
#define TOUCH_SPI_MISO_READY_DELAY_NS   200		// MISO signal is not ready until 200ns after the low-edge of CLK signal.
#define TOUCH_SPI_HOST    			SPI2_HOST
#define TOUCH_PIN_NUM_CS   			10			
#define TOUCH_PIN_NUM_MOSI 			11
#define TOUCH_PIN_NUM_CLK  			12
#define TOUCH_PIN_NUM_MISO 			13

#define XPT2046_CMD__GET_Z1 0xB1
#define XPT2046_CMD__GET_Z2 0xC1
#define XPT2046_CMD__GET_X 0x91
#define XPT2046_CMD__GET_Y 0xD1
#define XPT2046_Z_THRESHOLD	3800

#define STORAGE_NAMESPACE "storage"

// Configurations of the touch screen controller
typedef struct {
    spi_host_device_t host; // The SPI host used.
    gpio_num_t cs_io;       // CS pin number.
    gpio_num_t miso_io;     // MISO pin number.
} touch_config_t;

typedef struct {
	touch_config_t cfg;
	spi_device_handle_t spi;
} touch_context_t;
touch_context_t* touch_handle;

typedef struct {
	float a;
	float b;
	float c;
	float d;
	float e;
	float f;
} touch_calibration_coef_t;
// default touch calibration coefficient, it is not calibrated.
touch_calibration_coef_t touch_cal_coef = {0}; 

static void touch_calibration_task(void *pvParameter);

static void spi_cs_pull_high(spi_transaction_t* t){
    gpio_set_level(((touch_context_t*)t->user)->cfg.cs_io, 1);
}

static void spi_cs_pull_low(spi_transaction_t* t){
    gpio_set_level(((touch_context_t*)t->user)->cfg.cs_io, 0);
}

static esp_err_t touch_spi_init(const touch_config_t *cfg, touch_context_t** out_ctx){	
	esp_err_t err =ESP_OK;
	
	touch_context_t* ctx = (touch_context_t*)malloc(sizeof(touch_context_t));
	if(!ctx){
		ESP_LOGI(TAG, "Allocate touch_context_t fail.");
		return ESP_ERR_NO_MEM;
	}else{		
		*ctx = (touch_context_t){
			.cfg =*cfg,
		};
	}
	
    spi_device_interface_config_t devcfg={
        .command_bits = 8,
		.address_bits = 0,
        .clock_speed_hz = TOUCH_SPI_CLK_FREQ,
        .mode = 0,          // SPI mode 0
        .spics_io_num = -1, // control CS pin by SW
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = spi_cs_pull_low,
        .post_cb = spi_cs_pull_high,
        .input_delay_ns = TOUCH_SPI_MISO_READY_DELAY_NS,
    };	
	
	err = spi_bus_add_device(ctx->cfg.host, &devcfg, &ctx->spi);
	if(err != ESP_OK){goto cleanup;}
	
    gpio_set_level(ctx->cfg.cs_io, 1);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(ctx->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);	
	
    *out_ctx = ctx;
    return ESP_OK;
	
cleanup:
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    free(ctx);
    return err;		
}

static esp_err_t touch_spi_read(touch_context_t* ctx, uint8_t rx_cmd, uint8_t rx_bits_len, uint8_t* rx_buf){
    spi_transaction_t t = {
        .cmd = rx_cmd,
        .rxlength = rx_bits_len,
        .flags = SPI_TRANS_USE_RXDATA,
        .user = ctx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->spi, &t);
    if (err!= ESP_OK) return err;

    memcpy(rx_buf, t.rx_data, rx_bits_len / 8);
    return ESP_OK;
}

esp_err_t touch_utils_init(void){
	
	nvs_handle_t nvs_handle;
	size_t touch_cal_coef_size;
	
    spi_bus_config_t buscfg={
        .miso_io_num = TOUCH_PIN_NUM_MISO,
        .mosi_io_num = TOUCH_PIN_NUM_MOSI,
        .sclk_io_num = TOUCH_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };	
    touch_config_t touch_config = {
        .cs_io = TOUCH_PIN_NUM_CS,
        .host = TOUCH_SPI_HOST,
        .miso_io = TOUCH_PIN_NUM_MISO,
    };	
	
    // Initialize the SPI bus
    ESP_LOGI(TAG, "Initializing bus SPI2...");
    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_SPI_HOST, &buscfg, SPI_DMA_DISABLED));
	ESP_ERROR_CHECK(touch_spi_init(&touch_config, &touch_handle));
	
	// Load touch calibration coefficient from nvs
	ESP_ERROR_CHECK(nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle));  
	nvs_get_blob(nvs_handle, "touch_cal_coef", &touch_cal_coef, &touch_cal_coef_size);
	nvs_close(nvs_handle);
	
	ESP_LOGI(TAG, "nvs get touch_cal_coef: a = %.6f, b = %.6f, c = %.6f, d = %.6f,e = %.6f, f = %.6f", 
				  touch_cal_coef.a, touch_cal_coef.b, touch_cal_coef.c, touch_cal_coef.d, touch_cal_coef.e, touch_cal_coef.f);		
	
	return ESP_OK;
}

bool touch_is_pressed(uint16_t* z){
	uint16_t XPT2046_Z1, XPT2046_Z2;
	uint8_t SPI_RX_Data_Buf[2] = {0};
	
	if(touch_spi_read(touch_handle, XPT2046_CMD__GET_Z1, 16, SPI_RX_Data_Buf) != ESP_OK){return false;}
	XPT2046_Z1 = ((SPI_RX_Data_Buf[0] & 0x7F) << 5) | (SPI_RX_Data_Buf[1] >> 3);
	if(touch_spi_read(touch_handle, XPT2046_CMD__GET_Z2, 16, SPI_RX_Data_Buf) != ESP_OK){return false;}
	XPT2046_Z2 = ((SPI_RX_Data_Buf[0] & 0x7F) << 5) | (SPI_RX_Data_Buf[1] >> 3);
	
	if(z){*z = XPT2046_Z2 - XPT2046_Z1;}	
	if((XPT2046_Z2 - XPT2046_Z1) < XPT2046_Z_THRESHOLD){return true;}
	else{return false;}
}

esp_err_t touch_get_raw(uint16_t* x, uint16_t* y){
	uint16_t XPT2046_X, XPT2046_Y;
	uint8_t SPI_RX_Data_Buf[2] = {0};	
		
	if(touch_spi_read(touch_handle, XPT2046_CMD__GET_Y, 16, SPI_RX_Data_Buf) != ESP_OK){return ESP_FAIL;}
	XPT2046_Y = ((SPI_RX_Data_Buf[0] & 0x7F) << 5) | (SPI_RX_Data_Buf[1] >> 3);
	if(touch_spi_read(touch_handle, XPT2046_CMD__GET_X, 16, SPI_RX_Data_Buf) != ESP_OK){return ESP_FAIL;}
	XPT2046_X = ((SPI_RX_Data_Buf[0] & 0x7F) << 5) | (SPI_RX_Data_Buf[1] >> 3);	
	
	*x = XPT2046_X;
	*y = XPT2046_Y;
	return ESP_OK;	
}

esp_err_t touch_get_coordinate(uint16_t* x, uint16_t* y){
	uint16_t XPT2046_X, XPT2046_Y;
	
	if(touch_get_raw(&XPT2046_X, &XPT2046_Y) == ESP_OK){
		*x = ((float)XPT2046_X * touch_cal_coef.a + (float)XPT2046_Y * touch_cal_coef.b + touch_cal_coef.c);
		*y = ((float)XPT2046_X * touch_cal_coef.d + (float)XPT2046_Y * touch_cal_coef.e + touch_cal_coef.f);
		return ESP_OK;		
	}else{
		return ESP_FAIL;
	}
}

void touch_calibration_task_init(void){
	xTaskCreatePinnedToCore(touch_calibration_task, "touch_calibration_task", 4096, NULL, TASK_PRIORITY__TOUCH, NULL, TASK_CPU_CORE__TOUCH);
}

#define CALIBRATION_OFFSET		20
#define CALIBRATION_POINT_MAX	3
#define CALIBRATION_DONE		(CALIBRATION_POINT_MAX)
static void touch_calibration_task(void *pvParameter){
	lcd_event_t lcd_evt;
	uint16_t cal_coord_x[CALIBRATION_POINT_MAX] = {CALIBRATION_OFFSET, 
												   LCD_H_RES / 2,
												   LCD_H_RES - CALIBRATION_OFFSET};
	uint16_t cal_coord_y[CALIBRATION_POINT_MAX] = {LCD_V_RES / 2, 
												   CALIBRATION_OFFSET,
												   LCD_V_RES - CALIBRATION_OFFSET};
	uint16_t xpt2046_meas_x[CALIBRATION_POINT_MAX] = {0};
	uint16_t xpt2046_meas_y[CALIBRATION_POINT_MAX] = {0};	
	float divisor;	
	
	uint8_t touch_hold_count = 0;
	uint8_t calibration_state = 0;
	uint8_t i = 0;
	
    nvs_handle_t nvs_handle;
	
	ui__fill_color(lcd_frame_bitmap, BLACK);
	ui__draw_a_cross(lcd_frame_bitmap, RED, cal_coord_x[calibration_state], cal_coord_y[calibration_state], 10);
	lcd_evt.id = FLUSH_WITH_BITMAP;
	if(xQueueSend(lcd_event_queue, &lcd_evt, 0) != pdTRUE){
		ESP_LOGE(TAG, "Pushing FLUSH_WITH_BITMAP to lcd_event_queue fail");
	}		
	
	while(calibration_state < CALIBRATION_DONE){
		if(touch_hold_count < 20){
			if(touch_is_pressed(NULL) == true){touch_hold_count++;}
			else{touch_hold_count = 0;}
		}else if(touch_hold_count == 20){			
			touch_get_raw(&xpt2046_meas_x[calibration_state], &xpt2046_meas_y[calibration_state]);
			touch_hold_count++;
			ESP_LOGI(TAG, "cal_coord%d = (%d, %d), meas_coord%d = (%d, %d)\n", calibration_state, 
																			   cal_coord_x[calibration_state], 
																		       cal_coord_y[calibration_state],
																			   calibration_state, 
																			   xpt2046_meas_x[calibration_state],
																			   xpt2046_meas_y[calibration_state]);
			
			calibration_state++;
			ui__fill_color(lcd_frame_bitmap, BLACK);
			ui__draw_a_cross(lcd_frame_bitmap, RED, cal_coord_x[calibration_state], cal_coord_y[calibration_state], 10);
			lcd_evt.id = FLUSH_WITH_BITMAP;
			if(xQueueSend(lcd_event_queue, &lcd_evt, 0) != pdTRUE){
				ESP_LOGE(TAG, "Pushing FLUSH_WITH_BITMAP to lcd_event_queue fail");
			}						
		}else{
			if(touch_is_pressed(NULL) == false){
				if(++touch_hold_count >= 30){
					touch_hold_count = 0;
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	
	for(i = 0; i < CALIBRATION_POINT_MAX; i++){
		ESP_LOGI(TAG, "Calibratioin point %d: (%d, %d)", i, xpt2046_meas_x[i], xpt2046_meas_y[i]);
	}

    divisor = (float)xpt2046_meas_x[0] * ((float)xpt2046_meas_y[2] - (float)xpt2046_meas_y[1])
			- (float)xpt2046_meas_x[1] * (float)xpt2046_meas_y[2]
			+ (float)xpt2046_meas_y[1] * (float)xpt2046_meas_x[2]
			+ (float)xpt2046_meas_y[0] * ((float)xpt2046_meas_x[1] - (float)xpt2046_meas_x[2]);
	touch_cal_coef.a = ((float)cal_coord_x[0] * ((float)xpt2046_meas_y[2] - (float)xpt2046_meas_y[1])
						- (float)cal_coord_x[1] * (float)xpt2046_meas_y[2]
						+ (float)cal_coord_x[2] * (float)xpt2046_meas_y[1]
						+ ((float)cal_coord_x[1] - (float)cal_coord_x[2]) * (float)xpt2046_meas_y[0]) / divisor;
	touch_cal_coef.b = - ((float)cal_coord_x[0] * ((float)xpt2046_meas_x[2] - (float)xpt2046_meas_x[1])
					      - (float)cal_coord_x[1] * (float)xpt2046_meas_x[2]
					      + (float)cal_coord_x[2] * (float)xpt2046_meas_x[1]
					      + ((float)cal_coord_x[1] - (float)cal_coord_x[2]) * (float)xpt2046_meas_x[0]) / divisor;
	touch_cal_coef.c = ((float)cal_coord_x[0] * ((float)xpt2046_meas_y[1] * (float)xpt2046_meas_x[2] - (float)xpt2046_meas_x[1] * (float)xpt2046_meas_y[2])
						+ (float)xpt2046_meas_x[0] * ((float)cal_coord_x[1] * (float)xpt2046_meas_y[2] - (float)cal_coord_x[2] * (float)xpt2046_meas_y[1])
						+ (float)xpt2046_meas_y[0] * ((float)cal_coord_x[2] * (float)xpt2046_meas_x[1] - (float)cal_coord_x[1] * (float)xpt2046_meas_x[2])) / divisor;
	touch_cal_coef.d = ((float)cal_coord_y[0] * ((float)xpt2046_meas_y[2] - (float)xpt2046_meas_y[1])
						- (float)cal_coord_y[1] * (float)xpt2046_meas_y[2]
						+ (float)cal_coord_y[2] * (float)xpt2046_meas_y[1]
						+ ((float)cal_coord_y[1] - (float)cal_coord_y[2]) * (float)xpt2046_meas_y[0]) / divisor;
	touch_cal_coef.e = - ((float)cal_coord_y[0] * ((float)xpt2046_meas_x[2] - (float)xpt2046_meas_x[1])
						  - (float)cal_coord_y[1] * (float)xpt2046_meas_x[2]
						  + (float)cal_coord_y[2] * (float)xpt2046_meas_x[1]
						  + ((float)cal_coord_y[1] - (float)cal_coord_y[2]) * (float)xpt2046_meas_x[0]) / divisor;
	touch_cal_coef.f = ((float)cal_coord_y[0] * ((float)xpt2046_meas_y[1] * (float)xpt2046_meas_x[2] - (float)xpt2046_meas_x[1] * (float)xpt2046_meas_y[2])
						+ (float)xpt2046_meas_x[0] * ((float)cal_coord_y[1] * (float)xpt2046_meas_y[2] - (float)cal_coord_y[2] * (float)xpt2046_meas_y[1])
						+ (float)xpt2046_meas_y[0] * ((float)cal_coord_y[2] * (float)xpt2046_meas_x[1] - (float)cal_coord_y[1] * (float)xpt2046_meas_x[2])) / divisor;
	
	ESP_ERROR_CHECK(nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle));	
	ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "touch_cal_coef", &touch_cal_coef, sizeof(touch_calibration_coef_t)));
	ESP_ERROR_CHECK(nvs_commit(nvs_handle));
	nvs_close(nvs_handle);
	ESP_LOGI(TAG, "nvs_set: a = %.6f, b = %.6f, c = %.6f, d = %.6f,e = %.6f, f = %.6f", 
				  touch_cal_coef.a, touch_cal_coef.b, touch_cal_coef.c, touch_cal_coef.d, touch_cal_coef.e, touch_cal_coef.f);	
				  
	vTaskResume(remote_ctrl_task_handle);
	vTaskDelete(NULL);
}