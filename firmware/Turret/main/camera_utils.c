#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "main.h"

static const char *TAG = "camera_utils";	

#define CAM_PIN_PWDN    1
#define CAM_PIN_RESET   6
#define CAM_PIN_XCLK    -1
#define CAM_PIN_SIOD    17
#define CAM_PIN_SIOC    18

#define CAM_PIN_D7      17
#define CAM_PIN_D6      42
#define CAM_PIN_D5      16
#define CAM_PIN_D4      41
#define CAM_PIN_D3      15
#define CAM_PIN_D2      40
#define CAM_PIN_D1      7
#define CAM_PIN_D0      39
#define CAM_PIN_VSYNC   4
#define CAM_PIN_HREF    5
#define CAM_PIN_PCLK    2

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = -1,
    .pin_sccb_scl = -1,
	.sccb_i2c_port = I2C_PORT_DEFAULT,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 24000000, // This setting is no use for the ALIENTEK OV5640 module with a built-in crystal.
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1, // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY // CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

esp_err_t camera_init(){
	gpio_config_t io_conf = {};
	
    // Power up the camera if PWDN pin is defined
    if(CAM_PIN_PWDN != -1){
		io_conf.intr_type = GPIO_INTR_DISABLE;
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = 1LL << camera_config.pin_pwdn;;
		io_conf.pull_down_en = 0;
		io_conf.pull_up_en = 0;
		gpio_config(&io_conf);	
		gpio_set_level(camera_config.pin_pwdn, 0);	
    }	

    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
	
	sensor_t *s = esp_camera_sensor_get();
	if (s != NULL) {
		s->set_hmirror(s, 1);
		s->set_brightness(s, -1);
		s->set_saturation(s, 2);
	}
	
    return ESP_OK;
}

esp_err_t camera_capture(){
    // Acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }
	ESP_LOGD(TAG, "fb.len = %d", fb->len);
  
    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
}



