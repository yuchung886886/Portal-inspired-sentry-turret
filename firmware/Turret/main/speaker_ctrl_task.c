#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "speaker_ctrl_task.h"
#include "main.h"

/* I2S port and GPIOs */
#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_13)
#define I2S_WS_IO       (GPIO_NUM_14)
#define I2S_DO_IO       (GPIO_NUM_12)

#define SOUNDTRACK_SAMPLE_RATE     (44100)
static i2s_chan_handle_t i2s_tx_handle = NULL;

uint8_t speaker_ctrl_status = 0;

#define SPEAKER_CTRL_EVENT_QUEUE_SIZE		1
QueueHandle_t speaker_ctrl_event_queue;

extern const uint8_t turret_deploy_pcm_start[] 			asm("_binary_turret_deploy_pcm_start");
extern const uint8_t turret_deploy_pcm_end[]   			asm("_binary_turret_deploy_pcm_end");
extern const uint8_t turret_alarm_pcm_start[]			asm("_binary_turret_alarm_pcm_start");
extern const uint8_t turret_alarm_pcm_end[]				asm("_binary_turret_alarm_pcm_end");
extern const uint8_t turret_hello_pcm_start[]			asm("_binary_turret_hello_pcm_start");
extern const uint8_t turret_hello_pcm_end[]				asm("_binary_turret_hello_pcm_end");
extern const uint8_t turret_anyone_there_pcm_start[] 	asm("_binary_turret_anyone_there_pcm_start");
extern const uint8_t turret_anyone_there_pcm_end[]   	asm("_binary_turret_anyone_there_pcm_end");
extern const uint8_t turret_target_acquired_start[]		asm("_binary_turret_target_acquired_pcm_start");
extern const uint8_t turret_target_acquired_end[]		asm("_binary_turret_target_acquired_pcm_end");
extern const uint8_t turret_malfunction_start[]			asm("_binary_turret_malfunction_pcm_start");
extern const uint8_t turret_malfunction_end[]   		asm("_binary_turret_malfunction_pcm_end");
extern const uint8_t turret_retract_pcm_start[]			asm("_binary_turret_retract_pcm_start");
extern const uint8_t turret_retract_pcm_end[]			asm("_binary_turret_retract_pcm_end");
extern const uint8_t turret_goodnight_pcm_start[]		asm("_binary_turret_goodnight_pcm_start");
extern const uint8_t turret_goodnight_pcm_end[]			asm("_binary_turret_goodnight_pcm_end");
extern const uint8_t turret_still_there_pcm_start[]		asm("_binary_turret_are_you_still_there_pcm_start");
extern const uint8_t turret_still_there_pcm_end[]		asm("_binary_turret_are_you_still_there_pcm_end");

typedef struct {
    const uint8_t* start;
    const uint8_t* end;
} sound_track_t;
sound_track_t sound_tracks_list[] = {
	[SOUND_TRACK__DEPLOY] = {.start = turret_deploy_pcm_start, .end = turret_deploy_pcm_end},	
	[SOUND_TRACK__ALARM] = {.start = turret_alarm_pcm_start, .end = turret_alarm_pcm_end},	
	[SOUND_TRACK__HELLO] = {.start = turret_hello_pcm_start, .end = turret_hello_pcm_end},
	[SOUND_TRACK__ANYONE_THERE] = {.start = turret_anyone_there_pcm_start, .end = turret_anyone_there_pcm_end},
	[SOUND_TRACK__TARGET_ACQUIRED] = {.start = turret_target_acquired_start, .end = turret_target_acquired_end},
	[SOUND_TRACK__MALFUNCTION] = {.start = turret_malfunction_start, .end = turret_malfunction_end},
	[SOUND_TRACK__RETRACT] = {.start = turret_retract_pcm_start, .end = turret_retract_pcm_end},
	[SOUND_TRACK__GOODNIGHT] = {.start = turret_goodnight_pcm_start, .end = turret_goodnight_pcm_end},
	[SOUND_TRACK__ARE_YOU_STILL_THERE] = {.start = turret_still_there_pcm_start, .end = turret_still_there_pcm_end}
};

static const char *TAG = "speaker_ctrl";

#define TASK_SCHEDULED_PERIOD	100
static void speaker_ctrl_task(void *pvParameter);

static esp_err_t i2s_driver_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);	
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_handle, NULL));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SOUNDTRACK_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_handle));
    return ESP_OK;
}

esp_err_t speaker_ctrl_task_init(void){
	
	ESP_ERROR_CHECK(i2s_driver_init());

	speaker_ctrl_event_queue = xQueueCreate(SPEAKER_CTRL_EVENT_QUEUE_SIZE, sizeof(speaker_ctrl_event_t));
	if (speaker_ctrl_event_queue == NULL) {
		ESP_LOGE(TAG, "Create mutex fail");
		return ESP_FAIL;
	}			
		
	xTaskCreatePinnedToCore(speaker_ctrl_task, "speaker_ctrl_task", 4096, NULL, TASK_PRIORITY__SPEAKER_CTRL, NULL, TASK_CPU_CORE__SPEAKER_CTRL);
	
	return ESP_OK;
}

esp_err_t speaker_ctrl__play_music(uint8_t track_index){
	speaker_ctrl_event_t sc_evt;
	
	if(speaker_ctrl_status & SPEAKER_CTRL_STATUS__BUSY){
		ESP_LOGW(TAG, "i2s is busy.");
		return ESP_FAIL;
	}else{
		speaker_ctrl_status |= SPEAKER_CTRL_STATUS__BUSY;
		sc_evt.id = PLAY_MUSIC;
		sc_evt.info = &sound_tracks_list[track_index];
		if(xQueueSend(speaker_ctrl_event_queue, &sc_evt, 0) != pdTRUE){
			ESP_LOGW(TAG, "Pushing sc_evt to speaker_ctrl_event_queue fail");
		}		          
	}	
	return ESP_OK;
}

static void speaker_ctrl_task(void *pvParameter){
	speaker_ctrl_event_t sc_evt;
	sound_track_t* sound_track_ptr; 
	size_t i2s_bytes_written = 0;
	
	while(1){
		if((xQueueReceive(speaker_ctrl_event_queue, &sc_evt, 0)) == pdTRUE){
			switch (sc_evt.id){
				case PLAY_MUSIC:
					if(sc_evt.info){
						sound_track_ptr = (sound_track_t*)(sc_evt.info);
						i2s_channel_write(i2s_tx_handle, 
										  sound_track_ptr->start, 
										  sound_track_ptr->end - sound_track_ptr->start, 
										  &i2s_bytes_written,
										  portMAX_DELAY);
						if(!i2s_bytes_written){
							ESP_LOGE(TAG, "i2s music play falied.");
						}
						speaker_ctrl_status	&= ~SPEAKER_CTRL_STATUS__BUSY;
					}					
				break;
				default:
				break;
			}
		}			
		vTaskDelay(pdMS_TO_TICKS(TASK_SCHEDULED_PERIOD));
	}
}

