#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "main.h"
#include "fire_ctrl_task.h"
#include "orientation_ctrl_task.h"
#include "speaker_ctrl_task.h"

static const char *TAG = "fire_ctrl";

#define MOTOR_PWM_TIMER			LEDC_TIMER_0
#define MOTOR_PWM_MODE			LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_DUTY_RES		LEDC_TIMER_13_BIT 
#define MOTOR_PWM_FREQUENCY		(5000) 

#define AEG_MOTOR_PWM_DUTY_DEFAULT	((CONFIG_AEG_MOTOR_EFFECTIVE_VOLTAGE * 100) / CONFIG_AIRSOFT_BATTERY_VOLTAGE)
#define AEG_MOTOR_PWM_ADJUST_RANGE	10
#define AF_MOTOR_PWM_DUTY			((CONFIG_AF_MOTOR_EFFECTIVE_VOLTAGE * 100) / CONFIG_AIRSOFT_BATTERY_VOLTAGE)	// AF: Ammo Feeder
uint8_t aeg_motor_pwm_duty_min, aeg_motor_pwm_duty_max;
uint8_t aeg_motor_pwm_duty_L = AEG_MOTOR_PWM_DUTY_DEFAULT;
uint8_t aeg_motor_pwm_duty_R = AEG_MOTOR_PWM_DUTY_DEFAULT;

#define FIRE_CTRL_L_OUTPUT_IO	(18)
#define FIRE_CTRL_L_CHANNEL		LEDC_CHANNEL_0
#define FIRE_CTRL_R_OUTPUT_IO	(8) 				
#define FIRE_CTRL_R_CHANNEL		LEDC_CHANNEL_1
#define AF_CTRL_OUTPUT_IO		(11) 				
#define AF_CTRL_CHANNEL			LEDC_CHANNEL_2
ledc_channel_config_t aeg_motor_pwm_L = {
	.speed_mode     = MOTOR_PWM_MODE,
	.channel        = FIRE_CTRL_L_CHANNEL,
	.timer_sel      = MOTOR_PWM_TIMER,
	.intr_type      = LEDC_INTR_DISABLE,
	.gpio_num       = FIRE_CTRL_L_OUTPUT_IO,
	.duty           = 0,
	.hpoint         = 0
};
ledc_channel_config_t aeg_motor_pwm_R = {
	.speed_mode     = MOTOR_PWM_MODE,
	.channel        = FIRE_CTRL_R_CHANNEL,
	.timer_sel      = MOTOR_PWM_TIMER,
	.intr_type      = LEDC_INTR_DISABLE,
	.gpio_num       = FIRE_CTRL_R_OUTPUT_IO,
	.duty           = 0,
	.hpoint         = 0
};	
ledc_channel_config_t af_motor_pwm = {
	.speed_mode     = MOTOR_PWM_MODE,
	.channel        = AF_CTRL_CHANNEL,
	.timer_sel      = MOTOR_PWM_TIMER,
	.intr_type      = LEDC_INTR_DISABLE,
	.gpio_num       = AF_CTRL_OUTPUT_IO,
	.duty           = 0,
	.hpoint         = 0
};	
	
#define RED_DOT_L_CTRL_PIN_NUM		10	
#define RED_DOT_L_CTRL_PIN_SEL		(1ULL << RED_DOT_L_CTRL_PIN_NUM)	
#define RED_DOT_R_CTRL_PIN_NUM		9	
#define RED_DOT_R_CTRL_PIN_SEL		(1ULL << RED_DOT_R_CTRL_PIN_NUM)
#define SPRING_RELEASED_L_PIN_NUM	3	
#define SPRING_RELEASED_L_PIN_SEL	(1ULL << SPRING_RELEASED_L_PIN_NUM)
#define SPRING_RELEASED_R_PIN_NUM	46	
#define SPRING_RELEASED_R_PIN_SEL	(1ULL << SPRING_RELEASED_R_PIN_NUM)

#define STATUS__SAFETY_EN			0x01
#define STATUS__IS_FIRING			0x02
#define STATUS__L_FIRST_ROUND		0x04
#define STATUS__R_FIRST_ROUND		0x08
#define STATUS__L_SPRING_RELEASED	0x10
#define STATUS__R_SPRING_RELEASED	0x20
#define STATUS__SPRING_JAMMING		0x40
uint8_t fire_ctrl_status = STATUS__SAFETY_EN | STATUS__L_SPRING_RELEASED | STATUS__R_SPRING_RELEASED;

#define FIRE_BURST_COUNT		3

#define FIRE_PEROID				250		// in ms, "250" means each airsoft fires 4 rounds per second.
#define FIRE_PEROID_TOLERANCE	10
uint16_t fire_ctrl_timer_L = 0, fire_ctrl_timer_R = 0;
uint8_t remain_round_L = 0, remain_round_R = 0;
uint16_t measure_period_L = 0, measure_period_R = 0;
#define DEBOUNCE_COUNT_MAX		3	
uint8_t fire_ctrl_debounce_L = 0, fire_ctrl_debounce_R = 0;

#define FIRING_NOTIFY_COOLDOWN_MAX	10000
uint16_t firing_notify_cooldown = FIRING_NOTIFY_COOLDOWN_MAX;

#define FIRE_CTRL_EVENT_QUEUE_SIZE	1
QueueHandle_t fire_ctrl_event_queue;

#define TASK_SCHEDULED_PERIOD	50
static void fire_ctrl_task(void *pvParameter);

esp_err_t fire_ctrl__init(void){

	gpio_config_t io_conf = {};
		
	io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = RED_DOT_L_CTRL_PIN_SEL | RED_DOT_R_CTRL_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	gpio_set_level(RED_DOT_L_CTRL_PIN_NUM, 0);
	gpio_set_level(RED_DOT_R_CTRL_PIN_NUM, 0);
	
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = SPRING_RELEASED_L_PIN_SEL | SPRING_RELEASED_R_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
	
    ledc_timer_config_t motor_pwm_timer = {
        .speed_mode       = MOTOR_PWM_MODE,
        .timer_num        = MOTOR_PWM_TIMER,
        .duty_resolution  = MOTOR_PWM_DUTY_RES,
        .freq_hz          = MOTOR_PWM_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };	
    ESP_ERROR_CHECK(ledc_timer_config(&motor_pwm_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&aeg_motor_pwm_L));	
    ESP_ERROR_CHECK(ledc_channel_config(&aeg_motor_pwm_R));		
    ESP_ERROR_CHECK(ledc_channel_config(&af_motor_pwm));

	aeg_motor_pwm_duty_min = (AEG_MOTOR_PWM_DUTY_DEFAULT < AEG_MOTOR_PWM_ADJUST_RANGE)? 0: AEG_MOTOR_PWM_DUTY_DEFAULT - AEG_MOTOR_PWM_ADJUST_RANGE;
	aeg_motor_pwm_duty_max = (AEG_MOTOR_PWM_DUTY_DEFAULT > 100 - AEG_MOTOR_PWM_ADJUST_RANGE)? 100: AEG_MOTOR_PWM_DUTY_DEFAULT + AEG_MOTOR_PWM_ADJUST_RANGE;
	
	fire_ctrl_event_queue = xQueueCreate(FIRE_CTRL_EVENT_QUEUE_SIZE, sizeof(fire_ctrl_event_t));
	if (fire_ctrl_event_queue == NULL) {
		ESP_LOGE(TAG, "Create fire_ctrl_event_queue fail");
		return ESP_FAIL;
	}			
		
	xTaskCreatePinnedToCore(fire_ctrl_task, "fire_ctrl_task", 4096, NULL, TASK_PRIORITY__FIRE_CTRL, NULL, TASK_CPU_CORE__FIRE_CTRL);
	
	return ESP_OK;	
}

bool fire_ctrl__is_firing(void){	
	if(fire_ctrl_status & STATUS__IS_FIRING){return true;}
	else{return false;}
}

static void fire_ctrl_task(void *pvParameter){
	fire_ctrl_event_t fc_evt;
	orientation_ctrl_event_t oc_evt;
	
	while(1){
		if((xQueueReceive(fire_ctrl_event_queue, &fc_evt, 0)) == pdTRUE){
			switch (fc_evt.id){
				case OPEN_FIRE:
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED){
						if(fire_ctrl_status & STATUS__SAFETY_EN){
							remain_round_L = FIRE_BURST_COUNT;
							remain_round_R = FIRE_BURST_COUNT;				
						}else{
							if(!(fire_ctrl_status & STATUS__SPRING_JAMMING)){
								if(!(fire_ctrl_status & STATUS__IS_FIRING)){
									fire_ctrl_status |= STATUS__IS_FIRING;
									if(firing_notify_cooldown >= FIRING_NOTIFY_COOLDOWN_MAX){									
										speaker_ctrl__play_music(SOUND_TRACK__TARGET_ACQUIRED);
										vTaskDelay(pdMS_TO_TICKS(1500));
									}	
									ledc_set_duty(MOTOR_PWM_MODE, AF_CTRL_CHANNEL, (((1 << LEDC_TIMER_13_BIT) - 1) * AF_MOTOR_PWM_DUTY) / 100);
									ledc_update_duty(MOTOR_PWM_MODE, AF_CTRL_CHANNEL);							
									ledc_set_duty(MOTOR_PWM_MODE, FIRE_CTRL_L_CHANNEL, (((1 << LEDC_TIMER_13_BIT) - 1) * aeg_motor_pwm_duty_L) / 100);
									ledc_update_duty(MOTOR_PWM_MODE, FIRE_CTRL_L_CHANNEL);							
									fire_ctrl_timer_L = 0;
									fire_ctrl_status |= STATUS__L_FIRST_ROUND;									
									vTaskDelay(pdMS_TO_TICKS(FIRE_PEROID / 2));
									ledc_set_duty(MOTOR_PWM_MODE, FIRE_CTRL_R_CHANNEL, (((1 << LEDC_TIMER_13_BIT) - 1) * aeg_motor_pwm_duty_R) / 100);
									ledc_update_duty(MOTOR_PWM_MODE, FIRE_CTRL_R_CHANNEL);							
									fire_ctrl_timer_R = 0;
									fire_ctrl_status |= STATUS__R_FIRST_ROUND;								
																	
								}
								remain_round_L = FIRE_BURST_COUNT;
								remain_round_R = FIRE_BURST_COUNT;
								firing_notify_cooldown = 0;	
							}										
						}
					}else{
						oc_evt.id = SWITCH_TELESCOPIC_ARM;
						oc_evt.info = NULL;
						if(xQueueSend(orientation_ctrl_event_queue, &oc_evt, 0) != pdTRUE){
							ESP_LOGW(TAG, "Pushing SET_TELESCOPIC_ARM to orientation_ctrl_event_queue fail");
						}												
					}
				break;
				case CEASE_FIRE:
					if(!(fire_ctrl_status & STATUS__IS_FIRING)){
						if(fire_ctrl_status & STATUS__SPRING_JAMMING){fire_ctrl_status &= ~STATUS__SPRING_JAMMING;}
					}
				break;
				case SWITCH_SAFETY:
					if(fire_ctrl_status & STATUS__SAFETY_EN){
						fire_ctrl_status &= ~STATUS__SAFETY_EN;											
					}else{
						fire_ctrl_status |= STATUS__SAFETY_EN;			
					}
				break;
				default:
					ESP_LOGW(TAG, "Unknown event id: %d", fc_evt.id);
				break;
			}
		}

		if(firing_notify_cooldown < FIRING_NOTIFY_COOLDOWN_MAX){
			firing_notify_cooldown += TASK_SCHEDULED_PERIOD;
			if(firing_notify_cooldown == FIRING_NOTIFY_COOLDOWN_MAX){
				speaker_ctrl__play_music(SOUND_TRACK__ARE_YOU_STILL_THERE);
			}
		}
		
		if(!(fire_ctrl_status & STATUS__SAFETY_EN) && 
		   (orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED)){
			gpio_set_level(RED_DOT_L_CTRL_PIN_NUM, 1);
			gpio_set_level(RED_DOT_R_CTRL_PIN_NUM, 1);		
		}else{
			gpio_set_level(RED_DOT_L_CTRL_PIN_NUM, 0);
			gpio_set_level(RED_DOT_R_CTRL_PIN_NUM, 0);			
		}
		
		vTaskDelay(pdMS_TO_TICKS(TASK_SCHEDULED_PERIOD));
	}
}

void motor_pwm_ctrl(ledc_channel_config_t* ch_config, uint16_t* fire_ctrl_timer, uint8_t* fire_ctrl_debounce, uint16_t* measure_period, uint8_t* motor_pwm_duty, 
					uint8_t* remain_round, uint8_t spring_released_pin, uint8_t spring_released_flag, uint8_t first_round_flag){
	if(*remain_round){	
		(*fire_ctrl_timer) += (FIRE_CTRL_ISR_PERIOD_US / 1000);
		if(fire_ctrl_status & spring_released_flag){
			if(!gpio_get_level(spring_released_pin)){
			// Airsoft's spring is already released	
				*fire_ctrl_debounce = 0;
				if((*fire_ctrl_timer) > FIRE_PEROID * 2){
					speaker_ctrl__play_music(SOUND_TRACK__MALFUNCTION);			
					fire_ctrl_status |= STATUS__SPRING_JAMMING;
				}				
			}else{
			// Airsoft start pressing spring				
				if(++(*fire_ctrl_debounce) > DEBOUNCE_COUNT_MAX){
					fire_ctrl_status &= ~spring_released_flag;
					fire_ctrl_debounce = 0;
				}				
			}
		}else{
			if(!gpio_get_level(spring_released_pin)){
				if(++(*fire_ctrl_debounce) > DEBOUNCE_COUNT_MAX){
				// Airsoft's spring is just released	
					fire_ctrl_status |= spring_released_flag;
					
					if(fire_ctrl_status & first_round_flag){
						fire_ctrl_status &= ~first_round_flag;
						(*measure_period) = 0;
					}else{
						(*measure_period) = (*fire_ctrl_timer);
						// Adjust airsoft motor speed to meet FIRE_PEROID
						if((*fire_ctrl_timer) > FIRE_PEROID){
							if((*fire_ctrl_timer) - FIRE_PEROID > FIRE_PEROID_TOLERANCE){
								if((*motor_pwm_duty) < aeg_motor_pwm_duty_max){
									((*motor_pwm_duty))++;
									ledc_set_duty(MOTOR_PWM_MODE, ch_config->channel, (((1 << LEDC_TIMER_13_BIT) - 1) * (*motor_pwm_duty)) / 100);
									ledc_update_duty(MOTOR_PWM_MODE, ch_config->channel);
								}
							}
						}else{
							if(FIRE_PEROID - (*fire_ctrl_timer) > FIRE_PEROID_TOLERANCE){
								if((*motor_pwm_duty) > aeg_motor_pwm_duty_min){
									((*motor_pwm_duty))--;
									ledc_set_duty(MOTOR_PWM_MODE, ch_config->channel, (((1 << LEDC_TIMER_13_BIT) - 1) * (*motor_pwm_duty)) / 100);
									ledc_update_duty(MOTOR_PWM_MODE, ch_config->channel);								
								}							
							}
						}
					}				
					(*fire_ctrl_timer) = 0;				
					
					if(ch_config == &aeg_motor_pwm_L){
						ESP_LOGD(TAG, "L_Period: %d, L_duty: %d", (*measure_period), (*motor_pwm_duty));					
					}else{
						ESP_LOGD(TAG, "R_Period: %d, R_duty: %d", (*measure_period), (*motor_pwm_duty));					
					}				
					
					if(--((*remain_round)) == 0){
						ledc_set_duty(MOTOR_PWM_MODE, ch_config->channel, 0);
						ledc_update_duty(MOTOR_PWM_MODE, ch_config->channel);				
					}	
					
					fire_ctrl_debounce = 0;
				}
			}else{
			// Airsoft is still pressing spring	
				fire_ctrl_debounce = 0;
				if((*fire_ctrl_timer) > FIRE_PEROID * 2){
					speaker_ctrl__play_music(SOUND_TRACK__MALFUNCTION);			
					fire_ctrl_status |= STATUS__SPRING_JAMMING;
				}				
			}			
		}
	}			
}

void fire_ctrl__isr(void){	
	if(remain_round_L || remain_round_R){
		if(fire_ctrl_status & STATUS__SAFETY_EN){
			// Flashing LEDs if airsoft safety is locked.
			if(global_timer_us_count % 100000 < 50000){
				gpio_set_level(RED_DOT_L_CTRL_PIN_NUM, 0);
				gpio_set_level(RED_DOT_R_CTRL_PIN_NUM, 0);
				if(global_timer_us_count % 100000 == 0){
					remain_round_L--;
					remain_round_R--;
				}				
			}else{
				gpio_set_level(RED_DOT_L_CTRL_PIN_NUM, 1);
				gpio_set_level(RED_DOT_R_CTRL_PIN_NUM, 1);
			}
		}else{			
			motor_pwm_ctrl(&aeg_motor_pwm_L, &fire_ctrl_timer_L, &fire_ctrl_debounce_L, &measure_period_L, &aeg_motor_pwm_duty_L, &remain_round_L, 
						   SPRING_RELEASED_L_PIN_NUM, STATUS__L_SPRING_RELEASED, STATUS__L_FIRST_ROUND);
			motor_pwm_ctrl(&aeg_motor_pwm_R, &fire_ctrl_timer_R, &fire_ctrl_debounce_R, &measure_period_R, &aeg_motor_pwm_duty_R, &remain_round_R, 
						   SPRING_RELEASED_R_PIN_NUM, STATUS__R_SPRING_RELEASED, STATUS__R_FIRST_ROUND);
			if(fire_ctrl_status & STATUS__SPRING_JAMMING){
				ledc_set_duty(MOTOR_PWM_MODE, af_motor_pwm.channel, 0);
				ledc_update_duty(MOTOR_PWM_MODE, af_motor_pwm.channel);				
				ledc_set_duty(MOTOR_PWM_MODE, aeg_motor_pwm_L.channel, 0);
				ledc_update_duty(MOTOR_PWM_MODE, aeg_motor_pwm_L.channel);
				ledc_set_duty(MOTOR_PWM_MODE, aeg_motor_pwm_R.channel, 0);
				ledc_update_duty(MOTOR_PWM_MODE, aeg_motor_pwm_R.channel);					
				fire_ctrl_status |= (STATUS__L_SPRING_RELEASED | STATUS__R_SPRING_RELEASED);
				remain_round_L = 0;
				remain_round_R = 0;			
			}	
			if((remain_round_L == 0) && (remain_round_R == 0)){				
				ledc_set_duty(MOTOR_PWM_MODE, af_motor_pwm.channel, 0);
				ledc_update_duty(MOTOR_PWM_MODE, af_motor_pwm.channel);				
				fire_ctrl_status &= ~STATUS__IS_FIRING;
			}			
		}
	}
}