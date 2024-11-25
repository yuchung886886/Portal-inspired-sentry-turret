#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "../../Common/common.h"
#include "main.h"
#include "orientation_ctrl_task.h"
#include "speaker_ctrl_task.h"

static const char *TAG = "orientation_ctrl";

#define MOTOR_TELESCOPE_STEP_PIN_NUM	45	
#define MOTOR_TELESCOPE_STEP_PIN_SEL	(1ULL << MOTOR_TELESCOPE_STEP_PIN_NUM)
#define MOTOR_TILT_STEP_PIN_NUM			48
#define MOTOR_TILT_STEP_PIN_SEL			(1ULL << MOTOR_TILT_STEP_PIN_NUM)
#define MOTOR_PAN_STEP_PIN_NUM			47
#define MOTOR_PAN_STEP_PIN_SEL			(1ULL << MOTOR_PAN_STEP_PIN_NUM)
#define MOTOR_TELESCOPE_ENDSTOP_PIN_NUM	21 
#define MOTOR_TELESCOPE_ENDSTOP_PIN_SEL	(1ULL << MOTOR_TELESCOPE_ENDSTOP_PIN_NUM)
#define MOTOR_TILT_ENDSTOP_PIN_NUM		20 
#define MOTOR_TILT_ENDSTOP_PIN_SEL		(1ULL << MOTOR_TILT_ENDSTOP_PIN_NUM)
#define MOTOR_PAN_ENDSTOP_PIN_NUM		19
#define MOTOR_PAN_ENDSTOP_PIN_SEL		(1ULL << MOTOR_PAN_ENDSTOP_PIN_NUM)

#define MOTOR_TELESCOPIC_ARM_DIR__CW		0x00	// Retract telescopic arms
#define MOTOR_TELESCOPIC_ARM_DIR__CCW	0x01	// Extend telescopic arms
#define MOTOR_TILT_DIR__CW			0x00	// Tilt up
#define MOTOR_TILT_DIR__CCW			0x02	// Tilt down
#define MOTOR_PAN_DIR__CW			0x00	// Pan counter-clockwise
#define MOTOR_PAN_DIR__CCW			0x04	// Pan clockwise
#define MOTOR_DISABLE				0x08
static uint8_t PCF8574_output_status = 0x00;

#define MOTOR_TELESCOPE_STEP_PIN__HIGH		0x01
#define MOTOR_TILT_STEP_PIN__HIGH			0x02
#define MOTOR_PAN_STEP_PIN__HIGH			0x04
static uint8_t motors_step_pin_level = 0x00;

uint16_t orientation_ctrl_status = ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN | 
								   ORI_CTRL_STATUS__PAN_RIGHT_ENDSTOP_ALARM_EN | 
								   ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN |
								   ORI_CTRL_STATUS__TILT_DOWN_ENDSTOP_ALARM_EN;

#define MOTOR_TILT_RESET_ANGLE_IN_DEGREE		15
#define MOTOR_PAN_RESET_ANGLE_IN_DEGREE			60
int16_t motor_telescopic_arms_steps_remain = 0, motor_telescopic_arms_extended_steps = 0;
int16_t motor_tilt_steps_remain = 0, motor_tilt_angle_in_steps = CONFIG_MOTOR_TILT_RESET_STEPS * 2;
int16_t motor_pan_steps_remain = 0, motor_pan_angle_in_steps = CONFIG_MOTOR_PAN_RESET_STEPS * 2;

#define MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD	1
uint8_t motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD;
uint8_t motor_telescopic_arms_timer = 0;

#define MOTOR_TILT_CONST_HALF_PERIOD	1
#define MOTOR_TILT_DEC_STEPS_COUNT		32
uint8_t motor_tilt_dec_steps_half_period_lookup[MOTOR_TILT_DEC_STEPS_COUNT] = {8, 8, 8, 8, 8, 8,
																			   7, 7, 7, 7, 7,
																			   6, 6, 6, 
																			   5, 5, 5,
																			   4, 4, 4, 4,
																			   3, 3, 3, 3, 3,
																			   2, 2, 2, 2, 2, 2};
uint8_t motor_tilt_half_period = MOTOR_TILT_CONST_HALF_PERIOD;
uint8_t motor_tilt_timer = 0;

#define MOTOR_PAN_CONST_HALF_PERIOD		1
#define MOTOR_PAN_DEC_STEPS_COUNT		32
uint8_t motor_pan_dec_steps_half_period_lookup[MOTOR_TILT_DEC_STEPS_COUNT] = {8, 8, 8, 8, 8, 8,
																			  7, 7, 7, 7, 7,
																			  6, 6, 6, 
																			  5, 5, 5,
																			  4, 4, 4, 4,
																			  3, 3, 3, 3, 3,
																			  2, 2, 2, 2, 2, 2};
uint8_t motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD;
uint8_t motor_pan_timer = 0;

#define ORIENTATION_CTRL_EVENT_QUEUE_SIZE	3
QueueHandle_t orientation_ctrl_event_queue;

#define TASK_SCHEDULED_PERIOD	50
static void orientation_ctrl_task(void *pvParameter);
static esp_err_t pcf8574_set_output_pins(uint8_t pin_mask);
static esp_err_t extend_telescopic_arms(void);
static esp_err_t retract_telescopic_arms(void);
static esp_err_t pan_tilt_to_reset_angle(void);
static esp_err_t timeout_singing_init(void);
static esp_err_t pan_at_timeout_singing(void);

esp_err_t orientation_ctrl_init(void){
	esp_err_t ret = ESP_OK;
	
	gpio_config_t io_conf = {};		
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = MOTOR_TELESCOPE_STEP_PIN_SEL | MOTOR_TILT_STEP_PIN_SEL | MOTOR_PAN_STEP_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	motors_step_pin_level = 0x00;
	gpio_set_level(MOTOR_TELESCOPE_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_TELESCOPE_STEP_PIN__HIGH) ? 1 : 0);
	gpio_set_level(MOTOR_TILT_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_TILT_STEP_PIN__HIGH) ? 1 : 0);
	gpio_set_level(MOTOR_PAN_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_PAN_STEP_PIN__HIGH) ? 1 : 0);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = MOTOR_TELESCOPE_ENDSTOP_PIN_SEL | MOTOR_TILT_ENDSTOP_PIN_SEL | MOTOR_PAN_ENDSTOP_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
		
	PCF8574_output_status = MOTOR_TELESCOPIC_ARM_DIR__CCW | MOTOR_TILT_DIR__CW | MOTOR_PAN_DIR__CW;
	if((ret = pcf8574_set_output_pins(PCF8574_output_status)) != ESP_OK){
		ESP_LOGE(TAG, "PCF8574 set output fail");
		return ret;
	}
	
	#define INIT_STATE__TELESCOPE_ENDSTOP_SEARCH	0
	#define INIT_STATE__TILT_PAN_ENDSTOP_SEARCH		1	
	#define INIT_STATE__TILT_PAN_RESET				2
	#define INIT_STATE__DONE						3
	uint8_t initialize_state = INIT_STATE__TELESCOPE_ENDSTOP_SEARCH;	
	motor_telescopic_arms_extended_steps = 0;
	extend_telescopic_arms();
	while(initialize_state != INIT_STATE__DONE){
		switch(initialize_state){
			case INIT_STATE__TELESCOPE_ENDSTOP_SEARCH:
				if(gpio_get_level(MOTOR_TELESCOPE_ENDSTOP_PIN_NUM)){
					if(motor_telescopic_arms_steps_remain == 0){
						// TBD, endstop switch not reached.
					}
				}else{
					orientation_ctrl_status |= ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED;
					motor_tilt_steps_remain = CONFIG_MOTOR_TILT_RESET_STEPS * 2;
					motor_tilt_half_period = MOTOR_TILT_CONST_HALF_PERIOD;
					motor_tilt_timer = 0;	
					motor_pan_steps_remain = CONFIG_MOTOR_PAN_RESET_STEPS * 2;
					motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD;
					motor_pan_timer = 0;					
					ESP_LOGD(TAG, "Telescope motor run %d steps", CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS * 2 - motor_telescopic_arms_steps_remain);
					initialize_state = INIT_STATE__TILT_PAN_ENDSTOP_SEARCH;
				}
			break;
			case INIT_STATE__TILT_PAN_ENDSTOP_SEARCH:
				if(gpio_get_level(MOTOR_TILT_ENDSTOP_PIN_NUM)){
					if(motor_tilt_steps_remain == 0){
						// TBD, endstop switch not reached.
					}								
				}				
				if(gpio_get_level(MOTOR_PAN_ENDSTOP_PIN_NUM)){
					if(motor_pan_steps_remain == 0){
						// TBD, endstop switch not reached.
					}								
				}	
				if(!gpio_get_level(MOTOR_TILT_ENDSTOP_PIN_NUM) && !gpio_get_level(MOTOR_PAN_ENDSTOP_PIN_NUM)){
					PCF8574_output_status = MOTOR_TELESCOPIC_ARM_DIR__CCW | MOTOR_TILT_DIR__CCW | MOTOR_PAN_DIR__CCW;
					if((ret = pcf8574_set_output_pins(PCF8574_output_status)) != ESP_OK){
						ESP_LOGE(TAG, "PCF8574 set output fail");
						return ret;
					}
					motor_telescopic_arms_steps_remain = 0;
					motor_tilt_steps_remain = CONFIG_MOTOR_TILT_RESET_STEPS;
					motor_tilt_half_period = MOTOR_TILT_CONST_HALF_PERIOD;
					motor_tilt_timer = 0;
					motor_pan_steps_remain = CONFIG_MOTOR_PAN_RESET_STEPS;
					motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD;
					motor_pan_timer = 0;
					initialize_state = INIT_STATE__TILT_PAN_RESET;					
					ESP_LOGD(TAG, "To INIT_STATE__TILT_PAN_RESET");
				}
			break;
			case INIT_STATE__TILT_PAN_RESET:
				if((motor_tilt_steps_remain == 0) && (motor_pan_steps_remain == 0)){
					initialize_state = INIT_STATE__DONE;
					ESP_LOGD(TAG, "To INIT_STATE__DONE");
				}
			break;
			default:
			break;
		}
				
		vTaskDelay(pdMS_TO_TICKS(TASK_SCHEDULED_PERIOD));	
	}
	
	if(initialize_state == INIT_STATE__DONE){
		orientation_ctrl_event_queue = xQueueCreate(ORIENTATION_CTRL_EVENT_QUEUE_SIZE, sizeof(orientation_ctrl_event_t));
		if (orientation_ctrl_event_queue == NULL) {
			ESP_LOGE(TAG, "Create orientation_ctrl_event_queue fail");
			return ESP_FAIL;
		}			
		
		xTaskCreatePinnedToCore(orientation_ctrl_task, "orientation_ctrl_task", 4096, NULL, TASK_PRIORITY__ORIENTATION_CTRL, NULL, TASK_CPU_CORE__ORIENTATION_CTRL);
	}
	
	return ESP_OK;	
}

static void orientation_ctrl_task(void *pvParameter){
	orientation_ctrl_event_t oc_evt;	
	target_position_t* target_position_ptr = NULL;
	uint8_t PCF8574_output_status_next = 0;
		
	while(1){
		if((xQueueReceive(orientation_ctrl_event_queue, &oc_evt, 0)) == pdTRUE){
			switch (oc_evt.id){
				case SWITCH_TELESCOPIC_ARM:
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED){
ESP_LOGI(TAG, "Check C");						
						retract_telescopic_arms();
					}else{				
						extend_telescopic_arms();
					}
				break;
				case SET_PAN_TILT:
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED){
						if(oc_evt.info){
							target_position_ptr = (target_position_t*)oc_evt.info;
														
							PCF8574_output_status_next = PCF8574_output_status & 0x06;
							if(target_position_ptr->pan_angle_offset_x10 > 0){PCF8574_output_status_next |= MOTOR_PAN_DIR__CCW;}
							if(target_position_ptr->pan_angle_offset_x10 < 0){PCF8574_output_status_next &= ~MOTOR_PAN_DIR__CCW;}
							if(target_position_ptr->tilt_angle_offset_x10 > 0){PCF8574_output_status_next &= ~MOTOR_TILT_DIR__CCW;}
							if(target_position_ptr->tilt_angle_offset_x10 < 0){PCF8574_output_status_next |= MOTOR_TILT_DIR__CCW;}
							if((PCF8574_output_status_next) != (PCF8574_output_status & 0x06)){
								PCF8574_output_status = PCF8574_output_status_next;
								if(pcf8574_set_output_pins(PCF8574_output_status) != ESP_OK){
									ESP_LOGE(TAG, "PCF8574 set output fail");
									break;
								}
							} 

							motor_pan_steps_remain = abs(target_position_ptr->pan_angle_offset_x10 * CONFIG_MOTOR_PAN_RESET_STEPS / MOTOR_PAN_RESET_ANGLE_IN_DEGREE / 10);
							if(motor_pan_steps_remain < MOTOR_PAN_DEC_STEPS_COUNT){
								if(motor_pan_half_period < motor_pan_dec_steps_half_period_lookup[motor_pan_steps_remain]){
									motor_pan_half_period = motor_pan_dec_steps_half_period_lookup[motor_pan_steps_remain];
								}									
							}else{
								motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD;
							}
							motor_pan_timer = 0;							
							
							motor_tilt_steps_remain = abs(target_position_ptr->tilt_angle_offset_x10 * CONFIG_MOTOR_TILT_RESET_STEPS / MOTOR_TILT_RESET_ANGLE_IN_DEGREE / 10);
							if(motor_tilt_steps_remain < MOTOR_TILT_DEC_STEPS_COUNT){
								motor_tilt_half_period = motor_tilt_dec_steps_half_period_lookup[motor_tilt_steps_remain];
							}else{
								motor_tilt_half_period = MOTOR_TILT_CONST_HALF_PERIOD;
							}
							motor_tilt_timer = 0;							
							
							free(oc_evt.info);
						}							
					}else{
						extend_telescopic_arms();						
					}				
				break;
				case RESET_PAN_TILT_ALARM:
					orientation_ctrl_status |= (ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN | 
												ORI_CTRL_STATUS__PAN_RIGHT_ENDSTOP_ALARM_EN | 
												ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN | 
												ORI_CTRL_STATUS__TILT_DOWN_ENDSTOP_ALARM_EN);
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TIMEOUT_SINGING_EN){
					// If the turret is still singing due to wireless connection timeout, stop singing, reset orientation, say Hello.
						speaker_ctrl__stop_music();
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__TIMEOUT_SINGING_EN;
						
						if(!(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED)){
							extend_telescopic_arms();
ESP_LOGI(TAG, "Check D, orientation_ctrl_status = 0x%04X", orientation_ctrl_status);							
						}
						pan_tilt_to_reset_angle();
						orientation_ctrl_status |= ORI_CTRL_STATUS__EXTEND_TELESCOPE_ARMS_FROM_TIMEOUT;													
					}else if(orientation_ctrl_status & ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN){
					// If telescopic arms were retracted due to wireless connection timeout, externd arms and say Hello.
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN;
						extend_telescopic_arms();							
						orientation_ctrl_status |= ORI_CTRL_STATUS__EXTEND_TELESCOPE_ARMS_FROM_TIMEOUT;						
					}else{}													
				break;
				case ENABLE_TIMEOUT_SINGING:
					timeout_singing_init();
				break;
				case ENABLE_TIMEOUT_RETRACT:
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED){
ESP_LOGI(TAG, "Check B");						
						retract_telescopic_arms();
						orientation_ctrl_status |= ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN;					
					}
				break;
				case DUMP_RECV_COOR:
					ESP_LOGI(TAG, "motor_pan_steps_remain = %d, motor_tilt_steps_remain = %d", motor_pan_steps_remain, motor_tilt_steps_remain);
				break;
				default:
				break;
			}
		}			
		
		if(orientation_ctrl_status & ORI_CTRL_STATUS__PAN_TILT_TO_DEFAULT){	
			if((motor_pan_angle_in_steps == CONFIG_MOTOR_PAN_RESET_STEPS) && 
			   (motor_tilt_angle_in_steps == CONFIG_MOTOR_TILT_RESET_STEPS)){	
				if(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED){
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN){
						speaker_ctrl__play_music(SOUND_TRACK__GOODNIGHT);
						vTaskDelay(pdMS_TO_TICKS(2000));
					}				   				
					if(orientation_ctrl_status & ORI_CTRL_STATUS__RETRACT_REQ_PENDING){							
						PCF8574_output_status &= ~MOTOR_TELESCOPIC_ARM_DIR__CCW;
						if(pcf8574_set_output_pins(PCF8574_output_status) != ESP_OK){
							ESP_LOGE(TAG, "PCF8574 set output fail");						
						}else{
							motor_telescopic_arms_steps_remain = CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS;
							motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD;
							motor_telescopic_arms_timer = 0;
							speaker_ctrl__play_music(SOUND_TRACK__RETRACT);												
						}
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__RETRACT_REQ_PENDING;
					}										
				}
				orientation_ctrl_status &= ~ORI_CTRL_STATUS__PAN_TILT_TO_DEFAULT;
			}
		}

		if(orientation_ctrl_status & ORI_CTRL_STATUS__TIMEOUT_SINGING_EN){
			if(speaker_ctrl_status & SPEAKER_CTRL_STATUS__BUSY){
				pan_at_timeout_singing();
			}else{
				orientation_ctrl_status &= ~ORI_CTRL_STATUS__TIMEOUT_SINGING_EN;
ESP_LOGI(TAG, "Check A");
				retract_telescopic_arms();
				orientation_ctrl_status |= ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN;
			}
		}
		
		// If telescopic arms were retracted due to wireless connection timeout, 
		// play "Hello" track after the telescopic arms are extended when wireless connection resume.
		if(orientation_ctrl_status & ORI_CTRL_STATUS__EXTEND_TELESCOPE_ARMS_FROM_TIMEOUT){			
			if((motor_pan_angle_in_steps == CONFIG_MOTOR_PAN_RESET_STEPS) &&
			   (motor_tilt_angle_in_steps == CONFIG_MOTOR_TILT_RESET_STEPS) &&
			   (orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED)){
				if(!(speaker_ctrl_status & SPEAKER_CTRL_STATUS__BUSY)){
					speaker_ctrl__play_music(SOUND_TRACK__HELLO);
					orientation_ctrl_status &= ~ORI_CTRL_STATUS__EXTEND_TELESCOPE_ARMS_FROM_TIMEOUT;
				}
			}
		}
		
		vTaskDelay(pdMS_TO_TICKS(TASK_SCHEDULED_PERIOD));
	}
}

void orientation_ctrl_get_view_angle(int16_t* pan_angle_x10, int16_t* tilt_angle_x10){
	*pan_angle_x10 = ((motor_pan_angle_in_steps * ((float)MOTOR_PAN_RESET_ANGLE_IN_DEGREE / CONFIG_MOTOR_PAN_RESET_STEPS)) - MOTOR_PAN_RESET_ANGLE_IN_DEGREE) * 10; 
	*tilt_angle_x10 = -((motor_tilt_angle_in_steps * ((float)MOTOR_TILT_RESET_ANGLE_IN_DEGREE / CONFIG_MOTOR_TILT_RESET_STEPS)) - MOTOR_TILT_RESET_ANGLE_IN_DEGREE) * 10;	
}

#define GUNS_SEPERATE_DISTANCE		24	// in cm.
void orientation_ctrl_get_aim_angle(int16_t* pan_angle_x10, int16_t* tilt_angle_x10, uint8_t* grouping_diameter, uint16_t aim_distance){
	int8_t left_gun_horizontal_offset, right_gun_horizontal_offset;
	int8_t left_gun_vertical_offset, right_gun_vertical_offset;
	int16_t guns_center_pan_angle_x10, guns_center_tilt_angle_x10;
	float motor_tilt_angle;
	
	motor_tilt_angle = -((motor_tilt_angle_in_steps * ((float)MOTOR_TILT_RESET_ANGLE_IN_DEGREE / CONFIG_MOTOR_TILT_RESET_STEPS)) - MOTOR_TILT_RESET_ANGLE_IN_DEGREE);	
	
	left_gun_horizontal_offset = (-GUNS_SEPERATE_DISTANCE / 2) + (aim_distance * tan(atan((float)(CONFIG_LEFT_AEG_HORIZ_OFFSET - (-GUNS_SEPERATE_DISTANCE / 2)) / CONFIG_AIMING_SIGHT_CALIBRATION_DISTANCE)));
	right_gun_horizontal_offset = (GUNS_SEPERATE_DISTANCE / 2) + (aim_distance * tan(atan((float)(CONFIG_RIGHT_AEG_HORIZ_OFFSET - GUNS_SEPERATE_DISTANCE / 2) / CONFIG_AIMING_SIGHT_CALIBRATION_DISTANCE)));
	guns_center_pan_angle_x10 = atan(((float)(left_gun_horizontal_offset + right_gun_horizontal_offset) / 2) / aim_distance) * (180 / 3.1416) * 10;
	*pan_angle_x10 = guns_center_pan_angle_x10;
	
	left_gun_vertical_offset = aim_distance * tan((motor_tilt_angle * 3.1416 / 180) + atan((float)CONFIG_LEFT_AEG_VERT_OFFSET / CONFIG_AIMING_SIGHT_CALIBRATION_DISTANCE));	
	right_gun_vertical_offset = aim_distance * tan((motor_tilt_angle * 3.1416 / 180) + atan((float)CONFIG_RIGHT_AEG_VERT_OFFSET / CONFIG_AIMING_SIGHT_CALIBRATION_DISTANCE));
	guns_center_tilt_angle_x10 = atan(((float)(left_gun_vertical_offset + right_gun_vertical_offset) / 2) / aim_distance) * (180 / 3.1416) * 10;
	*tilt_angle_x10 = guns_center_tilt_angle_x10;
	
	*grouping_diameter = abs(right_gun_horizontal_offset - left_gun_horizontal_offset) > abs(right_gun_vertical_offset - left_gun_vertical_offset) ?
					     abs(right_gun_horizontal_offset - left_gun_horizontal_offset) : abs(right_gun_vertical_offset - left_gun_vertical_offset);
}

void stepper_motor_steps_ctrl_isr(void){
	if(motor_telescopic_arms_steps_remain > 0){
		if(++motor_telescopic_arms_timer == motor_telescopic_arms_half_period){		
			if((PCF8574_output_status & MOTOR_TELESCOPIC_ARM_DIR__CCW)){
				if(gpio_get_level(MOTOR_TELESCOPE_ENDSTOP_PIN_NUM)){
					if(motors_step_pin_level & MOTOR_TELESCOPE_STEP_PIN__HIGH){
						motors_step_pin_level &= ~MOTOR_TELESCOPE_STEP_PIN__HIGH;
					}else{
						motors_step_pin_level |= MOTOR_TELESCOPE_STEP_PIN__HIGH;
						motor_telescopic_arms_steps_remain--;
						motor_telescopic_arms_extended_steps++;
					}	
					gpio_set_level(MOTOR_TELESCOPE_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_TELESCOPE_STEP_PIN__HIGH) ? 1 : 0);				
				}else{
					ESP_LOGD(TAG, "motor_telescopic_arms_steps_remain = %d", motor_telescopic_arms_steps_remain);	
					orientation_ctrl_status |= ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED;
					motor_telescopic_arms_steps_remain = 0;				
					motor_telescopic_arms_extended_steps = CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS;
				}			
			}else{
				if(motor_telescopic_arms_extended_steps > 0){
					if(motors_step_pin_level & MOTOR_TELESCOPE_STEP_PIN__HIGH){
						motors_step_pin_level &= ~MOTOR_TELESCOPE_STEP_PIN__HIGH;
					}else{
						motors_step_pin_level |= MOTOR_TELESCOPE_STEP_PIN__HIGH;
						motor_telescopic_arms_steps_remain--;
						motor_telescopic_arms_extended_steps--;
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED;						
					}	
					gpio_set_level(MOTOR_TELESCOPE_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_TELESCOPE_STEP_PIN__HIGH) ? 1 : 0);				
				}else{
					motor_telescopic_arms_steps_remain = 0;
				}
			}
			motor_telescopic_arms_timer = 0;
		}		
	}
	
	if(motor_tilt_steps_remain > 0){	
		if(motor_tilt_timer++ == motor_tilt_half_period){	
			if(!(PCF8574_output_status & MOTOR_TILT_DIR__CCW)){	
				if(gpio_get_level(MOTOR_TILT_ENDSTOP_PIN_NUM)){	
					if(motor_tilt_angle_in_steps > -(CONFIG_MOTOR_TILT_RESET_STEPS / 5)){
						if(motors_step_pin_level & MOTOR_TILT_STEP_PIN__HIGH){
							motors_step_pin_level &= ~MOTOR_TILT_STEP_PIN__HIGH;
						}else{
							motors_step_pin_level |= MOTOR_TILT_STEP_PIN__HIGH;
							motor_tilt_angle_in_steps--;
							motor_tilt_steps_remain--;
							if(motor_tilt_steps_remain < MOTOR_TILT_DEC_STEPS_COUNT){
								if(motor_tilt_half_period < motor_tilt_dec_steps_half_period_lookup[motor_tilt_steps_remain]){
									motor_tilt_half_period = motor_tilt_dec_steps_half_period_lookup[motor_tilt_steps_remain];
								}							
							}						
							orientation_ctrl_status |= ORI_CTRL_STATUS__TILT_DOWN_ENDSTOP_ALARM_EN;
						}
						gpio_set_level(MOTOR_TILT_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_TILT_STEP_PIN__HIGH) ? 1 : 0);
					}else{
						if(orientation_ctrl_status & ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN){
							speaker_ctrl__play_music(SOUND_TRACK__ALARM);	
							orientation_ctrl_status &= ~ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN;
						}
						motor_tilt_steps_remain = 0;			
					}
				}else{
					ESP_LOGD(TAG, "motor_tilt_steps_remain = %d", motor_tilt_steps_remain);
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN){
						speaker_ctrl__play_music(SOUND_TRACK__ALARM);	
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN;
					}
					motor_tilt_angle_in_steps = 0;
					motor_tilt_steps_remain = 0;				
				}			
			}else{
				if(motor_tilt_angle_in_steps < (CONFIG_MOTOR_TILT_RESET_STEPS * 2)){
					if(motors_step_pin_level & MOTOR_TILT_STEP_PIN__HIGH){
						motors_step_pin_level &= ~MOTOR_TILT_STEP_PIN__HIGH;
					}else{
						motors_step_pin_level |= MOTOR_TILT_STEP_PIN__HIGH;
						motor_tilt_angle_in_steps++;
						motor_tilt_steps_remain--;
						if(motor_tilt_steps_remain < MOTOR_TILT_DEC_STEPS_COUNT){
							if(motor_tilt_half_period < motor_tilt_dec_steps_half_period_lookup[motor_tilt_steps_remain]){
								motor_tilt_half_period = motor_tilt_dec_steps_half_period_lookup[motor_tilt_steps_remain];
							}							
						}					
						orientation_ctrl_status |= ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN;
					}
					gpio_set_level(MOTOR_TILT_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_TILT_STEP_PIN__HIGH) ? 1 : 0);
				}else{
					if(orientation_ctrl_status & ORI_CTRL_STATUS__TILT_DOWN_ENDSTOP_ALARM_EN){
						speaker_ctrl__play_music(SOUND_TRACK__ALARM);	
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__TILT_DOWN_ENDSTOP_ALARM_EN;
					}				
					motor_tilt_steps_remain = 0;
				}			
			}
			motor_tilt_timer = 0;
		}		
	}
	
	if(motor_pan_steps_remain > 0){ 
		if(motor_pan_timer++ == motor_pan_half_period){
			if(!(PCF8574_output_status & MOTOR_PAN_DIR__CCW)){
				if(gpio_get_level(MOTOR_PAN_ENDSTOP_PIN_NUM)){
					if(motor_pan_angle_in_steps > -(CONFIG_MOTOR_PAN_RESET_STEPS / 10)){
						if(motors_step_pin_level & MOTOR_PAN_STEP_PIN__HIGH){
							motors_step_pin_level &= ~MOTOR_PAN_STEP_PIN__HIGH;
						}else{
							motors_step_pin_level |= MOTOR_PAN_STEP_PIN__HIGH;
							motor_pan_angle_in_steps--;
							motor_pan_steps_remain--;
							if(motor_pan_steps_remain < MOTOR_PAN_DEC_STEPS_COUNT){
								if(motor_pan_half_period < motor_pan_dec_steps_half_period_lookup[motor_pan_steps_remain]){
									motor_pan_half_period = motor_pan_dec_steps_half_period_lookup[motor_pan_steps_remain];
								}
							}
							orientation_ctrl_status |= ORI_CTRL_STATUS__PAN_RIGHT_ENDSTOP_ALARM_EN;
						}
						gpio_set_level(MOTOR_PAN_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_PAN_STEP_PIN__HIGH) ? 1 : 0);
					}else{
						if(orientation_ctrl_status & ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN){
							speaker_ctrl__play_music(SOUND_TRACK__ALARM);	
							orientation_ctrl_status &= ~ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN;
						}			
						motor_pan_steps_remain = 0;
					}
				}else{
					ESP_LOGD(TAG, "motor_pan_steps_remain = %d", motor_pan_steps_remain);	
					if(orientation_ctrl_status & ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN){
						speaker_ctrl__play_music(SOUND_TRACK__ALARM);	
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN;
					}					
					motor_pan_angle_in_steps = 0;
					motor_pan_steps_remain = 0;				
				}			
			}else{
				if(motor_pan_angle_in_steps < (CONFIG_MOTOR_PAN_RESET_STEPS * 2)){
					if(motors_step_pin_level & MOTOR_PAN_STEP_PIN__HIGH){
						motors_step_pin_level &= ~MOTOR_PAN_STEP_PIN__HIGH;
					}else{
						motors_step_pin_level |= MOTOR_PAN_STEP_PIN__HIGH;	
						motor_pan_angle_in_steps++;
						motor_pan_steps_remain--;
						if(motor_pan_steps_remain < MOTOR_PAN_DEC_STEPS_COUNT){
							if(motor_pan_half_period < motor_pan_dec_steps_half_period_lookup[motor_pan_steps_remain]){
								motor_pan_half_period = motor_pan_dec_steps_half_period_lookup[motor_pan_steps_remain];
							}
						}
						orientation_ctrl_status |= ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN;
					}
					gpio_set_level(MOTOR_PAN_STEP_PIN_NUM, (motors_step_pin_level & MOTOR_PAN_STEP_PIN__HIGH) ? 1 : 0);
				}else{
					if(orientation_ctrl_status & ORI_CTRL_STATUS__PAN_RIGHT_ENDSTOP_ALARM_EN){
						speaker_ctrl__play_music(SOUND_TRACK__ALARM);	
						orientation_ctrl_status &= ~ORI_CTRL_STATUS__PAN_RIGHT_ENDSTOP_ALARM_EN;	
					}				
					motor_pan_steps_remain = 0;
				}			
			}
			motor_pan_timer = 0;
		}		
	}
}

static esp_err_t pcf8574_set_output_pins(uint8_t pin_mask){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_FAIL;
	uint8_t data[2] = {pin_mask, pin_mask};
	
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( CONFIG_PCF8574_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C_Write Failed addr:0x%02x, pin_mask:0x%02x, ret:%d", CONFIG_PCF8574_SLAVE_ADDR, pin_mask, ret);
    }
    return ret;	
}

static esp_err_t extend_telescopic_arms(void){
	esp_err_t ret = ESP_FAIL;
	
	if(!(orientation_ctrl_status & ORI_CTRL_STATUS__TELESCOPIC_ARMS_EXTENDED)){
		PCF8574_output_status |= MOTOR_TELESCOPIC_ARM_DIR__CCW;
		if(pcf8574_set_output_pins(PCF8574_output_status) != ESP_OK){
			ESP_LOGE(TAG, "PCF8574 set output fail");
			return ESP_FAIL;
		}		
		motor_telescopic_arms_steps_remain = CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS + 5;
		motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD;
		motor_telescopic_arms_timer = 0;							
		speaker_ctrl__play_music(SOUND_TRACK__DEPLOY);											
	}

	return ESP_OK;
}

static esp_err_t retract_telescopic_arms(void){
	// Pan & Tilt the turret orientatioin to reset angle before retracting
	pan_tilt_to_reset_angle();
	orientation_ctrl_status |= ORI_CTRL_STATUS__RETRACT_REQ_PENDING;	
	return ESP_OK;
}

static esp_err_t pan_tilt_to_reset_angle(void){
	uint8_t PCF8574_output_status_next;
	esp_err_t ret = ESP_OK;
	 
	if(!(orientation_ctrl_status & ORI_CTRL_STATUS__PAN_TILT_TO_DEFAULT)){							
		PCF8574_output_status_next = ((motor_pan_angle_in_steps > CONFIG_MOTOR_PAN_RESET_STEPS) ? MOTOR_PAN_DIR__CW : MOTOR_PAN_DIR__CCW) |
									 ((motor_tilt_angle_in_steps > CONFIG_MOTOR_TILT_RESET_STEPS) ? MOTOR_TILT_DIR__CW : MOTOR_TILT_DIR__CCW) |
									 (PCF8574_output_status & MOTOR_TELESCOPIC_ARM_DIR__CCW);
		if(PCF8574_output_status != PCF8574_output_status_next){
			PCF8574_output_status = PCF8574_output_status_next;
			ret = pcf8574_set_output_pins(PCF8574_output_status);
			if( ret != ESP_OK){
				ESP_LOGE(TAG, "PCF8574 set output fail");
			}			
		}
		if(ret == ESP_OK){
			motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD;						
			motor_pan_steps_remain = abs(motor_pan_angle_in_steps - CONFIG_MOTOR_PAN_RESET_STEPS);
			motor_pan_timer = 0;
			motor_tilt_half_period = MOTOR_TILT_CONST_HALF_PERIOD;						
			motor_tilt_steps_remain = abs(motor_tilt_angle_in_steps - CONFIG_MOTOR_TILT_RESET_STEPS);	
			motor_tilt_timer = 0;							
			orientation_ctrl_status |= ORI_CTRL_STATUS__PAN_TILT_TO_DEFAULT;			
		}			
	}else{
		ret = ESP_OK;
	}
	return ret;
}

#define TIMEOUT_SINGING_STATE__ARMS_STANDBY			0
#define TIMEOUT_SINGING_STATE__ARMS_RETRACT			1
#define TIMEOUT_SINGING_STATE__ARMS_EXTEND			2
#define TIMEOUT_SINGING_STATE__ARMS_FINAL_EXTEND	3
uint8_t timeout_singing_state = TIMEOUT_SINGING_STATE__ARMS_STANDBY;
#define TIMEOUT_SINGING__ARMS_RETRACT_DELAY		1350
#define TIMEOUT_SINGING__ARMS_RETRACT_PERIOD 	1750
#define TIMEOUT_SINGING__ARMS_EXTEND_PERIOD 	600
uint16_t timeout_singing_timer = 0;
#define TIMEOUT_SINGING__TOTAL_RETRACT_COUNT	20
uint8_t timeout_singing_retract_count = 0;
static esp_err_t timeout_singing_init(void){
	uint8_t PCF8574_output_status_next;
	esp_err_t ret = ESP_OK;
	
	ret = speaker_ctrl__play_music(SOUND_TRACK__OPERA_SINGING);
	if(ret == ESP_OK){
		PCF8574_output_status_next = ((motor_pan_angle_in_steps > CONFIG_MOTOR_PAN_RESET_STEPS) ? MOTOR_PAN_DIR__CW : MOTOR_PAN_DIR__CCW) |
									 ((motor_tilt_angle_in_steps > CONFIG_MOTOR_TILT_RESET_STEPS) ? MOTOR_TILT_DIR__CW : MOTOR_TILT_DIR__CCW) |
									 (PCF8574_output_status & MOTOR_TELESCOPIC_ARM_DIR__CCW);
		if(PCF8574_output_status != PCF8574_output_status_next){
			PCF8574_output_status = PCF8574_output_status_next;
			ret = pcf8574_set_output_pins(PCF8574_output_status);
			if( ret != ESP_OK){
				ESP_LOGE(TAG, "PCF8574 set output fail");
			}			
		}
		if(ret == ESP_OK){
			motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD * 4;						
			motor_pan_steps_remain = motor_pan_angle_in_steps > CONFIG_MOTOR_PAN_RESET_STEPS ? 
									 motor_pan_angle_in_steps - CONFIG_MOTOR_PAN_RESET_STEPS / 2 : 
									 CONFIG_MOTOR_PAN_RESET_STEPS * 3 / 2 - motor_pan_angle_in_steps;
			motor_pan_timer = 0;
			motor_tilt_half_period = MOTOR_TILT_CONST_HALF_PERIOD * 2;						
			motor_tilt_steps_remain = abs(motor_tilt_angle_in_steps - CONFIG_MOTOR_TILT_RESET_STEPS);
			motor_tilt_timer = 0;										
		}
		
		timeout_singing_state = TIMEOUT_SINGING_STATE__ARMS_STANDBY;
		timeout_singing_timer = 0;
		timeout_singing_retract_count = 0;
		orientation_ctrl_status |= ORI_CTRL_STATUS__TIMEOUT_SINGING_EN;	
	}	
		
	return ret;
}

static esp_err_t pan_at_timeout_singing(void){
	esp_err_t ret = ESP_OK;
	uint8_t PCF8574_output_status_next = PCF8574_output_status;
	
	if(!motor_pan_steps_remain){
		PCF8574_output_status_next = motor_pan_angle_in_steps > CONFIG_MOTOR_PAN_RESET_STEPS ?
									 PCF8574_output_status_next & ~MOTOR_PAN_DIR__CCW :
									 PCF8574_output_status_next | MOTOR_PAN_DIR__CCW;		
		motor_pan_steps_remain = motor_pan_angle_in_steps > CONFIG_MOTOR_PAN_RESET_STEPS ? 
								 motor_pan_angle_in_steps - CONFIG_MOTOR_PAN_RESET_STEPS / 2 : 
								 CONFIG_MOTOR_PAN_RESET_STEPS * 3 / 2 - motor_pan_angle_in_steps;
		motor_pan_half_period = MOTOR_PAN_CONST_HALF_PERIOD * 4;						 
	}
	
	timeout_singing_timer += TASK_SCHEDULED_PERIOD;
	switch(timeout_singing_state){
		case TIMEOUT_SINGING_STATE__ARMS_STANDBY:
			if(timeout_singing_timer >= TIMEOUT_SINGING__ARMS_RETRACT_DELAY){
				timeout_singing_timer = 0;
				PCF8574_output_status_next &= ~MOTOR_TELESCOPIC_ARM_DIR__CCW;
				motor_telescopic_arms_steps_remain = CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS;
				motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD * 5;
				motor_telescopic_arms_timer = 0;				
				timeout_singing_state = TIMEOUT_SINGING_STATE__ARMS_RETRACT;				
			}
		break;
		case TIMEOUT_SINGING_STATE__ARMS_RETRACT:
			if(timeout_singing_timer >= TIMEOUT_SINGING__ARMS_RETRACT_PERIOD){
				timeout_singing_timer = 0;
				PCF8574_output_status_next |= MOTOR_TELESCOPIC_ARM_DIR__CCW;
				motor_telescopic_arms_steps_remain = CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS;				
				motor_telescopic_arms_timer = 0;				
				if(++timeout_singing_retract_count < TIMEOUT_SINGING__TOTAL_RETRACT_COUNT){
					motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD;
					timeout_singing_state = TIMEOUT_SINGING_STATE__ARMS_EXTEND;									
				}else{
					motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD * 5;
					timeout_singing_state = TIMEOUT_SINGING_STATE__ARMS_FINAL_EXTEND;
				}
			}
		break;
		case TIMEOUT_SINGING_STATE__ARMS_EXTEND:
			if(timeout_singing_timer >= TIMEOUT_SINGING__ARMS_EXTEND_PERIOD){
				timeout_singing_timer = 0;
				PCF8574_output_status_next &= ~MOTOR_TELESCOPIC_ARM_DIR__CCW;
				motor_telescopic_arms_steps_remain = CONFIG_MOTOR_TELESCOPIC_ARMS_RETRACT_STEPS;
				motor_telescopic_arms_half_period = MOTOR_TELESCOPIC_ARMS_CONST_HALF_PERIOD * 5;
				motor_telescopic_arms_timer = 0;				
				timeout_singing_state = TIMEOUT_SINGING_STATE__ARMS_RETRACT;				
			}
		break;		
		default:
		break;
	}
	
	if(PCF8574_output_status_next != PCF8574_output_status){
		PCF8574_output_status = PCF8574_output_status_next;
		ret = pcf8574_set_output_pins(PCF8574_output_status_next);
		if( ret != ESP_OK){
			ESP_LOGE(TAG, "PCF8574 set output fail");
		}		
	}		
	
	return ret;
}
