#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "../../Common/common.h"
#include "esp_camera.h"
#include "remote_ctrl_task.h"
#include "orientation_ctrl_task.h"
#include "fire_ctrl_task.h"
#include "range_finder_utils.h"
#include "speaker_ctrl_task.h"

static const char *TAG = "remote_ctrl";

#define REMOTE_CTRL_CONNECT_TIMEOUT		5000
#define REMOTE_CTRL_QUEUE_OP_TIMEOUT	1000
QueueHandle_t remote_ctrl_event_queue;

enum{
	REMOTE_CTRL_STATE__BROADCAST,
	REMOTE_CTRL_STATE__PAIRING,
	REMOTE_CTRL_STATE__CMD_WAITING,
	REMOTE_CTRL_STATE__CMD_HANDLING,
	REMOTE_CTRL_STATE__MAX
};
char* remote_ctrl_state_str[REMOTE_CTRL_STATE__MAX] = {"BROADCAST",
													   "PAIRING",
													   "CMD_WAITING",
													   "CMD_HANDLING"};

enum {
	ESPNOW_DATA_TYPE__T_BROADCAST,
	ESPNOW_DATA_TYPE__C_ECHO,
	ESPNOW_DATA_TYPE__T_PAIRING,
	ESPNOW_DATA_TYPE__C_COMMAND,
	ESPNOW_DATA_TYPE__T_DATA_START,
	ESPNOW_DATA_TYPE__T_DATA,
	ESPNOW_DATA_TYPE__C_ACK,
	ESPNOW_DATA_TYPE__MAX
};
char* espnow_data_type_str[ESPNOW_DATA_TYPE__MAX] = {"T_BROADCAST",
													 "C_ECHO",
													 "T_PAIRING",
													 "C_COMMAND",
													 "T_DATA_START",													 
													 "T_DATA",
													 "C_ACK"};
													 					 
static uint8_t s_remote_ctrl_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void remote_ctrl_deinit(remote_ctrl_send_param_t *send_param);

/* ESPNOW sending callback function is called in WiFi task.
   Users should not do lengthy operations from this task. Instead, post
   necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    remote_ctrl_event_t rc_evt;
    remote_ctrl_event_send_cb_t *send_cb = &rc_evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "ESPNOW sending callback with null mac_addr");
        return;
    }

    rc_evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(remote_ctrl_event_queue, &rc_evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Push ESPNOW sending callback to remote_ctrl_event_queue fail");
    }
}

/* ESPNOW receiving callback function is called in WiFi task.
   Users should not do lengthy operations from this task. Instead, post
   necessary data to a queue and handle it from a lower priority task. */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    remote_ctrl_event_t rc_evt;
    remote_ctrl_event_recv_cb_t *recv_cb = &rc_evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "ESPNOW receiving callback with null mac_addr");
        return;
    }

    rc_evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc ESPNOW received data buffer fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(remote_ctrl_event_queue, &rc_evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Push ESPNOW receiving callback to remote_ctrl_event_queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

	crc = buf->crc;
	if(buf->type == ESPNOW_DATA_TYPE__C_COMMAND){		
		if(data_len > ESPNOW_PAYLOAD_HEADER_LENGTH){
			crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf->payload, data_len - ESPNOW_PAYLOAD_HEADER_LENGTH);
			if (crc_cal != crc) {
				ESP_LOGW(TAG, "Received ESPNOW data CRC error");
				return -1;
			}			
		}
	}
	return buf->type;   
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(remote_ctrl_send_param_t *send_param, uint8_t payload_type, uint8_t payload_length, uint8_t* payload_buffer)
{	
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

	buf->type = payload_type;
	switch(buf->type){
		case ESPNOW_DATA_TYPE__T_DATA_START:
			memcpy(buf->payload, payload_buffer, payload_length);
			buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf->payload, payload_length);
			send_param->len = ESPNOW_PAYLOAD_HEADER_LENGTH + payload_length;			
		break;
		case ESPNOW_DATA_TYPE__T_DATA:		
			memcpy(buf->payload, payload_buffer, payload_length);
			buf->crc = 0;
			send_param->len = ESPNOW_PAYLOAD_HEADER_LENGTH + payload_length;		
		break;
		default:
			buf->crc = 0;
			send_param->len = ESPNOW_PAYLOAD_HEADER_LENGTH;
		break;		
	}
}

static void remote_ctrl_task(void *pvParameter)
{	
	remote_ctrl_send_param_t *send_param = (remote_ctrl_send_param_t *)pvParameter;

    remote_ctrl_event_t rc_evt;	
    espnow_data_t* espnow_data_ptr;
	int recv_espnow_data_type;
	
	remote_ctrl__turret_cmd_t* recv_tc;
	orientation_ctrl_event_t oc_evt;
	target_position_t* target_position_ptr;
	fire_ctrl_event_t fc_evt;
		
	uint16_t frame_ctx_has_been_sent = 0;
	camera_fb_t* camera_frame_buffer = 0;
	remote_ctrl__turret_status_t turret_status_buffer;
	uint8_t send_payload_buffer[16];
	uint8_t send_payload_length = 0;
	
	uint8_t recv_cmd_count = 0;
	uint8_t connection_timeout_count = 0;
	
	uint8_t remote_ctrl_state = REMOTE_CTRL_STATE__BROADCAST;
	
    /* Start sending broadcast ESPNOW data. */
    ESP_LOGI(TAG, "Start broadcasting");    
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "ESPNOW broadcast error");
        remote_ctrl_deinit(send_param);
        vTaskDelete(NULL);
    }
	
	while(1) {
		if (xQueueReceive(remote_ctrl_event_queue, &rc_evt, pdMS_TO_TICKS(REMOTE_CTRL_QUEUE_OP_TIMEOUT)) == pdTRUE) {			
			switch (rc_evt.id) {
				case ESPNOW_SEND_CB:
					remote_ctrl_event_send_cb_t *send_cb = &rc_evt.info.send_cb;
					espnow_data_ptr = (espnow_data_t*)(send_param->buffer);
					if(espnow_data_ptr->type != ESPNOW_DATA_TYPE__T_DATA){						
						ESP_LOGD(TAG, "Send %s to "MACSTR", status1: %d", 
								 espnow_data_type_str[espnow_data_ptr->type], 
								 MAC2STR(send_cb->mac_addr), 
								 send_cb->status);	
					}
					
					switch(remote_ctrl_state){						
						case REMOTE_CTRL_STATE__CMD_HANDLING:
							if(frame_ctx_has_been_sent < camera_frame_buffer->len){	
								if(camera_frame_buffer->len - frame_ctx_has_been_sent > ESPNOW_PAYLOAD_LENGTH){
									send_payload_length = ESPNOW_PAYLOAD_LENGTH;
								}else{
									send_payload_length = camera_frame_buffer->len - frame_ctx_has_been_sent;
								}
								
								espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__T_DATA, send_payload_length, &camera_frame_buffer->buf[frame_ctx_has_been_sent]);																							
								if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
									ESP_LOGE(TAG, "ESPNOW send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__T_DATA]);
									remote_ctrl_deinit(send_param);
									vTaskDelete(NULL);
								}	
								frame_ctx_has_been_sent += send_payload_length;
							}else{
								esp_camera_fb_return(camera_frame_buffer);
								remote_ctrl_state = REMOTE_CTRL_STATE__CMD_WAITING;								
							}
							break;							
						default:
							break;
					}			
					break;
					
				case ESPNOW_RECV_CB:				
					remote_ctrl_event_recv_cb_t *recv_cb = &rc_evt.info.recv_cb;					
					
					recv_espnow_data_type = espnow_data_parse(recv_cb->data, recv_cb->data_len);
					ESP_LOGD(TAG, "When %s, receive %s from: "MACSTR", len: %d", 
							 remote_ctrl_state_str[remote_ctrl_state], 
							 espnow_data_type_str[recv_espnow_data_type], 
							 MAC2STR(recv_cb->mac_addr), recv_cb->data_len);										
					
					switch(remote_ctrl_state){
						case REMOTE_CTRL_STATE__BROADCAST:		
							if (recv_espnow_data_type == ESPNOW_DATA_TYPE__C_ECHO) {
								
								// If MAC address does not exist in peer list, add it to peer list. 
								if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
									esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
									if (peer == NULL) {
										ESP_LOGE(TAG, "Malloc peer information fail");
										remote_ctrl_deinit(send_param);
										vTaskDelete(NULL);
									}
									memset(peer, 0, sizeof(esp_now_peer_info_t));
									peer->channel = CONFIG_ESPNOW_CHANNEL;
									peer->ifidx = ESPNOW_WIFI_IF;
									peer->encrypt = true;
									memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
									memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
									ESP_ERROR_CHECK( esp_now_add_peer(peer) );
									free(peer);
								}

								// Stop sending broadcast ESPNOW data and start sending pairing ESPNOW data to that controller.						
								espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__T_PAIRING, 0, NULL);
								memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);									
								if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
									ESP_LOGE(TAG, "Send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__T_PAIRING]);
									remote_ctrl_deinit(send_param);
									vTaskDelete(NULL);
								}
								remote_ctrl_state = REMOTE_CTRL_STATE__PAIRING;
							}
							break;	
						case REMOTE_CTRL_STATE__PAIRING:
						case REMOTE_CTRL_STATE__CMD_WAITING:
							if (recv_espnow_data_type == ESPNOW_DATA_TYPE__C_COMMAND) {															
								if (esp_now_is_peer_exist(recv_cb->mac_addr) == true){
									connection_timeout_count = 0;
									recv_cmd_count++;
								
									recv_tc = (remote_ctrl__turret_cmd_t*)(((espnow_data_t*)(recv_cb->data))->payload);
									// Decode received orientationi ctrl command									
									if(recv_tc->turret_ctrl & TURRET_CTRL_BITSMAP__TELESCOPIC_ARM_CTRL_BTN_IS_PRESSED){
										oc_evt.id = SWITCH_TELESCOPIC_ARM;
										oc_evt.info = NULL;
										if(xQueueSend(orientation_ctrl_event_queue, &oc_evt, 0) != pdTRUE){
											ESP_LOGW(TAG, "Push SET_TELESCOPIC_ARM to orientation_ctrl_event_queue fail");
										}
									}else if(recv_tc->turret_ctrl & TURRET_CTRL_BITSMAP__SET_PAN_TILT){
										target_position_ptr = malloc(sizeof(target_position_t));
										if(target_position_ptr == NULL){
											ESP_LOGE(TAG, "malloc target_position_t fail");
										}else{
											target_position_ptr->pan_angle_offset_x10 = recv_tc->pan_angle_offset_x10;
											target_position_ptr->tilt_angle_offset_x10 = recv_tc->tilt_angle_offset_x10;
											target_position_ptr->distance = turret_status_buffer.aim_distance;
											oc_evt.info = target_position_ptr;
											oc_evt.id = SET_PAN_TILT;
											if(xQueueSend(orientation_ctrl_event_queue, &oc_evt, 0) != pdTRUE){
												ESP_LOGW(TAG, "Push SET_PAN_TILT to orientation_ctrl_event_queue fail");
												free(target_position_ptr);
											}											
										}										
									}else{
										oc_evt.id = RESET_PAN_TILT_ALARM;
										oc_evt.info = NULL;
										if(xQueueSend(orientation_ctrl_event_queue, &oc_evt, 0) != pdTRUE){
											ESP_LOGW(TAG, "Push RESET_PAN_TILT_ALARM to orientation_ctrl_event_queue fail");
										}										
									}
									// Decode received fire ctrl command		
									if(recv_tc->turret_ctrl & TURRET_CTRL_BITSMAP__SAFETY_BTN_IS_PRESSED){
										fc_evt.id = SWITCH_SAFETY;
									}else{
										if(recv_tc->turret_ctrl & TURRET_CTRL_BITSMAP__AIM_ICON_IS_PRESSED){
											fc_evt.id = OPEN_FIRE;
										}else{
											fc_evt.id = CEASE_FIRE;
										}										
									}
									if(xQueueSend(fire_ctrl_event_queue, &fc_evt, 0) != pdTRUE){
										// This xQueueSend() will return FAIL when fire_ctrl_task calls vTaskDelay() to wait the finish of playing soundtrack.
										// ESP_LOGW(TAG, "Push event to fire_ctrl_event_queue fail");											
									}											
																			
									// Collect camera frame and turret status 
									camera_frame_buffer = esp_camera_fb_get();
									if(camera_frame_buffer){										
										frame_ctx_has_been_sent = 0;
										send_payload_length = 0;
										memcpy(&(send_payload_buffer[send_payload_length]), &(camera_frame_buffer->len), sizeof(size_t));					
										send_payload_length += sizeof(size_t);
										
										orientation_ctrl_get_view_angle(&(turret_status_buffer.view_pan_angle_x10), 
																		&(turret_status_buffer.view_tilt_angle_x10));
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.view_pan_angle_x10), sizeof(int16_t));	
										send_payload_length += sizeof(int16_t);																				
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.view_tilt_angle_x10), sizeof(int16_t));	
										send_payload_length += sizeof(int16_t);
										
										orientation_ctrl_get_aim_angle(&(turret_status_buffer.aim_pan_angle_x10), 
																	   &(turret_status_buffer.aim_tilt_angle_x10),
																	   &(turret_status_buffer.aim_grouping_diameter),
																	   turret_status_buffer.aim_distance);											
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.aim_pan_angle_x10), sizeof(int16_t));	
										send_payload_length += sizeof(int16_t);																				
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.aim_tilt_angle_x10), sizeof(int16_t));	
										send_payload_length += sizeof(int16_t);
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.aim_grouping_diameter), sizeof(uint8_t));	
										send_payload_length += sizeof(uint8_t);										
										
										if(recv_cmd_count % 2){
										// Ultrasnic range finder polling period must slower than 70ms
											range_finder__get_range(&(turret_status_buffer.aim_distance));
										}
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.aim_distance), sizeof(uint16_t));	
										send_payload_length += sizeof(uint16_t);
										
										turret_status_buffer.fire_ctrl_status = fire_ctrl_status;
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.fire_ctrl_status), sizeof(uint8_t));	
										send_payload_length += sizeof(uint8_t);
										
										turret_status_buffer.orientation_ctrl_status = orientation_ctrl_status;
										memcpy(&(send_payload_buffer[send_payload_length]), &(turret_status_buffer.orientation_ctrl_status), sizeof(uint8_t));	
										send_payload_length += sizeof(uint8_t);										
										
										espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__T_DATA_START, send_payload_length, send_payload_buffer);
										memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);										
										if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
											ESP_LOGE(TAG, "ESPNOW send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__T_DATA_START]);
											remote_ctrl_deinit(send_param);
											vTaskDelete(NULL);
										}				

										if(remote_ctrl_state == REMOTE_CTRL_STATE__PAIRING){
											if(!(orientation_ctrl_status & ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN) &&
											   !(orientation_ctrl_status & ORI_CTRL_STATUS__TIMEOUT_SINGING_EN)){
												speaker_ctrl__play_music(SOUND_TRACK__HELLO);
											}else{
												// Orientation ctrl task will play "Hello" track after extend the telescope arms. 
											}											
										}
										remote_ctrl_state = REMOTE_CTRL_STATE__CMD_HANDLING;																				
									}								
								}
							}				
						break;
						default:
						break;
					}					
					free(recv_cb->data);
					break;
				case DUMP_RECV_CMD_COUNT:
					ESP_LOGI(TAG, "recv_cmd_count = %d", recv_cmd_count);
					recv_cmd_count = 0;
					break;
				default:
					ESP_LOGE(TAG, "Undefined ESPNOW callback: %d", rc_evt.id);
					break;
			}			
		}else{
			ESP_LOGW(TAG, "When %s, QReceive timeout %d", remote_ctrl_state_str[remote_ctrl_state], connection_timeout_count);
			if(connection_timeout_count <= ORIENTATION_CTRL__TIMEOUT_TO_SING){connection_timeout_count++;}
			switch(remote_ctrl_state){
				case REMOTE_CTRL_STATE__BROADCAST:	
					if(connection_timeout_count == 3){
						speaker_ctrl__play_music(SOUND_TRACK__ANYONE_THERE);
					}else if(connection_timeout_count == ORIENTATION_CTRL__TIMEOUT_TO_SING){												
						oc_evt.id = ENABLE_TIMEOUT_SINGING;
						oc_evt.info = NULL;
						if(xQueueSend(orientation_ctrl_event_queue, &oc_evt, 0) != pdTRUE){
							ESP_LOGW(TAG, "Push ENABLE_TIMEOUT_SINGING to orientation_ctrl_event_queue fail");
						}									
					}else{
						// Resent ESPNOW data if no RECV_CB event
						if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
							ESP_LOGE(TAG, "Re-send T_BROADCAST error");
							remote_ctrl_deinit(send_param);
							vTaskDelete(NULL);
						}
					}
					break;
				case REMOTE_CTRL_STATE__PAIRING:
					// Resent ESPNOW data if no RECV_CB event
					if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
						ESP_LOGE(TAG, "Re-send T_PAIRING error");
						remote_ctrl_deinit(send_param);
						vTaskDelete(NULL);
					}
					break;
				default:
					if(connection_timeout_count == (REMOTE_CTRL_CONNECT_TIMEOUT / REMOTE_CTRL_QUEUE_OP_TIMEOUT)){				
						connection_timeout_count = 0;	
						
						espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__T_BROADCAST, 0, NULL);
						memcpy(send_param->dest_mac, s_remote_ctrl_broadcast_mac, ESP_NOW_ETH_ALEN);												
						if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
							ESP_LOGE(TAG, "ESPNOW broadcast error");
							remote_ctrl_deinit(send_param);
							vTaskDelete(NULL);
						}				
						remote_ctrl_state = REMOTE_CTRL_STATE__BROADCAST;  				
					}									
					break;
			}			
		}		
	}	
}

static void remote_ctrl_deinit(remote_ctrl_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(remote_ctrl_event_queue);
    esp_now_deinit();
}

esp_err_t remote_ctrl_init(void)
{
    remote_ctrl_send_param_t *send_param;

    remote_ctrl_event_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(remote_ctrl_event_t));
    if (remote_ctrl_event_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }
	
	/* WiFi should start before using ESPNOW */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
	
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
	ESP_ERROR_CHECK( esp_wifi_config_espnow_rate(ESPNOW_WIFI_IF, WIFI_PHY_RATE_24M) );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(remote_ctrl_event_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_remote_ctrl_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(remote_ctrl_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(remote_ctrl_event_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(remote_ctrl_send_param_t));	  
    send_param->buffer = malloc(sizeof(espnow_data_t));	
    if (send_param->buffer == NULL) {	
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(remote_ctrl_event_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }	
	
	espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__T_BROADCAST, 0, NULL);
	memcpy(send_param->dest_mac, s_remote_ctrl_broadcast_mac, ESP_NOW_ETH_ALEN);
   	
    xTaskCreate(remote_ctrl_task, "remote_ctrl_task", 4096, send_param, 4, NULL);

    return ESP_OK;
}
