#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "remote_ctrl_task.h"
#include "lcd_task.h"
#include "touch_utils.h"
#include "user_interface_utils.h"
#include "main.h"
#include "imgs.h"
#include "esp_log.h"

static const char *TAG = "remote_ctrl";

#define REMOTE_CTRL_CONNECT_TIMEOUT		10000
#define REMOTE_CTRL_RETRY_INTERVAL		1000

TaskHandle_t remote_ctrl_task_handle;

#define REMOTE_CTRL_QUEUE_OP_TIMEOUT	1000
QueueHandle_t remote_ctrl_event_queue;

#define RECV_BUF_COUNT	3

uint8_t remote_ctrl_state = REMOTE_CTRL_STATE__LISTENING; 
char* remote_ctrl_state_str[REMOTE_CTRL_STATE__MAX] = {"LISTENING",
														"PAIRING",
														"DATA_WAITING",
														"DATA_RECEIVING",
														"IDLE"};
														
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
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    remote_ctrl_event_t evt;
    remote_ctrl_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "ESPNOW sending callback with null mac_addr");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(remote_ctrl_event_queue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Push ESPNOW sending callback to remote_ctrl_event_queue fail");
    }
}

/* ESPNOW receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    remote_ctrl_event_t evt;
    remote_ctrl_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "ESPNOW receiving callback with null mac_addr");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc ESPNOW received data buffer fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(remote_ctrl_event_queue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Push ESPNOW receiving callback to remote_ctrl_event_queue fail");
        free(recv_cb->data);
    }
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(remote_ctrl_send_param_t *send_param, uint8_t payload_type, uint8_t payload_length, uint8_t* payload_buffer)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
	uint8_t i = 0;

	buf->type = payload_type;	    
	if(buf->type == ESPNOW_DATA_TYPE__C_COMMAND){
		for(i = 0; i < payload_length; i++){
			buf->payload[i] = payload_buffer[i];
		}
		buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf->payload, payload_length);
		send_param->len = ESPNOW_PAYLOAD_HEADER_LENGTH + payload_length;
	}else{
		buf->crc = 0;
		send_param->len = ESPNOW_PAYLOAD_HEADER_LENGTH;		
	}		
}
									 
static void remote_ctrl_task(void *pvParameter)
{
    remote_ctrl_send_param_t *send_param = (remote_ctrl_send_param_t *)pvParameter;
	remote_ctrl_event_t rc_recv_evt;
	espnow_data_t* espnow_data_ptr;	
	int recv_espnow_data_type;	
	uint8_t recv_espnow_data_index = 0;
	uint8_t connection_timeout_count = 0;
		
	uint16_t crc_cal;	
	lcd_event_t lcd_evt;
	
	remote_ctrl__turret_cmd_t send_tc;
	remote_ctrl__turret_data_t* recv_td;	
	uint8_t recv_td_index__latest = 0;
	uint8_t recv_td_index__next = 0;
	uint8_t recv_td_index__flushing = 0;
	uint8_t i = 0;
	
	#define TASK_STATUS__1ST_FRAME_IS_SENT	0x01
	#define TASK_STATUS__FRAME_REQ_PENDING	0x02
	uint8_t remote_ctrl_task_status = 0;
	
	recv_td = malloc(sizeof(remote_ctrl__turret_data_t) * (RECV_BUF_COUNT));
	if (recv_td == NULL) {
		ESP_LOGE(TAG, "Malloc turret data buffer fail");
		remote_ctrl_deinit(send_param);
		vTaskDelete(NULL);
	}else{
		for(i = 0; i < RECV_BUF_COUNT; i++){
			memset(&recv_td[i], 0, sizeof(remote_ctrl__turret_data_t));
		}		
	}
	
	memcpy(lcd_frame_bitmap, img_turret_searching, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
	lcd_evt.id = FLUSH_WITH_BITMAP;
	if(xQueueSend(lcd_event_queue, &lcd_evt, 0) != pdTRUE){
		ESP_LOGE(TAG, "Push FLUSH_WITH_BITMAP to lcd_event_queue fail");
	}	
		
    ESP_LOGI(TAG, "Start listening broadcast");
    
	while(1){	
		if (xQueueReceive(remote_ctrl_event_queue, &rc_recv_evt, pdMS_TO_TICKS(REMOTE_CTRL_QUEUE_OP_TIMEOUT)) == pdTRUE) {
			switch (rc_recv_evt.id) {
				case ESPNOW_SEND_CB:
					remote_ctrl_event_send_cb_t *send_cb = &rc_recv_evt.info.send_cb;
					espnow_data_ptr = (espnow_data_t*)(send_param->buffer);
					ESP_LOGD(TAG, "Send %s to "MACSTR", status: %d", 
							 espnow_data_type_str[espnow_data_ptr->type], 
							 MAC2STR(send_cb->mac_addr), 
							 send_cb->status);
					break;
				case ESPNOW_RECV_CB:
					remote_ctrl_event_recv_cb_t *recv_cb = &rc_recv_evt.info.recv_cb;
					espnow_data_ptr = (espnow_data_t*)(recv_cb->data);												
					recv_espnow_data_type = espnow_data_ptr->type;
					if(recv_espnow_data_type != ESPNOW_DATA_TYPE__T_DATA){
						ESP_LOGD(TAG, "When %s, receive %s from: "MACSTR", len: %d", 
								 remote_ctrl_state_str[remote_ctrl_state], 
								 espnow_data_type_str[recv_espnow_data_type], 
								 MAC2STR(recv_cb->mac_addr), recv_cb->data_len);							
					}																			
					switch(remote_ctrl_state){
						case REMOTE_CTRL_STATE__LISTENING:	
							if (recv_espnow_data_type == ESPNOW_DATA_TYPE__T_BROADCAST) {	
								
								/* If MAC address does not exist in peer list, add it to peer list. */
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
								
								// Signal the turret the broadcasting is received. 
								espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__C_ECHO, 0, NULL);								
								if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
									ESP_LOGE(TAG, "Send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__C_ECHO]);
									remote_ctrl_deinit(send_param);
									vTaskDelete(NULL);
								}
								remote_ctrl_state = REMOTE_CTRL_STATE__PAIRING;
							}					
							break;
						case REMOTE_CTRL_STATE__PAIRING:
							if (recv_espnow_data_type == ESPNOW_DATA_TYPE__T_PAIRING){																
								if (esp_now_is_peer_exist(recv_cb->mac_addr) == true){		
								
									// Fill turret control command content							
									ui__update_turret_command(&send_tc, &(recv_td[recv_td_index__latest]));
									
									espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__C_COMMAND, sizeof(remote_ctrl__turret_cmd_t), (uint8_t*)&send_tc);
									memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);									
									if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
										ESP_LOGE(TAG, "Send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__C_COMMAND]);
										remote_ctrl_deinit(send_param);
										vTaskDelete(NULL);
									}
									remote_ctrl_state = REMOTE_CTRL_STATE__DATA_WAITING;									
								}
							}
							break;
						case REMOTE_CTRL_STATE__DATA_WAITING:
							if(recv_espnow_data_type == ESPNOW_DATA_TYPE__T_DATA_START){
							/* Camera frame length and turret status will be received at REMOTE_CTRL_STATE__DATA_WAITING stage, 
							   then camera frame context will be received at next stage (REMOTE_CTRL_STATE__DATA_RECEIVING). */
								if (esp_now_is_peer_exist(recv_cb->mac_addr) == true){																	
									crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)(espnow_data_ptr->payload), recv_cb->data_len - ESPNOW_PAYLOAD_HEADER_LENGTH );
									if (crc_cal != espnow_data_ptr->crc) {
										ESP_LOGW(TAG, "ESPNOW received data CRC error");
									}else{
										connection_timeout_count = 0;										
										recv_espnow_data_index = 0;	
										
										memcpy(&(recv_td[recv_td_index__next].frame_len), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(size_t));	
										recv_espnow_data_index += sizeof(size_t);
										recv_td[recv_td_index__next].recv_frame_len = 0;
										recv_td[recv_td_index__next].frame_buf = malloc(recv_td[recv_td_index__next].frame_len);
										if(recv_td[recv_td_index__next].frame_buf == NULL){												
											ESP_LOGW(TAG, "Malloc %d bytes of frame buffer fail.", recv_td[recv_td_index__next].frame_len);
											recv_td[recv_td_index__next].frame_len = 0;
										}else{												
											remote_ctrl_state = REMOTE_CTRL_STATE__DATA_RECEIVING;											
										}		
		
										memcpy(&(recv_td[recv_td_index__next].turret_status.view_pan_angle_x10), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(int16_t));											
										recv_espnow_data_index += sizeof(int16_t);
										memcpy(&(recv_td[recv_td_index__next].turret_status.view_tilt_angle_x10), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(int16_t));											
										recv_espnow_data_index += sizeof(int16_t);		
										memcpy(&(recv_td[recv_td_index__next].turret_status.aim_pan_angle_x10), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(int16_t));											
										recv_espnow_data_index += sizeof(int16_t);
										memcpy(&(recv_td[recv_td_index__next].turret_status.aim_tilt_angle_x10), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(int16_t));											
										recv_espnow_data_index += sizeof(int16_t);
										memcpy(&(recv_td[recv_td_index__next].turret_status.aim_grouping_diameter), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(uint8_t));											
										recv_espnow_data_index += sizeof(uint8_t);										
										memcpy(&(recv_td[recv_td_index__next].turret_status.aim_distance), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(uint16_t));											
										recv_espnow_data_index += sizeof(uint16_t);
										memcpy(&(recv_td[recv_td_index__next].turret_status.fire_ctrl_status), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(uint8_t));											
										recv_espnow_data_index += sizeof(uint8_t);
										memcpy(&(recv_td[recv_td_index__next].turret_status.orientation_ctrl_status), &(espnow_data_ptr->payload[recv_espnow_data_index]), sizeof(uint8_t));											
										recv_espnow_data_index += sizeof(uint8_t);
									}																
								}								
							}
							break;							
						case REMOTE_CTRL_STATE__DATA_RECEIVING:
							if(recv_espnow_data_type == ESPNOW_DATA_TYPE__T_DATA){																														
								if (esp_now_is_peer_exist(recv_cb->mac_addr) == true){		
									if(recv_td[recv_td_index__next].recv_frame_len + (recv_cb->data_len - ESPNOW_PAYLOAD_HEADER_LENGTH) > recv_td[recv_td_index__next].frame_len){
										ESP_LOGW(TAG, "Invalid received frame length");																				
										// Flush all data already received.
										free(recv_td[recv_td_index__next].frame_buf);
										memset(&recv_td[recv_td_index__next], 0, sizeof(remote_ctrl__turret_data_t));		
										// Re-send C_COMMAND
										if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
											ESP_LOGE(TAG, "Send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__C_COMMAND]);
											remote_ctrl_deinit(send_param);
											vTaskDelete(NULL);
										}				
										remote_ctrl_state = REMOTE_CTRL_STATE__DATA_WAITING;
									}else{
										memcpy(&(recv_td[recv_td_index__next].frame_buf[recv_td[recv_td_index__next].recv_frame_len]), 
											   espnow_data_ptr->payload, 
											   recv_cb->data_len - ESPNOW_PAYLOAD_HEADER_LENGTH);
										recv_td[recv_td_index__next].recv_frame_len += (recv_cb->data_len - ESPNOW_PAYLOAD_HEADER_LENGTH);								
									} 
										
									if(recv_td[recv_td_index__next].recv_frame_len == recv_td[recv_td_index__next].frame_len){	
									// All turret data is received
										recv_td_index__latest = recv_td_index__next;
										for(i = 0; i < RECV_BUF_COUNT; i++){
											if((i != recv_td_index__latest) && (i != recv_td_index__flushing)){
												recv_td_index__next = i;
												free(recv_td[recv_td_index__next].frame_buf);
												memset(&recv_td[recv_td_index__next], 0, sizeof(remote_ctrl__turret_data_t));
												break;
											}
										}
										if(i == RECV_BUF_COUNT){
											ESP_LOGE(TAG, "No free recv_td to allocate");
											remote_ctrl_deinit(send_param);
											vTaskDelete(NULL);																						
										}
										
										if(!(remote_ctrl_task_status & TASK_STATUS__1ST_FRAME_IS_SENT) || 
										    (remote_ctrl_task_status & TASK_STATUS__FRAME_REQ_PENDING)){
											lcd_evt.id = FLUSH_WITH_TURRET_DATA;
											lcd_evt.info = &(recv_td[recv_td_index__latest]);
											if (xQueueSend(lcd_event_queue, &lcd_evt, 0) != pdTRUE) {
												ESP_LOGW(TAG, "Push FLUSH_WITH_TURRET_DATA to lcd_event_queue fail");
											}else{											
												recv_td_index__flushing = recv_td_index__latest;
												remote_ctrl_task_status |= TASK_STATUS__1ST_FRAME_IS_SENT;
												remote_ctrl_task_status &= ~TASK_STATUS__FRAME_REQ_PENDING;
											}										
										}	
										
										// Fill the next turret controll command content
										ui__update_turret_command(&send_tc, &(recv_td[recv_td_index__latest]));
										
										espnow_data_prepare(send_param, ESPNOW_DATA_TYPE__C_COMMAND, sizeof(remote_ctrl__turret_cmd_t), (uint8_t*)&send_tc);
										if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
											ESP_LOGE(TAG, "Send %s error", espnow_data_type_str[ESPNOW_DATA_TYPE__C_COMMAND]);
											remote_ctrl_deinit(send_param);
											vTaskDelete(NULL);
										}
										remote_ctrl_state = REMOTE_CTRL_STATE__DATA_WAITING;
									}																																			
								}								
							}
							break;
						default:
							break;
					}									
					free(recv_cb->data);
					break;				
				case LCD_FRAME_REQ:		
					if(recv_td_index__flushing != recv_td_index__latest){
						lcd_evt.id = FLUSH_WITH_TURRET_DATA;
						lcd_evt.info = &(recv_td[recv_td_index__latest]);						
						if (xQueueSend(lcd_event_queue, &lcd_evt, 0) != pdTRUE) {
							ESP_LOGW(TAG, "Push FLUSH_WITH_TURRET_DATA to lcd_event_queue fail");
						}else{
							recv_td_index__flushing = recv_td_index__latest;
						}
					}else{
						ESP_LOGW(TAG, "NO ready recv_td to display.");
						remote_ctrl_task_status |= TASK_STATUS__FRAME_REQ_PENDING;
					}
					break;
				default:
					ESP_LOGE(TAG, "Callback type error: %d", rc_recv_evt.id);
					break;
			}			
		}else{
			ESP_LOGW(TAG, "When %s, QReceive timeout %d", remote_ctrl_state_str[remote_ctrl_state], connection_timeout_count);
			if(++connection_timeout_count >= (REMOTE_CTRL_CONNECT_TIMEOUT / REMOTE_CTRL_QUEUE_OP_TIMEOUT)){
				connection_timeout_count = 0;							
				for(i = 0; i < RECV_BUF_COUNT; i++){
					if(recv_td[i].frame_buf != NULL){
						free(recv_td[i].frame_buf);
					}
					memset(&recv_td[i], 0, sizeof(remote_ctrl__turret_data_t));
				}
				recv_td_index__flushing = 0;
				recv_td_index__latest = 0;
				recv_td_index__next = 0;
				memcpy(send_param->dest_mac, s_remote_ctrl_broadcast_mac, ESP_NOW_ETH_ALEN);

				memcpy(lcd_frame_bitmap, img_turret_searching, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
				lcd_evt.id = FLUSH_WITH_BITMAP;
				if(xQueueSend(lcd_event_queue, &lcd_evt, 0) != pdTRUE){
					ESP_LOGE(TAG, "Push FLUSH_WITH_BITMAP to lcd_event_queue fail");
				}				
				
				remote_ctrl_state = REMOTE_CTRL_STATE__LISTENING;    
			}else{
				switch(remote_ctrl_state){
					case REMOTE_CTRL_STATE__PAIRING:
						if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
							ESP_LOGE(TAG, "Re-send C_ECHO error");
							remote_ctrl_deinit(send_param);
							vTaskDelete(NULL);
						}
						break;
					case REMOTE_CTRL_STATE__DATA_WAITING:
					case REMOTE_CTRL_STATE__DATA_RECEIVING:
						// Flush all turret data already received.
						free(recv_td[recv_td_index__next].frame_buf);
						memset(&recv_td[recv_td_index__next], 0, sizeof(remote_ctrl__turret_data_t));		
						
						if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
							ESP_LOGE(TAG, "Re-send C_COMMAND error");
							remote_ctrl_deinit(send_param);
							vTaskDelete(NULL);
						}
						remote_ctrl_state = REMOTE_CTRL_STATE__DATA_WAITING;
						break;
					default:
						break;
				}				
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
        ESP_LOGE(TAG, "Create remote_ctrl_event_queue fail");
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
    memcpy(send_param->dest_mac, s_remote_ctrl_broadcast_mac, ESP_NOW_ETH_ALEN);
    
	xTaskCreatePinnedToCore(remote_ctrl_task, "remote_ctrl_task", 4096, send_param, TASK_PRIORITY__REMOTE_CTRL, &remote_ctrl_task_handle, TASK_CPU_CORE__REMOTE_CTRL);

    return ESP_OK;
}

