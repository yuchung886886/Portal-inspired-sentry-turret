/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_now.h"
#include "../../Common/common.h"

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           12

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_remote_ctrl_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
	LCD_FRAME_REQ,
} remote_ctrl_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} remote_ctrl_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} remote_ctrl_event_recv_cb_t;

typedef union {
    remote_ctrl_event_send_cb_t send_cb;
    remote_ctrl_event_recv_cb_t recv_cb;
} remote_ctrl_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    remote_ctrl_event_id_t id;
    remote_ctrl_event_info_t info;
} remote_ctrl_event_t;

/* User defined field of ESPNOW data in this example. */
#define ESPNOW_PAYLOAD_HEADER_LENGTH	3 /*type, crc*/
#define ESPNOW_PAYLOAD_LENGTH			247
typedef struct {
    uint8_t type;							// enum ESPNOW_DATA_TYPE__*		
    uint16_t crc;							// CRC16 value of ESPNOW data.
    uint8_t payload[ESPNOW_PAYLOAD_LENGTH];	// Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    int len;                              // Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device.
} remote_ctrl_send_param_t;


/* Buffer to store received turret status*/
typedef struct {
	uint8_t* frame_buf;
	size_t frame_len;
	size_t recv_frame_len;	
	remote_ctrl__turret_status_t turret_status;
} remote_ctrl__turret_data_t;

enum {
	REMOTE_CTRL_STATE__LISTENING,
	REMOTE_CTRL_STATE__PAIRING,
	REMOTE_CTRL_STATE__DATA_WAITING,
	REMOTE_CTRL_STATE__DATA_RECEIVING,
	REMOTE_CTRL_STATE__IDLE,
	REMOTE_CTRL_STATE__MAX
};
extern uint8_t remote_ctrl_state;

extern QueueHandle_t remote_ctrl_event_queue;

extern TaskHandle_t remote_ctrl_task_handle;

esp_err_t remote_ctrl_init(void);
#endif
