#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "main.h"
#include "range_finder_utils.h"

#define SRF02_SLAVE_ADDR	0x70
#define REG_ADDR__CMD		0x00
#define REG_ADDR__RANGE_H	0x02
#define REG_ADDR__RANGE_L	0x03
#define CMD__RANGING_INCH	0x50
#define CMD__RANGING_CM		0x51
#define CMD__RANGING_MS		0x52
static const char *TAG = "range_finder";

esp_err_t range_finder__get_range(uint16_t* range){
    i2c_cmd_handle_t cmd;
	uint8_t data[8] = {0};
	esp_err_t ret = ESP_FAIL;
	
	// Get the last ranging result
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SRF02_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);	
	data[0] = REG_ADDR__RANGE_H;
	i2c_master_write(cmd, data, 1, ACK_CHECK_EN);
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C_Write Failed addr:0x%02x, data:0x%02x, ret:%d", SRF02_SLAVE_ADDR, REG_ADDR__RANGE_H, ret);
		return ret;
    }	
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SRF02_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);	
    i2c_master_read_byte(cmd, &data[0], ACK_VAL);
    i2c_master_read_byte(cmd, &data[1], NACK_VAL);
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C_Read Failed addr:0x%02x, ret:%d", SRF02_SLAVE_ADDR, ret);
		return ret;
    }else{		
		*range = (data[0] << 8) | data[1];
	}

	// Dispatch ranging for the next get_range
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SRF02_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);

	data[0] = CMD__RANGING_CM;
	i2c_master_write(cmd, data, 1, ACK_CHECK_EN);
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C_Write Failed addr:0x%02x, data:0x%02x 0x%02x, ret:%d", SRF02_SLAVE_ADDR, data[0], data[1], ret);
		return ret;
    }		
			
	return ESP_OK;
}
