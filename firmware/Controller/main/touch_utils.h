#ifndef TOUCH_UTILS_H
#define TOUCH_UTILS_H

esp_err_t touch_utils_init(void);
void touch_calibration_task_init(void);

bool touch_is_pressed(uint16_t* z);
esp_err_t touch_get_raw(uint16_t* x, uint16_t* y);
esp_err_t touch_get_coordinate(uint16_t* x, uint16_t* y);

#endif