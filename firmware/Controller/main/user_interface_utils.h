#ifndef USER_INTERFACE_UTILS_H
#define USER_INTERFACE_UTILS_H

#include "remote_ctrl_task.h"

// RGB565 color code
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07
#define GRAY  			 0X8430
#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C
#define GRAYBLUE       	 0X5458
#define LIGHTGREEN     	 0X841F
#define LGRAY 			 0XC618
#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12

void ui__fill_color(uint8_t* bitmap, uint16_t color);
void ui__draw_a_cross(uint8_t* bitmap, uint16_t color, uint16_t x, uint16_t y, uint8_t width);
void ui__draw_turret_status(uint8_t* bitmap, remote_ctrl__turret_data_t* td);
void ui__update_turret_command(remote_ctrl__turret_cmd_t* tc, remote_ctrl__turret_data_t* td);

#endif