#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include "esp_log.h"
#include "../../Common/common.h"
#include "remote_ctrl_task.h"
#include "lcd_task.h"
#include "touch_utils.h"
#include "user_interface_utils.h"
#include "font.h"

static const char *TAG = "user_interface";

#define FONT_SIZE__INFO		12
#define FONT_SIZE__BUTTON	24
#define FONT_HEIGHT__BUTTON	FONT_SIZE__BUTTON
#define FONT_WIDTH__BUTTON	12
#define ITEM_GAP			2

static const uint8_t button_icon__extend_telescopic_arm[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFE, 0x00, 0x00,
	0x00, 0x00, 0xFE, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x18, 0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0x7F, 0x00, 0x00,
	0x00, 0x00, 0x7F, 0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	
};
static const uint8_t button_icon__retract_telescopic_arm[] = {
	0x00, 0x00, 0xFE, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFE, 0x00, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x18, 0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x7F, 0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0x7F, 0x00, 0x00
};
static const uint8_t button_icon__safety_off[] = {
	0x00, 0x00, 0xF0, 0xF0, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x70, 0x60,
	0x40, 0x60, 0x60, 0x60, 0x80, 0xC0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x81, 0xC3, 0xC3, 0x66, 0x3C, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x0F, 0x07, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x04, 0x0C, 0x0C, 0x0C, 0x0C, 0x0E, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x03, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t button_icon__safety_on[] = {
	0x00, 0x00, 0xF0, 0xF0, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x70, 0x60,
	0x40, 0x60, 0x60, 0xE0, 0xC0, 0xE0, 0xF0, 0xB8, 0x9C, 0x0E, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x38,
	0x1C, 0x0E, 0x07, 0x03, 0x01, 0x00, 0x00, 0x81, 0x81, 0xC3, 0xC3, 0x66, 0x3C, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x0F, 0x87, 0xCC, 0xEC, 0x7C, 0x3C, 0x1C, 0x0E, 0x0F, 0x0F, 0x0D, 0x0C, 0x0E, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x03, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


static uint32_t uint8_pow(uint8_t m, uint8_t n);
static float interpolate_y(uint16_t x, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void get_aim_icon(uint16_t* center_coor_x, uint16_t* center_coor_y, uint8_t* length, 
						  int16_t pan_angle_x10, int16_t tilt_angle_x10, uint8_t grouping_diameter, uint16_t aim_distance);
static bool aim_icon_is_pressed(uint16_t touch_coor_x, uint16_t touch_coor_y);
static bool safty_button_is_pressed(uint16_t touch_coor_x, uint16_t touch_coor_y);

void ui__fill_color(uint8_t* bitmap, uint16_t color){
	for(int i = 0; i < LCD_H_RES * LCD_V_RES; i++){
		bitmap[i*2] = color & 0x00FF;
		bitmap[i*2 + 1] = color >> 8;
	}
}

void ui__draw_a_line(uint8_t* bitmap, uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
	uint16_t t;	
	int16_t xerr = 0, yerr = 0, delta_x, delta_y, distance; 
	int16_t incx, incy, uRow, uCol; 
	
	delta_x = x2 - x1;
	delta_y = y2 - y1; 
	uRow = x1;  
	uCol = y1; 
	
	if(delta_x > 0) incx = 1;
	else if(delta_x == 0) incx = 0;
	else {incx = -1; delta_x =-delta_x;} 
	if(delta_y > 0) incy = 1; 
	else if(delta_y == 0) incy = 0;
	else{incy = -1; delta_y = -delta_y;} 
	if(delta_x > delta_y) distance = delta_x;
	else distance = delta_y; 
	for(t = 0; t <= distance + 1; t++){  
		if((uRow >= 0) && (uRow < LCD_H_RES) && (uCol >= 0) && (uCol < LCD_V_RES)){
			bitmap[LCD_H_RES * 2 * uCol + 2 * uRow] = color & 0x00FF;
			bitmap[LCD_H_RES * 2 * uCol + 2 * uRow + 1] = color >> 8;			
		}		
		xerr += delta_x; 
		yerr += delta_y; 
		if(xerr > distance){ 
			xerr -= distance; 
			uRow += incx; 
		} 
		if(yerr > distance){ 
			yerr -= distance; 
			uCol += incy; 
		} 
	}  	
}

void ui__draw_a_cross(uint8_t* bitmap, uint16_t color, uint16_t x, uint16_t y, uint8_t width){	
	ui__draw_a_line(bitmap, color, (int16_t)(x - width / 2),  (int16_t)y, (int16_t)(x + width / 2),  (int16_t)y);
	ui__draw_a_line(bitmap, color, (int16_t)x,  (int16_t)(y - width / 2), (int16_t)x,  (int16_t)(y + width / 2));
}

#define MODE__HIGH_DIGITS_ZERO 	0x80
#define MODE__CLEAR_AREA		0x01
void ui__draw_a_rectangular(uint8_t* bitmap, uint16_t color, uint16_t x, uint16_t y, uint8_t width, uint8_t height, uint8_t mode){	
	if(mode & MODE__CLEAR_AREA){
		for(int row = y; row < y + height; row++){
			if((row >= 0) && (row < LCD_V_RES)){
				for(int col = x; col < x + width; col++){
					if((col >= 0) && (col < LCD_H_RES)){
						bitmap[LCD_H_RES * 2 * row + 2 * col] = color & 0x00FF;
						bitmap[LCD_H_RES * 2 * row + 2 * col + 1] = color >> 8;	
					}else{
						break;
					}
				}
			}else{
				break;
			}
		}
	}else{
		ui__draw_a_line(bitmap, color, x, y, x + width, y);
		ui__draw_a_line(bitmap, color, x, y, x, y + height);
		ui__draw_a_line(bitmap, color, x + width, y, x + width, y + height);
		ui__draw_a_line(bitmap, color, x, y + height, x + width, y + height);			
	}
}

#define BUTTON_WIDTH 	32
#define BUTTON_HEIGHT	24
void ui__draw_a_button(uint8_t* bitmap, uint16_t foreground_color, uint16_t background_color, uint16_t x, uint16_t y, const uint8_t* icon){
	uint8_t i, j;
	uint16_t pixel_coor_x, pixel_coor_y;
	uint8_t temp;
	
	for(i = 0; i < (BUTTON_WIDTH * BUTTON_HEIGHT) / 8; i++){
		temp = icon[i];
		for(j = 0; j < 8; j++){
			pixel_coor_x = x + (i % BUTTON_WIDTH);
			pixel_coor_y = y + (i / BUTTON_WIDTH) * 8 + j;
			if(pixel_coor_x < LCD_H_RES && pixel_coor_y < LCD_V_RES){
				if(temp & 0x01){				
					bitmap[(LCD_H_RES * pixel_coor_y + pixel_coor_x) * 2 ] = foreground_color & 0x00FF;
					bitmap[(LCD_H_RES * pixel_coor_y + pixel_coor_x) * 2 + 1] = foreground_color >> 8;					
				}else{
					bitmap[(LCD_H_RES * pixel_coor_y + pixel_coor_x) * 2 ] = background_color & 0x00FF;
					bitmap[(LCD_H_RES * pixel_coor_y + pixel_coor_x) * 2 + 1] = background_color >> 8;										
				}				
			}
			temp >>= 1;
		}
	}
}

void ui__draw_a_char(uint8_t* bitmap, uint16_t foreground_color, uint16_t background_color, uint16_t x, uint16_t y, uint8_t c, uint8_t font_size, uint8_t mode){  							  
    uint8_t temp , t1, t;
	uint16_t y0 = y;
	uint8_t csize = (font_size / 8 + ((font_size % 8)? 1 : 0)) * (font_size / 2);
	
 	c = c - ' ';
	for(t = 0; t < csize; t++){   
		if(font_size == 12){temp = ascii_1206[c][t];}
		else if(font_size==24)temp=ascii_2412[c][t];
		else return;
		for(t1 = 0; t1 < 8; t1++){			    
			if(temp & 0x80){
				bitmap[LCD_H_RES * 2 * y + 2 * x] = foreground_color & 0x00FF;
				bitmap[LCD_H_RES * 2 * y + 2 * x + 1] = foreground_color >> 8;	
			}else if((mode & MODE__CLEAR_AREA)){
				bitmap[LCD_H_RES * 2 * y + 2 * x] = background_color & 0x00FF;
				bitmap[LCD_H_RES * 2 * y + 2 * x + 1] = background_color >> 8;	
			}
			temp <<= 1;			
			y++;
			if(y >= LCD_V_RES){return;}
			if((y - y0) == font_size){
				y = y0;
				x++;
				if(x >= LCD_H_RES){return;}
				break;
			}
		}  	 
	}  	    	   	 	  
}

void ui__draw_numbers(uint8_t* bitmap, uint16_t foreground_color, uint16_t background_color, uint16_t x, uint16_t y, uint32_t num, uint8_t str_len, uint8_t font_size, uint8_t mode){  
	uint8_t t, temp;
	uint8_t enshow = 0;
	
	for(t = 0; t < str_len; t++){
		temp = (num / uint8_pow(10, str_len - t - 1)) % 10;
		if(enshow == 0 && t < (str_len - 1)){
			if(temp == 0){
				if(mode & MODE__HIGH_DIGITS_ZERO)
					ui__draw_a_char(bitmap, foreground_color, background_color, x + (font_size / 2) * t, y, '0', font_size, mode);  
				else 
					ui__draw_a_char(bitmap, foreground_color, background_color, x + (font_size / 2) * t, y, ' ', font_size, mode);  
 				continue;
			}
			else 
				enshow = 1; 		 	 
		}
	 	ui__draw_a_char(bitmap, foreground_color, background_color, x + (font_size / 2) * t, y, temp +'0', font_size, mode); 
	}
}

uint8_t aim_icon_length; 
uint16_t aim_icon_center_coor_x = 0;
uint16_t aim_icon_center_coor_y = 0;
static bool aim_icon_is_pressed(uint16_t touch_coor_x, uint16_t touch_coor_y){
	if(touch_coor_x < aim_icon_center_coor_x - aim_icon_length / 2){return false;}
	if(touch_coor_x > aim_icon_center_coor_x + aim_icon_length / 2){return false;}
	if(touch_coor_y < aim_icon_center_coor_y - aim_icon_length / 2){return false;}
	if(touch_coor_y > aim_icon_center_coor_y + aim_icon_length / 2){return false;}
	
	return true;
}

#define BUTTON_SAFETY__LEFT_UP_COOR_X	(ITEM_GAP)
#define BUTTON_SAFETY__LEFT_UP_COOR_Y	(ITEM_GAP)
static bool safty_button_is_pressed(uint16_t touch_coor_x, uint16_t touch_coor_y){
	if(touch_coor_x < BUTTON_SAFETY__LEFT_UP_COOR_X){return false;}
	if(touch_coor_x > BUTTON_SAFETY__LEFT_UP_COOR_X + BUTTON_WIDTH){return false;}
	if(touch_coor_y < BUTTON_SAFETY__LEFT_UP_COOR_Y){return false;}
	if(touch_coor_y > BUTTON_SAFETY__LEFT_UP_COOR_Y + BUTTON_HEIGHT){return false;}
	return true;
}

#define BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_X	(ITEM_GAP)
#define BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_Y	(BUTTON_SAFETY__LEFT_UP_COOR_Y + BUTTON_HEIGHT + ITEM_GAP)
static bool telescopic_arm_ctrl_button_is_pressed(uint16_t touch_coor_x, uint16_t touch_coor_y){
	if(touch_coor_x < BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_X){return false;}
	if(touch_coor_x > BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_X + BUTTON_WIDTH){return false;}
	if(touch_coor_y < BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_Y){return false;}
	if(touch_coor_y > BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_Y + BUTTON_HEIGHT){return false;}
	return true;
}

#define COOR_X_OFFSET_MIN	5
#define COOR_Y_OFFSET_MIN	5
void ui__draw_turret_status(uint8_t* bitmap, remote_ctrl__turret_data_t* td){
	uint16_t foreground_color, background_color;
	uint8_t n;
		
	// Draw safty ctrl button
	if(td->turret_status.fire_ctrl_status & FIRE_CTRL_STATUS__SAFETY_EN){
		foreground_color = BLACK;	background_color = GREEN;
		ui__draw_a_button(bitmap, foreground_color, background_color, 
						  BUTTON_SAFETY__LEFT_UP_COOR_X, 
						  BUTTON_SAFETY__LEFT_UP_COOR_Y, 
						  button_icon__safety_on);
	}else{
		foreground_color = BLACK;	background_color = RED;	
		ui__draw_a_button(bitmap, foreground_color, background_color, 
						  BUTTON_SAFETY__LEFT_UP_COOR_X, 
						  BUTTON_SAFETY__LEFT_UP_COOR_Y, 
						  button_icon__safety_off);		
	}

	// Draw telescopic arm ctrl button
	if(td->turret_status.orientation_ctrl_status & OC_STATUS__TELESCOPE_ARMS_EXTENDED){
		foreground_color = BLACK;	background_color = GREEN;	
		ui__draw_a_button(bitmap, foreground_color, background_color, 
						  BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_X, 
						  BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_Y, 
						  button_icon__retract_telescopic_arm);		
	}else{
		foreground_color = BLACK;	background_color = GREEN;	
		ui__draw_a_button(bitmap, foreground_color, background_color, 
						  BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_X, 
						  BUTTON_TELESCOPIC_ARM_CTRL__LEFT_UP_COOR_Y, 
						  button_icon__extend_telescopic_arm);			
	}	
	
	// Draw aim sight	
	if(td->turret_status.fire_ctrl_status & FIRE_CTRL_STATUS__IS_FIRING){foreground_color = RED;}
	else{foreground_color = GREEN;}	
	get_aim_icon(&aim_icon_center_coor_x, &aim_icon_center_coor_y, &aim_icon_length, 
				  td->turret_status.aim_pan_angle_x10, td->turret_status.aim_tilt_angle_x10, td->turret_status.aim_grouping_diameter,
				  td->turret_status.aim_distance);
	ui__draw_a_rectangular(bitmap, foreground_color, 
						   aim_icon_center_coor_x - (aim_icon_length / 2), 
						   aim_icon_center_coor_y - (aim_icon_length / 2),
						   aim_icon_length,
						   aim_icon_length,
						   0);				  					   
	
	// Append aim distance
	n = td->turret_status.aim_distance / 100;	
	ui__draw_numbers(bitmap, foreground_color, BLACK, 
					 aim_icon_center_coor_x - (aim_icon_length / 2),  
					 aim_icon_center_coor_y + (aim_icon_length / 2), 
					 n, 
					 1, FONT_SIZE__INFO, 0);
	ui__draw_a_char(bitmap, foreground_color, BLACK, 
					aim_icon_center_coor_x - (aim_icon_length / 2) + (FONT_SIZE__INFO / 2) * 1,   
					aim_icon_center_coor_y + (aim_icon_length / 2), 
					'.', 
					FONT_SIZE__INFO, 0);  	
	n = (td->turret_status.aim_distance % 100) / 10;
	ui__draw_numbers(bitmap, foreground_color, BLACK, 
					 aim_icon_center_coor_x - (aim_icon_length / 2) + (FONT_SIZE__INFO / 2) * 2,  
					 aim_icon_center_coor_y + (aim_icon_length / 2), 
					 n, 
					 1, FONT_SIZE__INFO, 0);	
					 					 
	// Append pan angle
	ui__draw_numbers(bitmap, foreground_color, BLACK, 
					 aim_icon_center_coor_x - (FONT_SIZE__INFO / 2),  
					 LCD_V_RES - ITEM_GAP - FONT_SIZE__INFO, 
					 abs(td->turret_status.view_pan_angle_x10 / 10) % 100, 
					 2, FONT_SIZE__INFO, MODE__HIGH_DIGITS_ZERO);					 
	if(td->turret_status.view_pan_angle_x10 < 0){
		ui__draw_a_char(bitmap, foreground_color, BLACK, 
						aim_icon_center_coor_x - (FONT_SIZE__INFO / 2) * 2,   
						LCD_V_RES - ITEM_GAP - FONT_SIZE__INFO, 
						'-', 
						FONT_SIZE__INFO, 0);		
	}
	if((aim_icon_center_coor_y + (aim_icon_length / 2) + FONT_SIZE__INFO + ITEM_GAP) < (LCD_V_RES - ITEM_GAP - FONT_SIZE__INFO - ITEM_GAP)){
		ui__draw_a_line(bitmap, foreground_color, 
						aim_icon_center_coor_x, aim_icon_center_coor_y + (aim_icon_length / 2) + FONT_SIZE__INFO + ITEM_GAP, 
						aim_icon_center_coor_x, LCD_V_RES - ITEM_GAP - FONT_SIZE__INFO - ITEM_GAP);		
	}
	
	// Append tilt angle
	ui__draw_numbers(bitmap, foreground_color, BLACK, 
					 LCD_H_RES - (FONT_SIZE__INFO / 2) * 2,  
					 aim_icon_center_coor_y - (FONT_SIZE__INFO / 2), 
					 abs(td->turret_status.view_tilt_angle_x10 / 10) % 100, 
					 2, FONT_SIZE__INFO, MODE__HIGH_DIGITS_ZERO);	
	if(td->turret_status.view_tilt_angle_x10 < 0){
		ui__draw_a_char(bitmap, foreground_color, BLACK, 
						LCD_H_RES - (FONT_SIZE__INFO / 2) * 3,   
						aim_icon_center_coor_y - (FONT_SIZE__INFO / 2), 
						'-', 
						FONT_SIZE__INFO, 0);		
	}
	if((aim_icon_center_coor_x + (aim_icon_length / 2) + ITEM_GAP * 2) < (LCD_H_RES - (FONT_SIZE__INFO / 2) * 3 - ITEM_GAP)){
		ui__draw_a_line(bitmap, foreground_color, 
						aim_icon_center_coor_x + (aim_icon_length / 2) + ITEM_GAP * 2, aim_icon_center_coor_y, 
						LCD_H_RES - (FONT_SIZE__INFO / 2) * 3 - ITEM_GAP, aim_icon_center_coor_y);		
	}	
	
	
	// Draw camera center		
	foreground_color = BLUE;	
	ui__draw_a_rectangular(bitmap, foreground_color, 
						   LCD_H_RES / 2 - COOR_X_OFFSET_MIN, 
						   LCD_V_RES / 2 - COOR_Y_OFFSET_MIN,
						   COOR_X_OFFSET_MIN * 2, COOR_Y_OFFSET_MIN * 2,
						   0);			
	
}

uint8_t touch_is_pressed_prev = 0;	
void ui__update_turret_command(remote_ctrl__turret_cmd_t* tc, remote_ctrl__turret_data_t* td){
	uint16_t touch_coor_x = 0, touch_coor_y = 0;	
	uint8_t turret_ctrl_bitsmap = 0;
	int16_t pan_angle_offset_x10 = 0;
	int16_t tilt_angle_offset_x10 = 0;
	float opposite_length, max_opposite_length;
	
	if(touch_is_pressed(NULL)){
		if(touch_get_coordinate(&touch_coor_x, &touch_coor_y) == ESP_OK){
			if(safty_button_is_pressed(touch_coor_x, touch_coor_y) == true){
				if(!touch_is_pressed_prev){turret_ctrl_bitsmap |= TURRET_CTRL_BITSMAP__SAFETY_BTN_IS_PRESSED;}
			}
			else if(telescopic_arm_ctrl_button_is_pressed(touch_coor_x, touch_coor_y) == true){						
				if(!touch_is_pressed_prev){turret_ctrl_bitsmap |= TURRET_CTRL_BITSMAP__TELESCOPIC_ARM_CTRL_BTN_IS_PRESSED;}
			}
			else{
				if(aim_icon_is_pressed(touch_coor_x, touch_coor_y) == true){
					turret_ctrl_bitsmap |= TURRET_CTRL_BITSMAP__AIM_ICON_IS_PRESSED;
				}
				
				if(td->turret_status.aim_distance){
					max_opposite_length = td->turret_status.aim_distance * tan((float)CONFIG_HORIZ_FOV_ANGLE / 2 * 3.1416 / 180);
					if(abs(touch_coor_x - aim_icon_center_coor_x) > COOR_X_OFFSET_MIN){
						turret_ctrl_bitsmap |= TURRET_CTRL_BITSMAP__SET_PAN_TILT;								
						opposite_length = interpolate_y(abs(touch_coor_x - LCD_H_RES / 2), 
														0, 0,
														LCD_H_RES / 2, max_opposite_length);
						pan_angle_offset_x10 = (atan(opposite_length / td->turret_status.aim_distance) * 180 / 3.1416) * 10;
						if(touch_coor_x < (LCD_H_RES / 2)){pan_angle_offset_x10 = -pan_angle_offset_x10;}
						pan_angle_offset_x10 = pan_angle_offset_x10 - td->turret_status.aim_pan_angle_x10;
					}else{
						pan_angle_offset_x10 = 0;
					}				
					if(abs(touch_coor_y - aim_icon_center_coor_y) > COOR_Y_OFFSET_MIN){
						turret_ctrl_bitsmap |= TURRET_CTRL_BITSMAP__SET_PAN_TILT;
						opposite_length = interpolate_y(abs(touch_coor_y - LCD_V_RES / 2), 
														0, 0,
														LCD_H_RES / 2, max_opposite_length);
						tilt_angle_offset_x10 = (atan(opposite_length / td->turret_status.aim_distance) * 180 / 3.1416) * 10;	
						if(touch_coor_y > (LCD_V_RES / 2)){tilt_angle_offset_x10 = -tilt_angle_offset_x10;}
						tilt_angle_offset_x10 = tilt_angle_offset_x10 - td->turret_status.aim_tilt_angle_x10;
					}else{
						tilt_angle_offset_x10 = 0;
					}													
				}
			}
			touch_is_pressed_prev = 1;		
		}else{
			// TBD, touch SPI communication fail.
		}			
	}else{
		touch_is_pressed_prev = 0;		
	}	
	
	tc->turret_ctrl = turret_ctrl_bitsmap;
	tc->pan_angle_offset_x10 = pan_angle_offset_x10;
	tc->tilt_angle_offset_x10 = tilt_angle_offset_x10;
}

static uint32_t uint8_pow(uint8_t m, uint8_t n){
	uint32_t result = 1;
	
	while(n--){result *= m;}    
	return result;	
}

static float interpolate_y(uint16_t x, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
	return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

static void get_aim_icon(uint16_t* center_coor_x, uint16_t* center_coor_y, uint8_t* length, 
						  int16_t pan_angle_x10, int16_t tilt_angle_x10, uint8_t grouping_diameter, uint16_t aim_distance){
	float offset, opposite_length, max_opposite_length;	

	if(aim_distance){
		max_opposite_length = aim_distance * tan((float)CONFIG_HORIZ_FOV_ANGLE / 2 * 3.1416 / 180);
		
		opposite_length = fabs(aim_distance * tan(fabs((float)pan_angle_x10 / 10) * 3.1416 / 180));
		offset = interpolate_y(opposite_length, 
							   0, 0,
							   max_opposite_length, LCD_H_RES / 2);
		*center_coor_x = (pan_angle_x10 > 0) ? ((LCD_H_RES / 2) + offset) : ((LCD_H_RES / 2) - offset);	
		
		opposite_length = fabs(aim_distance * tan(fabs((float)tilt_angle_x10 / 10) * 3.1416 / 180));
		offset = interpolate_y(opposite_length, 
							   0, 0,
							   max_opposite_length, LCD_H_RES / 2);
		*center_coor_y = (tilt_angle_x10 > 0) ? ((LCD_V_RES / 2) - offset) : ((LCD_V_RES / 2) + offset);	
		*length = interpolate_y(grouping_diameter, 
								0, 0,
								max_opposite_length, LCD_H_RES / 2);		
		if(*length < LCD_V_RES / 10){*length = LCD_V_RES / 10;}						
	}else{
		*center_coor_x = LCD_H_RES / 2;
		*center_coor_y = LCD_V_RES / 2;
		*length = LCD_V_RES;
	}
}