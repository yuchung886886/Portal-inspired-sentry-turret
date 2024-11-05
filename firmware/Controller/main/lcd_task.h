#ifndef LCD_TASK_H
#define LCD_TASK_H

// The resolution of the LCD display
#define LCD_H_RES              320
#define LCD_V_RES              240

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

typedef enum {
    FLUSH_WITH_TURRET_DATA,
	FLUSH_WITH_BITMAP,
    FLUSH_DONE,
	GET_FRAME_RATE,
} lcd_event_id_t;

typedef struct {
    lcd_event_id_t id;
    void* info;
} lcd_event_t;

extern QueueHandle_t lcd_event_queue;
extern uint8_t* lcd_frame_bitmap;

esp_err_t lcd_init(void);

#endif