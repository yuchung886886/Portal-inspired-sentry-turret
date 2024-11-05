#ifndef COMMON_H
#define COMMON_H

typedef struct {
	int16_t view_pan_angle_x10;
	int16_t view_tilt_angle_x10;
	int16_t aim_pan_angle_x10;	
	int16_t aim_tilt_angle_x10;
	uint8_t aim_grouping_diameter;
	uint16_t aim_distance;
	#define FIRE_CTRL_STATUS__SAFETY_EN			0x01
	#define FIRE_CTRL_STATUS__IS_FIRING			0x02
	#define FIRE_CTRL_STATUS__SPRING_JAMMING	0x40
	uint8_t fire_ctrl_status;
	#define OC_STATUS__TELESCOPE_ARMS_EXTENDED	0x01
	uint8_t orientation_ctrl_status;
} remote_ctrl__turret_status_t;

typedef struct {
	#define TURRET_CTRL_BITSMAP__SET_PAN_TILT						0x01
	#define TURRET_CTRL_BITSMAP__AIM_ICON_IS_PRESSED				0x02
	#define TURRET_CTRL_BITSMAP__SAFETY_BTN_IS_PRESSED				0x04
	#define TURRET_CTRL_BITSMAP__TELESCOPIC_ARM_CTRL_BTN_IS_PRESSED	0x08	
	uint8_t turret_ctrl;
	int16_t pan_angle_offset_x10;
	int16_t tilt_angle_offset_x10;
} remote_ctrl__turret_cmd_t;
	
#endif
