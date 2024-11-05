#ifndef ORIENTATION_CTRL_TASK_H
#define ORIENTATION_CTRL_TASK_H

typedef enum {
	SWITCH_TELESCOPIC_ARM,
    SET_PAN_TILT,
	RESET_PAN_TILT_ALARM,
	ENABLE_TIMEOUT_RETRACT,
	DUMP_RECV_COOR,
} orientation_ctrl_event_id_t;

typedef struct {
    orientation_ctrl_event_id_t id;
    void* info;
} orientation_ctrl_event_t;

typedef struct {
	int16_t pan_angle_offset_x10;
	int16_t tilt_angle_offset_x10;
	uint16_t distance;
} target_position_t;

#define ORI_CTRL_STATUS__TELESCOPE_ARMS_EXTENDED				0x0001
#define ORI_CTRL_STATUS__PAN_TILT_FOR_RETRACT_TELESCOPIC_ARMS	0x0002
#define ORI_CTRL_STATUS__TIMEOUT_RETRACT_EN						0x0008
#define ORI_CTRL_STATUS__PAN_LEFT_ENDSTOP_ALARM_EN				0x0010
#define ORI_CTRL_STATUS__PAN_RIGHT_ENDSTOP_ALARM_EN				0x0020
#define ORI_CTRL_STATUS__TILT_UP_ENDSTOP_ALARM_EN				0x0040
#define ORI_CTRL_STATUS__TILT_DOWN_ENDSTOP_ALARM_EN				0x0080
#define ORI_CTRL_STATUS__EXTEND_TELESCOPE_ARMS_FROM_TIMEOUT		0x0100
extern uint16_t orientation_ctrl_status;

extern QueueHandle_t orientation_ctrl_event_queue;

esp_err_t orientation_ctrl_init(void);
void orientation_ctrl_get_view_angle(int16_t* pan_angle_x10, int16_t* tilt_angle_x10);
void orientation_ctrl_get_aim_angle(int16_t* pan_angle_x10, int16_t* tilt_angle_x10, uint8_t* grouping_diameter, uint16_t aim_distance);

#define STEPPER_MOTOR_STEP_HALF_PERIOD_MS		1
void stepper_motor_steps_ctrl_isr(void);

#define ORIENTATION_CTRL__TIMEOUT_TO_RETRACT	20

#endif