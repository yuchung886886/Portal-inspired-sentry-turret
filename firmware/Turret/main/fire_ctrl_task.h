#ifndef FIRE_CTRL_TASK_H
#define FIRE_CTRL_TASK_H

typedef enum {
    OPEN_FIRE,
	CEASE_FIRE,
	SWITCH_SAFETY,
} fire_ctrl_event_id_t;

typedef struct {
    fire_ctrl_event_id_t id;
    void* info;
} fire_ctrl_event_t;

extern QueueHandle_t fire_ctrl_event_queue;
extern uint8_t motor_pwm_duty_L;
extern uint8_t motor_pwm_duty_R;
extern uint8_t fire_ctrl_status;

#define FIRE_CTRL_ISR_PERIOD_MS	10
void fire_ctrl__isr(void);

esp_err_t fire_ctrl__init(void);
bool fire_ctrl__is_firing(void);


#endif