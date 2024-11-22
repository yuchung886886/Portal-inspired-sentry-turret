#ifndef SPEAKER_CTRL_TASK_H
#define SPEAKER_CTRL_TASK_H

typedef enum {
    PLAY_MUSIC,
	STOP_MUSIC,
} speaker_ctrl_event_id_t;

typedef struct {
    speaker_ctrl_event_id_t id;
    void* info;
} speaker_ctrl_event_t;

#define SPEAKER_CTRL_STATUS__BUSY	0x01
extern uint8_t speaker_ctrl_status;

esp_err_t speaker_ctrl_task_init(void);

#define SOUND_TRACK__DEPLOY					0
#define SOUND_TRACK__ALARM					1
#define SOUND_TRACK__HELLO					2
#define SOUND_TRACK__ANYONE_THERE			3
#define SOUND_TRACK__TARGET_ACQUIRED		4
#define SOUND_TRACK__MALFUNCTION			5
#define SOUND_TRACK__RETRACT				6
#define SOUND_TRACK__GOODNIGHT				7
#define SOUND_TRACK__ARE_YOU_STILL_THERE	8
#define SOUND_TRACK__OPERA_SINGING			9
esp_err_t speaker_ctrl__play_music(uint8_t track_index);
esp_err_t speaker_ctrl__stop_music();

#endif