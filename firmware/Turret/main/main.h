#ifndef MAIN_H
#define MAIN_H

#define I2C_PORT_DEFAULT	1
#define I2C_PIN_SDA    		38
#define I2C_PIN_SCL    		0
#define WRITE_BIT			I2C_MASTER_WRITE      /*!< I2C master write */
#define READ_BIT			I2C_MASTER_READ       /*!< I2C master read */
#define ACK_CHECK_EN		0x1                   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS		0x0                   /*!< I2C master will not check ack from slave */
#define ACK_VAL				0x0                   /*!< I2C ack value */
#define NACK_VAL			0x1                   /*!< I2C nack value */

#define TASK_CPU_CORE__REMOTE_CTRL		0
#define TASK_CPU_CORE__ORIENTATION_CTRL	1
#define TASK_CPU_CORE__FIRE_CTRL		1
#define TASK_CPU_CORE__SPEAKER_CTRL		1

#define TASK_PRIORITY__REMOTE_CTRL				(configMAX_PRIORITIES - 1)
#define TASK_PRIORITY__SPEAKER_CTRL				(configMAX_PRIORITIES - 2)
#define TASK_PRIORITY__ORIENTATION_CTRL			(configMAX_PRIORITIES - 3)
#define TASK_PRIORITY__FIRE_CTRL				(configMAX_PRIORITIES - 4)

#define GLOBAL_TIMER_INTR_PEROID	1
extern uint16_t global_timer_ms_count;

#endif