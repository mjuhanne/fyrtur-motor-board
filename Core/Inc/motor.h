/*
 * motor.h
 *
 *  Created on: Aug 29, 2020
 *      Author: markojuhanne
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#define DEFAULT_TARGET_SPEED 18

/* If no hall sensor interrupts are received during this time period, sssume motor is stopped/stalled */
#define HALL_TIMEOUT 300 // Milliseconds.

/* If motor has been just energized, we will allow longer grace period before stall detection is applied */
#define MOVEMENT_GRACE_PERIOD 1000 // Milliseconds

enum motor_status {
	Stopped,
	Moving,
	Error
};

enum motor_direction {
	None,
	Up,
	Down
};

enum motor_command {
	NoCommand,
	MotorUp,
	MotorDown,
	Stop
};

uint8_t handle_command(uint8_t * rx_buffer, uint8_t * tx_buffer, uint8_t burstindex, uint8_t * tx_bytes);

void motor_init();
void motor_stop();
void motor_adjust_rpm();
void motor_stall_check();
void motor_process();

#endif /* SRC_MOTOR_H_ */
