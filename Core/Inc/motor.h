/*
 * motor.h
 *
 *  Created on: Aug 29, 2020
 *      Author: Marko Juhanne
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#define VERSION_MAJOR 0
#define VERSION_MINOR 76

#define GEAR_RATIO 171

// If != 0,  motor will be stopped if voltage drops below minimum voltage (in order to protect battery)
#define DEFAULT_MINIMUM_VOLTAGE 0	// voltage check is bypassed

#define DEFAULT_FULL_CURTAIN_LEN GEAR_RATIO * (13 + 265.0/360) * 4

#define DEFAULT_TARGET_SPEED 18	// RPM

#define DEFAULT_AUTO_CAL_SETTING 1	// auto-calibration is enabled by default

/* If no hall sensor interrupts are received during this time period, assume motor is stopped/stalled */
#define HALL_SENSOR_TIMEOUT 300 // Milliseconds.

/*
 * Allow slightly longer timeout when stopping
 */
#define HALL_SENSOR_TIMEOUT_WHILE_STOPPING 1000 // Milliseconds.

/* If motor has been just energized, we will allow longer timeout period before stall detection is applied */
#define HALL_SENSOR_GRACE_PERIOD 1000 // Milliseconds

/*
 * After the motor is stalled, we wait a bit for the curtain tension to release and the curtain rod to settle. After this period
 * motor is considered to be in top position (location = 0)
 */
#define ENDPOINT_CALIBRATION_PERIOD 1000 // Milliseconds


/*
 * These parameters are used to slow down the motor speed when approaching the target location
 */
#define DEFAULT_MINIMUM_SLOWDOWN_SPEED 5
#define DEFAULT_SLOWDOWN_FACTOR 48


typedef enum motor_status {
	Stopped,
	Moving,
	Stopping,
	CalibratingEndPoint,
	Error
} motor_status;

typedef enum motor_direction {
	None,
	Up,
	Down
} motor_direction;

typedef enum motor_command {
	NoCommand,
	MotorUp,
	MotorDown,
	Stop
} motor_command;

uint8_t handle_command(uint8_t * rx_buffer, uint8_t * tx_buffer, uint8_t burstindex, uint8_t * tx_bytes);

void motor_init();
void motor_load_settings();
void motor_set_default_settings();
void motor_stop();
void motor_adjust_rpm();
void motor_stall_check();
void motor_process();

#endif /* SRC_MOTOR_H_ */
