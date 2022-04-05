/*
 * motor.h
 *
 *  Created on: Aug 29, 2020
 *      Author: Marko Juhanne
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#define VERSION_MAJOR 0
#define VERSION_MINOR 85

#define GEAR_RATIO 171

#define NORMAL_ORIENTATION	0	// Curtain is installed in normal ("back roll") configuration
#define REVERSE_ORIENTATION	1	// Curtain is installed in reverse configuration (curtain rod is flipped to "front roll" configuration)

// If != 0,  motor will be stopped if voltage drops below minimum voltage (in order to protect battery)
#define DEFAULT_MINIMUM_VOLTAGE 0	// voltage check is bypassed

#define DEFAULT_FULL_CURTAIN_LEN GEAR_RATIO * (13 + 265.0/360) * 4

#define DEFAULT_TARGET_SPEED 18	// RPM
#define READ_DEFAULT_SPEED_FROM_EEPROM  // Reads default speed from EEPROM if it's stored there. Otherwise use the value above

#define DEFAULT_AUTO_CAL_SETTING 1	// auto-calibration is enabled by default

#define DEFAULT_ORIENTATION NORMAL_ORIENTATION

/* 
The DEFAULT_MAX_MOTOR_CURRENT is the maximum amount of current that motor can draw while moving. Used for stall detection 
at the top position. Note that this is upper limit is not enforced when starting the movement in order to break the static friction!
*/
#define DEFAULT_MAX_MOTOR_CURRENT 2048 // in mA.  0 == DISABLED

// We want at least this many sensor ticks to assume the motor is moving (and free of static friction). In reality
// this should be lower number, but it can take few ticks before the measured current starts to drop even though static friction
// has already been overcome. This is done because we don't want to erroneously think the motor is stalling 
#define MIN_SENSOR_TICKS 8

/* If no hall sensor interrupts are received during this time period, assume motor is stopped/stalled */
#define DEFAULT_STALL_DETECTION_TIMEOUT 296 // Milliseconds.

/* If motor has been just energized, we will allow longer timeout period before stall detection is applied */
#define HALL_SENSOR_TIMEOUT_WHILE_STARTING 1000 // Milliseconds

/*
 * Allow slightly longer timeout when stopping also
 */
#define HALL_SENSOR_TIMEOUT_WHILE_STOPPING 1000 // Milliseconds.


/*
 * After the motor is stalled, we wait a bit for the curtain tension to release and the curtain rod to settle. After this period
 * motor is considered to be in top position (location = 0)
 */
#define ENDPOINT_CALIBRATION_PERIOD 1000 // Milliseconds


/* 
 * The minimum motor current (high enough resistance) which is needed to signal that the motor has indeed stalled
 * at the upmost position. If motor has stalled with a current below this value, it would suggest that during low speed 
 * (usually RPM < 5) movement there was sudden increase in friction which caused the motor to stall. The lower voltages
 * used for low speeds most likely make the motor more susceptible to stall. 
 * 
 * Usually the RPM adjustment function would increase the PWM duty cycle but sometimes it doesn't respond quickly enough 
 * before Hall sensor timeout kicks in.
 * 
 * If this happens, you can try using a higher speeds or try to make sure there isn't any extra friction in the system
 * (such as those protruding metal tabs in the motor housing when it was disassembled and reassembled). In any case,
 * it seems that 3-4 RPM is the minimum speed which the Fyrtur can sustaing without stalling but your mileage may wary.
 */
#define MINIMUM_CALIBRATION_CURRENT 500 // mA


/*
 * These parameters are used to slow down the motor speed when approaching the target location
 */
#define DEFAULT_MINIMUM_SLOWDOWN_SPEED 5
#define DEFAULT_SLOWDOWN_FACTOR 48

/*
 * "Dance" is a series of movement steps (up or down) that the firmware uses to signal the user that it has acknowleged
 * certain commands (such as CMD_SET_MAX_CURTAIN_LENGTH or CMD_SET_FULL_CURTAIN_LENGTH)
 */
#define DANCE_STEP_LENGTH 100	// length of one dance step (measured in Hall sensor ticks)


/*
 * Flexi-speed is a mechanism to change the motor speed setting even when using the custom firmware 
 * with original Ikea Fyrtur Zigbee module. There are 4 different speed settings (3, 5, 15 and 25 RPM). User can
 * cycle between them by repeatedly rolling the blinds up (CMD_UP command) 3 times. Firmware then selects the next speed 
 * in the preset list and will signal the user by doing a little "down/up dance" curtain movement.
 */
// disabled by default
//#define FLEXISPEED_ENABLED

/*
 * The number of repeated CMD_UP commands before we cycle to next speed preset setting
 */
#define FLEXISPEED_TRIGGER_LIMIT 3




typedef enum motor_status_t {
	Stopped,
	Moving,
	Stopping,
	CalibratingEndPoint,
	Bootloader,
	Error
} motor_status_t;

typedef enum motor_error_t {
	NoError = 0,
	StalledMovingUp,
	StalledMovingDown,
	FrictionError
} motor_error_t;

typedef enum motor_direction_t {
	None,
	Up,
	Down
} motor_direction_t;

typedef enum motor_command_t {
	NoCommand,
	MotorUp,
	MotorDown,
	Stop,
	EnterBootloader,
	Dance,
} motor_command_t;

uint8_t handle_command(uint8_t * rx_buffer, uint8_t * tx_buffer, uint8_t * tx_bytes);

void motor_init();
void motor_load_settings();
void motor_set_default_settings();
void motor_stop();
void motor_adjust_rpm();
void motor_stall_check();
void motor_process();

#endif /* SRC_MOTOR_H_ */
