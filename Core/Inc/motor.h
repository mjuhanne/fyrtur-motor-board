/*
 * motor.h
 *
 *  Created on: Aug 29, 2020
 *      Author: Marko Juhanne
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#define VERSION_MAJOR 0
#define VERSION_MINOR 83

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

UPDATE 4/2022: 

TLDR; This should be safe BUT DON'T COME AFTER ME WITH A LAWYER IF YOUR HOUSE BURNS DOWN :)

Versions 0.82 (and earlier) have a predefined maximum motor current limit setting (1A default, configurable up to 2A).
This software limit can cause annoyances on some motor units which have higher static and/or dynamic friction than anticipated. This causes inconsistently high power draw and thus the firmware makes false assumption that the motor is stalled 
and the firmware then resets its known position to zero (upmost position) erroneously. When this position mismatch 
occurs it's a recipe for disaster because when the user (or most likely the home automation system as scheduled) 
tries to roll blinds down it causes movement further below the absolute lower limit, after which blinds start to move up 
but in reverse orientation.
This can create an annoying mess with wrinkled curtains and even damage the gluing between the curtains and the axel rod!

This false stall detection behaviour has sometimes occured even after blinds have started to rotate a bit and they should be 
free of static friction, so it's possible that abnormal current spikes might be detected by the current sensing circuitry 
also after initial movement.

The current limit feature in this firmware was designed to be an extra layer of safety in addition to the fuse on the custom (NOT ORIGINAL!) Fyrtur board. The main method of stall detection is via timeout of signals from the Hall sensors. 
This is also the way the original firmware works as far as I know. In my reverse engineering project of the original Ikea motor firmware I didn't find any evidence that the current sensing circuitry would be used for stall detection (take this with a grain of salt - it's possible that I have overlooked this part of the code). So it seems that even Ikea (or the 
subcontractor that manufactured the motor module) didn't bother to use the current sensing circuitry, possibly because
of these glitches.

This makes me conclude that in order to avoid annoying false stall detection and position mismatches it should be safe to 
disable the current sensing feature and to rely solely on the Hall sensor timeout (296 ms by default). After all, the current sensing is an extra feature not implemented originally by the manufacture. Needless to say, this kind of software feature, even if enabled, doesn't prevent from possible programming errors or physical catastrofic failures. This it's mandatory that other precautions are made to prevent too high current consumption in case of actual faulty operation.  Most (if not all) power adapters shut down the output if their maximum current draw is exceeded. The original Fyrtur board doesn't have any kind of fuse, but the Li-Ion battery modules are specified to have maximum 2A output. So I would like to assume that they have an internal current limiting / protection circuitry, BUT I HAVEN'T TESTED THEIR SAFETY WHEN SHORT CIRCUITED NOR DO I WANT TO!  

So, from version 0.83 and onwards the current sensing feature is disabled, but can be re-enabled if needed
via CMD_EXT_SET_MAX_MOTOR_CURRENT command.
*/
#define DEFAULT_MAX_MOTOR_CURRENT 0 // in mA.  0 == DISABLED

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
