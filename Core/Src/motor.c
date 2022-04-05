/*
 * motor.c
 *
 *  Created on: Aug 29, 2020
 *      Author: Marko Juhanne
 */
#include "main.h"
#include "motor.h"
#include "eeprom.h"
#include "bootloader.h"
#include "stdlib.h" // abs function

extern uint8_t blink;

motor_status_t status;
motor_direction_t direction;

/* LOCATION is the spatial position of the curtain measured in motor revolutions. Due to additional gear mechanism, it takes
 * GEAR_RATIO revolutions in order to actually reach 1 full revolution of the curtain rod. Motor revolution is detected by HALL sensor,
 * which generates 4 interrupts (ticks) per motor revolution.
 *
 * POSITION itself is a measure of curtain position reported by float between 0.0 (fully closed) and 100.0 (fully open) and can be
 * calculated from LOCATION with location_to_position100 (and vice versa with position100_to_location).
 *
 * Maximum POSITION is affected by user-customizable max curtain length (configured via CMD_SET_MAX_CURTAIN_LENGTH). In addition to this,
 * there is the "absolute" limit of full (factory defined) curtain length. However these limits can be ignored using CMD_OVERRIDE_XXX move commands and also be
 * re-configured with CMD_SET_MAX_CURTAIN_LENGTH / CMD_SET_FULL_CURTAIN_LENGTH commands.
 */
#define DEG_TO_LOCATION(x) (GEAR_RATIO * 4 * x / 360)
int32_t target_location = 0;
int32_t location = 0;

uint8_t orientation = DEFAULT_ORIENTATION;

uint32_t full_curtain_length = DEFAULT_FULL_CURTAIN_LEN;
uint32_t max_curtain_length;

uint16_t minimum_voltage;	// value is minimum voltage (in Volts) * 16 (float stored as interger value)
uint32_t idle_mode_sleep_delay;

/* the motor driver gate PWM duty cycle is initially 60/255 when first energized and then adjusted according to target_speed */
#define INITIAL_PWM 60

uint8_t default_speed;
uint8_t target_speed = 0; // target RPM
uint8_t curr_pwm = 0;  // PWM setting
uint8_t pwm_when_stalled = 0; // for debugging

/*
* This flag is set when a new signal is received from Hall sensor and we can update the duty cycle.
* Used only when slowing down
* (speeding up must be done without waiting for this signal because otherwise we could end up stalling the motor prematurely)
*/
uint8_t rpm_updated = 0;

uint16_t max_motor_current = DEFAULT_MAX_MOTOR_CURRENT;
uint16_t highest_motor_current = 0;	// for statistics

/*
 * When doing calibration the curtain rod is rotated upwards to highest position until motor stalls. The next phase is the endpoint calibration, when the motor is de-energized,
 * which causes the rod to rotate slightly downwards due to curtain tension. We must wait a bit before tension is released and curtain rod
 * is settled in order to correct for the downwards movement. After this time period the motor is considered to be in top position (location = 0)
 */
uint32_t endpoint_calibration_started_timestamp = 0;

/*
 * When calibrating we allow unrestricted movement until calibration procedure is done
 */
uint8_t calibrating = 0;

uint8_t auto_calibration; // If enabled, auto-calibration will roll up the blinds during power up in order to calibrate top curtain position. Enabled by default

/*
 * count how many milliseconds since previous HALL sensor #1 interrupt occurred
 * in order to calculate hall_sensor_interval (and RPM) and also to detect motor stalling
 */
uint32_t hall_sensor_1_idle_time = 0;

uint32_t hall_sensor_1_ticks = 0;	// how many hall sensor #1 ticks(signals) after movement
uint32_t hall_sensor_2_ticks = 0;	// how many hall sensor #2 ticks(signals) after movement

uint32_t hall_sensor_1_interval = 0; // how many milliseconds between Hall sensor #1 ticks

/*
 * Used for stall detection grace period
 * Motor is given some time to gather speed by increasing PWM duty cycle before applying stall detection
 */
uint32_t movement_started_timestamp = 0;

uint16_t stall_detection_timeout = DEFAULT_STALL_DETECTION_TIMEOUT;

int rotor_position = -1;

uint8_t min_slowdown_speed = DEFAULT_MINIMUM_SLOWDOWN_SPEED;
uint8_t	slowdown_factor = DEFAULT_SLOWDOWN_FACTOR;

uint16_t last_stalling_current = 0;

motor_command_t command; // for deferring execution to main loop since we don't want to invoke HAL_Delay in UARTinterrupt handler
motor_error_t last_error;

// --- Flexi-speed parameters
uint8_t flexispeed_sel = 0;	// selected setting 
uint8_t flexispeed_settings[] = { 3, 5, 15, 25 };	// speed settings in RPM
// keeps track how many times CMD_UP is called repeatedly and if past FLEXISPEED_TRIGGER_LIMIT, then cycle to next speed setting
uint8_t flexispeed_trigger_counter;	
uint16_t last_command; // the last command issued (except CMD_GET_STATUS)

// for debugging
int dir_error = 0;
int sensor_ticks_while_stopped = 0;
int sensor_ticks_while_calibrating_endpoint = 0;

// ----- Commands supported also by original Fyrtur module -----

// commands with 1 parameter
#define CMD_GO_TO	0xdd

// commands without parameter
#define CMD_UP 		0x0add
#define CMD_DOWN 	0x0aee
#define CMD_UP_17 	0x0a0d
#define CMD_DOWN_17	0x0a0e
#define CMD_STOP	0x0acc

#define CMD_OVERRIDE_UP_90		0xfad1
#define CMD_OVERRIDE_DOWN_90	0xfad2
#define CMD_OVERRIDE_UP_6		0xfad3
#define CMD_OVERRIDE_DOWN_6		0xfad4
#define CMD_SET_MAX_CURTAIN_LENGTH	0xfaee	// will be stored to flash memory
#define CMD_SET_FULL_CURTAIN_LENGTH	0xfacc	// will be stored to flash memory
#define CMD_RESET_CURTAIN_LENGTH	0xfa00	// reset maximum curtain length to factory setting (full curtain length). New value is stored to flash memory

#define CMD_TOGGLE_ORIENTATION	0xd600	// Toggle blinds orientation between back-roll and front-roll
#define CMD_RESET_ORIENTATION	0xd500	// Reset blinds orientation back to standard back-roll

#define CMD_GET_STATUS 	0xcccc
#define CMD_GET_STATUS2 0xcccd
#define CMD_GET_STATUS3 0xccce
#define CMD_GET_STATUS4 0xccdd

// ------ Commands supported only by our custom firmware -------

// commands with 1 parameter
#define CMD_EXT_GO_TO					0x10	// target position is the lower 4 bits of the 1st byte + 2nd byte (12 bits of granularity), where lower 4 bits is the decimal part
#define CMD_EXT_SET_SPEED 				0x20	// setting speed via this command will not alter non-volatile memory (so it's safe for limited write-cycle flash memory)
#define CMD_EXT_SET_DEFAULT_SPEED 		0x30	// default speed will be stored to flash memory
#define CMD_EXT_SET_MINIMUM_VOLTAGE		0x40	// minimum voltage. Will be stored to flash memory
#define CMD_EXT_SET_LOCATION			0x50	// location is the lower 4 bits of the 1st byte + 2nd byte (1 sign bit + 11 bits of integer part)
#define CMD_EXT_SET_AUTO_CAL			0x60	// If enabled, auto-calibration will roll up the blinds during power up in order to calibrate top curtain position. Enabled by default
#define CMD_EXT_SET_ORIENTATION			0x61	// Sets curtain orientation (affects motor direction). (0 = normal orientation, default. 1 = reverse)
#define CMD_EXT_SET_MAX_MOTOR_CURRENT 	0x62	// Set max motor current (current is 2nd byte * 16, measured in mA. Maximum value is 0xff = 4A)
#define CMD_EXT_SET_STALL_DETECTION_TIMEOUT 0x63	// Set hall sensor timeout (to detect motor stalling). (value is 2nd byte * 8, measured in milliseconds)
#define CMD_EXT_SET_SLEEP_DELAY 		0x64	// The delay (in ms) after which sleep mode is entered when motor is idle. (value is 2nd byte * 256, measured in milliseconds. 0 = sleep mode is disabled)
#define CMD_EXT_GO_TO_LOCATION			0x70	// Go to target location (measured in Hall sensor ticks). Location is the lower 4 bits of the 1st byte + 2nd byte
#define CMD_EXT_SET_SLOWDOWN_FACTOR 	0x80	// Set slowdown factor
#define CMD_EXT_SET_MIN_SLOWDOWN_SPEED	0x90	// Set minimum approach speed

// Ping function for debugging/testing. If 1st parameter is 0 then blink internal LED (not the one on WiFi module!). 
// Otherwise send back ping message with the 1st parameter incremented by 1.
#define CMD_EXT_PING					0xa0	

// commands without parameter
#define CMD_EXT_OVERRIDE_DOWN		0xfada	// Continous move down ignoring the max/full curtain length. Maximum movement of 5 revolutions per command
#define CMD_EXT_GET_LOCATION 		0xccd0
#define CMD_EXT_GET_VERSION 		0xccdc
#define CMD_EXT_GET_STATUS 			0xccde
#define CMD_EXT_GET_TUNING_PARAMS 	0xccd3
#define CMD_EXT_GET_LIMITS 			0xccdf
#define CMD_EXT_DEBUG	 			0xccd1
#define CMD_EXT_SENSOR_DEBUG 		0xccd2
#define CMD_EXT_ENTER_BOOTLOADER	0xff00

// Steps for "acknowledge dance"
motor_command_t dance_steps_up_down[] = {
	MotorUp,
	Stop,
	MotorDown,
	NoCommand
};

motor_command_t dance_steps_down_up[] = {
	MotorDown,
	Stop,
	MotorUp,
	NoCommand
};

int dance_step_pos;
motor_command_t * dance_steps;

/****************** EEPROM variables ********************/

typedef enum eeprom_var_t {
	MAX_CURTAIN_LEN_EEPROM = 0,
	FULL_CURTAIN_LEN_EEPROM = 1,
	MINIMUM_VOLTAGE_EEPROM = 2,
	DEFAULT_SPEED_EEPROM = 3,
	AUTO_CAL_EEPROM = 4,
	ORIENTATION_EEPROM = 5,
	MAX_MOTOR_CURRENT_EEPROM = 6,
	STALL_DETECTION_TIMEOUT_EEPROM = 7,
	IDLE_MODE_SLEEP_DELAY_EEPROM = 8
} eeprom_var_t;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888, 0x9999, 0xAAAA, 0xBBB1, 0xCCCC, 0xDDDD};


void motor_set_default_settings() {
	max_curtain_length = DEFAULT_FULL_CURTAIN_LEN; // by default, max_curtain_length is full_curtain_length
	full_curtain_length = DEFAULT_FULL_CURTAIN_LEN;
	minimum_voltage = DEFAULT_MINIMUM_VOLTAGE;
	default_speed = DEFAULT_TARGET_SPEED;
	auto_calibration = DEFAULT_AUTO_CAL_SETTING;
	orientation = DEFAULT_ORIENTATION;
	max_motor_current = DEFAULT_MAX_MOTOR_CURRENT;
	stall_detection_timeout = DEFAULT_STALL_DETECTION_TIMEOUT;
	idle_mode_sleep_delay = DEFAULT_IDLE_MODE_SLEEP_DELAY;
}

#ifndef SLIM_BINARY
void motor_load_settings() {
	uint16_t tmp;
	if (EE_ReadVariable(VirtAddVarTab[MAX_CURTAIN_LEN_EEPROM], &tmp) != 0) {
		tmp = max_curtain_length = DEFAULT_FULL_CURTAIN_LEN;	// by default, max_curtain_length is full_curtain_length
		EE_WriteVariable(VirtAddVarTab[FULL_CURTAIN_LEN_EEPROM], tmp);
	} else {
		max_curtain_length = tmp;
	}
	if (EE_ReadVariable(VirtAddVarTab[FULL_CURTAIN_LEN_EEPROM], &tmp) != 0) {
		tmp = full_curtain_length = DEFAULT_FULL_CURTAIN_LEN;
		EE_WriteVariable(VirtAddVarTab[FULL_CURTAIN_LEN_EEPROM], tmp);
	} else {
		full_curtain_length = tmp;
	}
	if (EE_ReadVariable(VirtAddVarTab[MINIMUM_VOLTAGE_EEPROM], &tmp) != 0) {
		minimum_voltage = DEFAULT_MINIMUM_VOLTAGE;
		tmp = minimum_voltage;
		EE_WriteVariable(VirtAddVarTab[MINIMUM_VOLTAGE_EEPROM], tmp);
	} else {
		minimum_voltage = tmp;
	}
#ifdef READ_DEFAULT_SPEED_FROM_EEPROM
	if (EE_ReadVariable(VirtAddVarTab[DEFAULT_SPEED_EEPROM], &tmp) != 0) {
		tmp = default_speed = DEFAULT_TARGET_SPEED;
		EE_WriteVariable(VirtAddVarTab[DEFAULT_SPEED_EEPROM], tmp);
	} else {
		default_speed = tmp;
	}
#else
	default_speed = DEFAULT_TARGET_SPEED;
#endif
	if (EE_ReadVariable(VirtAddVarTab[AUTO_CAL_EEPROM], &tmp) != 0) {
		auto_calibration = DEFAULT_AUTO_CAL_SETTING;
		tmp = auto_calibration;
		EE_WriteVariable(VirtAddVarTab[AUTO_CAL_EEPROM], tmp);
	} else {
		auto_calibration = tmp;
	}
	if (EE_ReadVariable(VirtAddVarTab[ORIENTATION_EEPROM], &tmp) != 0) {
		orientation = DEFAULT_ORIENTATION;
		tmp = orientation;
		EE_WriteVariable(VirtAddVarTab[ORIENTATION_EEPROM], tmp);
	} else {
		orientation = tmp;
	}
	if (EE_ReadVariable(VirtAddVarTab[MAX_MOTOR_CURRENT_EEPROM], &tmp) != 0) {
		max_motor_current = DEFAULT_MAX_MOTOR_CURRENT;
		tmp = max_motor_current;
		EE_WriteVariable(VirtAddVarTab[MAX_MOTOR_CURRENT_EEPROM], tmp);
	} else {
		max_motor_current = tmp;
	}
	if (EE_ReadVariable(VirtAddVarTab[STALL_DETECTION_TIMEOUT_EEPROM], &tmp) != 0) {
		stall_detection_timeout = DEFAULT_STALL_DETECTION_TIMEOUT;
		tmp = stall_detection_timeout;
		EE_WriteVariable(VirtAddVarTab[STALL_DETECTION_TIMEOUT_EEPROM], tmp);
	} else {
		stall_detection_timeout = tmp;
	}
	if (EE_ReadVariable(VirtAddVarTab[IDLE_MODE_SLEEP_DELAY_EEPROM], &tmp) != 0) {
		idle_mode_sleep_delay = DEFAULT_IDLE_MODE_SLEEP_DELAY;
		tmp = idle_mode_sleep_delay;
		EE_WriteVariable(VirtAddVarTab[IDLE_MODE_SLEEP_DELAY_EEPROM], tmp);
	} else {
		idle_mode_sleep_delay = tmp;
	}
}
#endif

void motor_write_setting( eeprom_var_t var, uint16_t value ) {
#ifndef SLIM_BINARY
	uint16_t tmp;
	if (status == Stopped) {
		// motor has to be stopped to change non-volatile settings (writing to FLASH should occur uninterrupted)
		EE_ReadVariable(VirtAddVarTab[var], &tmp);
		if (tmp != value) {
			EE_WriteVariable(VirtAddVarTab[var], value);
		}
	}
#endif
}

uint32_t position100_to_location( float position ) {
	if (position > 100)
		return max_curtain_length;
	return position*max_curtain_length/100;
}


float location_to_position100() {
	if (calibrating) {
		// When calibrating we ignore our position and return 50% instead
		return 50;
	}
	if (location < 0) {	// don't reveal positions higher than top position (should not happen if calibrated correctly)
		return 0;
	}
	if  (status == CalibratingEndPoint) {
		return 0;	// we don't publish the unstable position when doing top limit calibration
	}
	if (location > max_curtain_length) {
		return 100;
	}
	return 100*(float)location / max_curtain_length;
}


float get_rpm() {
	float rpm = 0;
	if (hall_sensor_1_interval) {
		// 60000 ms in minute
		// 2 hall sensor #1 interrupts per motor revolution
		// GEAR_RATIO motor revolutions per curtain rod revolution
		rpm = 60*1000.0/GEAR_RATIO/hall_sensor_1_interval/2;
	}
	return rpm;
}


/*
 * This function adjusts location when the curtain rod is rotated by motor AS WELL AS by passive movement.
 * Movement is ignored only during calibrating since we are rolling upwards against hard-stop (and to location 0) anyway.
 */
int process_sensor(motor_direction_t sensor_direction) {
	if (!calibrating) {
		if (sensor_direction == Up) {
			location--;
			if (direction == Up) {
				if (target_location != -1) {	// if target is -1, force movement up until the motor stalls which causes calibration
					if (location - 1 <= target_location) {	// stop just before the target
						motor_stop();
						return 1;
					}
				}
			}
		} else if (sensor_direction == Down) {
			location++;
			if (direction == Down) {
				if(location + 1 >= target_location) { // stop just before the target
					motor_stop();
					return 1;
				}
			}
		}

		// If motor is rotating, slow it down when approaching the target location
		if ( (direction != None) && (target_location != -1) ) {
			int distance_to_target = abs(target_location - location);
			if (distance_to_target < target_speed * slowdown_factor /8) {
				status = Stopping;
				int new_speed = distance_to_target*8/slowdown_factor;
				if (new_speed < min_slowdown_speed)
					new_speed = min_slowdown_speed; // minimum approach speed
				if (new_speed < target_speed)
					target_speed = new_speed;
			}
		}
	}
	return 0;
}


/*
 * Hall sensors will create following interrupts:
 * Upwards movement: HALL1 HIGH, HALL2 HIGH, HALL1 LOW, HALL2 LOW
 * Downwards movement: HALL2 HIGH, HALL1 HIGH, HALL2 LOW, HALL1 LOW
 */
void hall_sensor_callback( uint8_t sensor, uint8_t value ) {

	// This calculation will give following values for rotor_position:
	// Upwards movement: ..., 0, 1, 2, 3, 0, 1, 2, 3, 0, ...
	// Downwards movement: ..., 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, ...
	// Note that changing direction will "skip" 1 position:
	// 	e.g. HALL2_HIGH -> HALL1_LOW -> stop and change direction -> HALL1_HIGH -> HALL2_LOW will translate to:
	//	1, 2, (stop & change dir), 0, 3, ...
	// Also note that if orientation == REVERSE_ORIENTATION, the movement direction is reversed
	int new_rotor_position = sensor + (1-value)*2;

	if (sensor == HALL_1_SENSOR) {
		hall_sensor_1_ticks++;
		if (hall_sensor_1_ticks > 1) {
			// At least two sensor ticks are needed to calculate interval correctly
			hall_sensor_1_interval = hall_sensor_1_idle_time;	// update time passed between hall sensor interrupts
			rpm_updated = 1;
		}
		hall_sensor_1_idle_time = 0;
	} else {
		hall_sensor_2_ticks++;
	}

	// save for debugging
	if (status == Stopped) {
		sensor_ticks_while_stopped++;
	} else if (status == CalibratingEndPoint) {
		sensor_ticks_while_calibrating_endpoint++;
	}


	if (rotor_position != -1) {
		int diff = (4 + new_rotor_position - rotor_position) & 0x3;
		if ( ((diff == 1) && (orientation == NORMAL_ORIENTATION)) ||
				 ((diff == 3) && (orientation == REVERSE_ORIENTATION)) ) {
			// Sensor direction is UP

			if (direction != Down) {
				// Process Up movement while motor is rotating upwards or disengaged
				process_sensor(Up);
			} else if (direction == Down ) {
				// Mismatched direction between sensor and motor.
				dir_error++;
			}
		} else if ( ((diff == 3) && (orientation == NORMAL_ORIENTATION)) ||
					 ((diff == 1) && (orientation == REVERSE_ORIENTATION)) ) {
			// Sensor direction is DOWN

			if (direction != Up) {
				// Process Down movement while motor is rotating downwards or disengaged
				process_sensor(Down);
			} else if (direction == Up) {
				// Mismatched direction between sensor and motor.
				dir_error++;
			}
		} else {
			// Change of direction. Don't need to adjust location(?)
			//process_sensor(direction);
		}
	}

	rotor_position = new_rotor_position;
}

void update_motor_pwm() {
	if ( ((direction == Up) && (orientation == NORMAL_ORIENTATION)) ||
			 ((direction == Down) && (orientation == REVERSE_ORIENTATION)) ) {
		TIM1->CCR4 = curr_pwm;
	} else if ( ((direction == Down) && (orientation == NORMAL_ORIENTATION)) ||
			 ((direction == Up) && (orientation == REVERSE_ORIENTATION)) ) {
		TIM1->CCR1 = curr_pwm;
	}
}

/* Called every 10ms by TIM3 */
void motor_adjust_rpm() {
	if ((status == Moving) || (status == Stopping)) {
		float speed = get_rpm();
		if (speed < target_speed) {
			if (curr_pwm < 254) {
				curr_pwm++;
				if (target_speed - speed > 2)	// additional acceleration if speed difference is greater
					curr_pwm++;
				update_motor_pwm();
			}
		}

		// Lower RPM only if new tick from Hall sensor has been received
		if (rpm_updated) {
			if (speed > target_speed) {
				rpm_updated = 0;
				if (curr_pwm > 1) {
					curr_pwm--;
					if (speed - target_speed > 2)	// additional deceleration if speed difference is greater
						curr_pwm--;
					if (speed - target_speed > 4)	// additional deceleration if speed difference is greater
						curr_pwm--;
					update_motor_pwm();
				}
			}
		}
	}
}


/*
 * This is periodically (every 1 millisecond) called by SysTick_Handler
 */
void motor_stall_check() {
	if ( (status == Moving) || (status == Stopping) ) {
		// Count how many milliseconds since previous HALL sensor interrupt
		// in order to calculate RPM and detect motor stalling
		hall_sensor_1_idle_time ++;
		if (HAL_GetTick() - movement_started_timestamp > HALL_SENSOR_TIMEOUT_WHILE_STARTING) {
			// enough time has passed since motor is energized -> apply stall detection

			if (hall_sensor_1_idle_time > stall_detection_timeout) {
				// motor has stalled/stopped

				if ( (status == Stopping) && (hall_sensor_1_idle_time < HALL_SENSOR_TIMEOUT_WHILE_STOPPING) ) {
					// when slowing down, allow longer time to recover from premature stalling
				} else {
					motor_stopped();
				}
			}
		}
		uint16_t curr = get_motor_current();
		if (curr > highest_motor_current) {
			highest_motor_current = curr;
		}
		if (max_motor_current != 0) {
			if ( (curr > max_motor_current) && (hall_sensor_1_ticks > MIN_SENSOR_TICKS) ) {
				// maximum current limit exceeded while moving -> motor has stalled. Note that because static friction
				// can be quite high, we do not enforce the current limit if motor hasn't turned yet (no Hall sensor signals received)
				motor_stopped();
			}
		}
	} else if (status == CalibratingEndPoint) {
		if (HAL_GetTick() - endpoint_calibration_started_timestamp > ENDPOINT_CALIBRATION_PERIOD) {
			// Calibration is done and we are at top position
			status = Stopped;
			calibrating = 0;	// Limits will be enforced from now on
			location = 0;
		}
	}
}

void motor_stopped() {
	if (status != Stopped) {
		// motor has stalled!

		motor_status_t current_status = status;
		motor_direction_t current_direction = direction;
		uint16_t motor_current = get_motor_current();
		pwm_when_stalled = curr_pwm;

		// De-energize the motor
		motor_stop();

		last_stalling_current = get_motor_current();

		if (current_status == Moving)  {
			if (hall_sensor_1_ticks > MIN_SENSOR_TICKS) {

				if (current_direction == Up) {
					if (motor_current > MINIMUM_CALIBRATION_CURRENT) {
						// The motor has stalled with enough resistance (motor current) so we assume that we have reached 
						// the top position. Now remaining is the endpoint calibration
						// (adjusting for the backward movement because of curtain tension)
						status = CalibratingEndPoint;
						sensor_ticks_while_calibrating_endpoint = 0;	// for debugging
						blink += 2;
						// now we wait until curtain rod stabilizes				
						endpoint_calibration_started_timestamp = HAL_GetTick();
					} else {
						// The motor has stalled but without too much resistance (motor current is low)
						// This suggests that during low speed (usually RPM < 5) movement there was sudden increase in friction
						// which caused the motor to stall. However the RPM adjustment function didn't respond quickly
						// enough to ramp up the PWM duty cycle before Hall sensor timeout kicked in. 
						// See MINIMUM_CALIBRATION_CURRENT definition
						last_error = StalledMovingUp;
						status = Error;
						blink += 4;
					}
				} else {
					// motor should not stall when direction is down!
					last_error = StalledMovingDown;
					status = Error;
					blink += 4;
				}
			} else {
				// motor has not even moved! Faulty Hall sensor or too much static friction 
				last_error = FrictionError;
				status = Error;
				blink += 3;
			}
		} else if (current_status == Stopping) {
			// Motor was accidently stalled during slowing down
			status = Stopped;
		}
	}
}


void motor_stop() {

	// Make sure that all mosfets are off
	pwm_stop(LOW1_PWM_CHANNEL);
	pwm_stop(LOW2_PWM_CHANNEL);
	/*
	 * Remember to double-check that the code generated in HAL_TIM_MspPostInit by CubeMX has GPIO_PULLDOWN setting
	 * enabled for LOW_1_GATE_Pin and LOW_2_GATE_Pin !!
	 */

	HAL_GPIO_WritePin(HIGH_1_GATE_GPIO_Port, HIGH_1_GATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_RESET);
	TIM1->CCR1 = 0;
	TIM1->CCR4 = 0;
	status = Stopped;
	last_error = NoError;
	direction = None;
	curr_pwm = 0;

	// for debugging
	sensor_ticks_while_stopped = 0;

	// reset stall detection timeout
	hall_sensor_1_interval = 0;
	hall_sensor_1_idle_time = 0;
	target_speed = 0;
}


void motor_start_common(uint8_t motor_speed) {
	if (status == Moving) {
		motor_stop();	// stop the motor and wait for the movement to stop
		HAL_Delay(500);
	} else {
		motor_stop();	// first reset all the settings just in case..
		HAL_Delay(10);
	}
	blink += 1;
	movement_started_timestamp = HAL_GetTick();
	highest_motor_current = 0; // clear previous record
	target_speed = motor_speed;
	curr_pwm = INITIAL_PWM;
	status = Moving;
	hall_sensor_1_ticks = 0;
	hall_sensor_2_ticks = 0;
	disable_sleep_timer();
}

void motor_up(uint8_t motor_speed) {

	motor_start_common(motor_speed);

	direction = Up;
	update_motor_pwm();
	if (orientation == NORMAL_ORIENTATION) {
		// turn on LOW2 PWM and HIGH1
		pwm_start(LOW2_PWM_CHANNEL);
		HAL_GPIO_WritePin(HIGH_1_GATE_GPIO_Port, HIGH_1_GATE_Pin, GPIO_PIN_SET);
	} else {
		// turn on LOW1 PWM and HIGH2
		pwm_start(LOW1_PWM_CHANNEL);
		HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_SET);
	}
}

void motor_down(uint8_t motor_speed) {

	motor_start_common(motor_speed);

	direction = Down;
	update_motor_pwm();
	if (orientation == NORMAL_ORIENTATION) {
		// turn on LOW1 PWM and HIGH2
		pwm_start(LOW1_PWM_CHANNEL);
		HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_SET);
	} else {
		// turn on LOW2 PWM and HIGH1
		pwm_start(LOW2_PWM_CHANNEL);
		HAL_GPIO_WritePin(HIGH_1_GATE_GPIO_Port, HIGH_1_GATE_Pin, GPIO_PIN_SET);
	}
}


#ifndef SLIM_BINARY
uint8_t check_voltage() {
	if (minimum_voltage != 0) {
		uint16_t voltage = get_voltage() / 30;
		if (voltage < minimum_voltage)
			return 0;
	}
	return 1;
}
#endif

void process_next_command( motor_command_t next_command ) {
	if (next_command == MotorUp) {
		if (check_voltage()) {
			motor_up(default_speed);
		}
	} else if (next_command == MotorDown) {
		if (check_voltage()) {
			motor_down(default_speed);
		}
	} else if (next_command == Stop) {
		motor_stop();
	} else if( next_command == EnterBootloader) {
		motor_stop();
		// Wait until all UART TX is done
		while (!uart_tx_done()) {
		}
		reset_to_bootloader();
		// .. not reached
	}
}

void dance() {
	// Move the blinds up/down or down/up a bit, depending on whether the blinds are almost down or up
	// (so not to cause calibration or to extend past lower limit)
	dance_step_pos = 0;
	command = Dance;
	if (location < (max_curtain_length/2)) {
		dance_steps = dance_steps_down_up;
	} else {
		dance_steps = dance_steps_up_down;
	}
}

void do_dance() {
	if (status == Stopped) {
		if (dance_steps[dance_step_pos] == NoCommand) {
			// end of dance
			command = NoCommand;
		} else {
			// do the next step
			if (dance_steps[dance_step_pos] == MotorUp) {
				target_location -= DANCE_STEP_LENGTH;
			} else if (dance_steps[dance_step_pos] == MotorDown) {
				target_location += DANCE_STEP_LENGTH;
			}
			process_next_command( dance_steps[dance_step_pos] );
		}
		dance_step_pos++;
	} else if (status == Error) {
		command = NoCommand;
	}
}

uint8_t check_flexispeed_trigger() {
#ifdef FLEXISPEED_ENABLED
	if (last_command == CMD_UP) {
		flexispeed_trigger_counter++;
		if ( flexispeed_trigger_counter >= FLEXISPEED_TRIGGER_LIMIT -1) {
			// Speed switching is triggered
			flexispeed_trigger_counter = 0;
			flexispeed_sel++;
			if (flexispeed_sel >= sizeof(flexispeed_settings)) {
				flexispeed_sel = 0;
			}
			uint8_t speed = flexispeed_settings[ flexispeed_sel ];
			motor_write_setting(DEFAULT_SPEED_EEPROM, speed);
			default_speed = speed;
			dance();
			return 1;
		}
	}
#endif
	return 0;
}

void motor_process() {
	if (command == Dance) {
		do_dance();
	} else if (command != NoCommand) {
		process_next_command(command);
		command = NoCommand;
	}
	if (idle_mode_sleep_delay > 0) {
		if ( (status == Stopped) || (status == Error) ) {
			if (!sleep_timer_enabled()) {
				reset_sleep_timer();
			} else {
				if (sleep_timer_timeout()) {
					disable_sleep_timer();
					enter_sleep_mode();
				}
			}
		}
	}
}

#define VOLTAGE_TO_INT(x) (int)(x*30*16)

// Very rudimentary battery/voltage curve
uint8_t calculate_battery() {
	if (get_voltage() < VOLTAGE_TO_INT(6.5))
		return 0x00;
	if (get_voltage() <= VOLTAGE_TO_INT(7.0))
		return 0x00 + (get_voltage() - VOLTAGE_TO_INT(6.5)) * (0x12 - 0x00) / (VOLTAGE_TO_INT(7.0) - VOLTAGE_TO_INT(6.5));
	if (get_voltage() <= VOLTAGE_TO_INT(7.2))
		return 0x12 + (get_voltage() - VOLTAGE_TO_INT(7.0)) * (0x2e - 0x12) / (VOLTAGE_TO_INT(7.2) - VOLTAGE_TO_INT(7.0));
	if (get_voltage() <= VOLTAGE_TO_INT(7.5))
		return 0x2e + (get_voltage() - VOLTAGE_TO_INT(7.2)) * (0x45 - 0x2e) / (VOLTAGE_TO_INT(7.5) - VOLTAGE_TO_INT(7.2));
	return 0x45;
}


uint8_t handle_command(uint8_t * rx_buffer, uint8_t * tx_buffer, uint8_t * tx_bytes) {
	uint8_t cmd1, cmd2;
	cmd1 = rx_buffer[3];
	cmd2 = rx_buffer[4];
	uint16_t cmd = (cmd1 << 8) + cmd2;

	uint8_t cmd_handled = 1;

	if (sleep_timer_enabled()) {
		reset_sleep_timer();
	}

	switch (cmd) {

		case CMD_GET_STATUS:
			{
				tx_buffer[2] = 0xd8;
				tx_buffer[3] = calculate_battery();
				tx_buffer[4] = (uint8_t)( get_voltage()/16);  // returned value in the message is Volts * 30 as in original FW
				float rpm = get_rpm();
				if ( (rpm < 1) && ( (status == Moving) || (status == Stopping) || (status == CalibratingEndPoint) )) {
					// If speed is so slow that it's (almost) stalling or we in the middle of end-point calibration,
					// report a minimal speed anyway so that controller module knows that we are not finished yet
					rpm = 1;
				}
				tx_buffer[5] = (uint8_t)rpm;
				tx_buffer[6] = location_to_position100();
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;

		case CMD_UP:
			{
				if (!check_flexispeed_trigger()) {
					// Normal CMD_UP
					target_location = -1;	// motor goes up until it stalls which forces calibration
					command = MotorUp;
				}
			}
			break;

		case CMD_DOWN:
			{
				target_location = max_curtain_length;
				command = MotorDown;
			}
			break;

		case CMD_UP_17:
			{
				target_location = location - DEG_TO_LOCATION(17);
				if (target_location < 0)
					target_location = 0;
				command = MotorUp;
			}
			break;

		case CMD_DOWN_17:
			{
				target_location = location + DEG_TO_LOCATION(17);
				if (target_location > max_curtain_length)
					target_location = max_curtain_length;
				command = MotorDown;
			}
			break;

		case CMD_STOP:
			{
				command = Stop;
			}
			break;

		case CMD_OVERRIDE_UP_90:
			{
				target_location = location - DEG_TO_LOCATION(90);
				command = MotorUp;
			}
			break;

		case CMD_OVERRIDE_DOWN_90:
			{
				target_location = location + DEG_TO_LOCATION(90);
				command = MotorDown;
			}
			break;

		case CMD_OVERRIDE_UP_6:
			{
				target_location = location - DEG_TO_LOCATION(6);
				command = MotorUp;
			}
			break;

		case CMD_OVERRIDE_DOWN_6:
			{
				target_location = location + DEG_TO_LOCATION(6);
				command = MotorDown;
			}
			break;
		case CMD_SET_FULL_CURTAIN_LENGTH:
			{
				motor_write_setting(FULL_CURTAIN_LEN_EEPROM, location);
				full_curtain_length = location;
			}
			// fall-through: maximum curtain length will be reset as well

		case CMD_SET_MAX_CURTAIN_LENGTH:
			{
				motor_write_setting(MAX_CURTAIN_LEN_EEPROM, location);
				max_curtain_length = location;
				dance();
			}
			break;

		case CMD_RESET_CURTAIN_LENGTH:
			{
				motor_write_setting(MAX_CURTAIN_LEN_EEPROM, full_curtain_length);
				max_curtain_length = full_curtain_length;
				calibrating = 1;	// allow unrestricted movement until the end of calibration
			}
			break;
		case CMD_TOGGLE_ORIENTATION:
			{
				orientation = (orientation+1)&1;
				motor_write_setting(ORIENTATION_EEPROM, orientation);
				location = full_curtain_length - location;
			}
			break;
		case CMD_RESET_ORIENTATION:
			{
				if (orientation == 1) {
					orientation = 0;
					motor_write_setting(ORIENTATION_EEPROM, orientation);
					location = full_curtain_length - location;
				}
			}
			break;
		case CMD_EXT_OVERRIDE_DOWN:
			{
				target_location = location + DEG_TO_LOCATION(360*5);
				command = MotorDown;
			}
			break;
		case CMD_EXT_GET_VERSION:
			{
				tx_buffer[2] = 0xd0;
				tx_buffer[3] = VERSION_MAJOR;
				tx_buffer[4] = VERSION_MINOR;
				tx_buffer[5] = minimum_voltage;
				tx_buffer[6] = default_speed;
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;
		case CMD_EXT_GET_TUNING_PARAMS:
			{
				tx_buffer[2] = 0xd5;
				tx_buffer[3] = slowdown_factor;
				tx_buffer[4] = min_slowdown_speed;
				tx_buffer[5] = stall_detection_timeout/8;
				tx_buffer[6] = max_motor_current/16;
				int16_t curr = last_stalling_current/16;
				if (curr>255) {
					curr = 255; // maximum reported value is 4 amps
				}
				tx_buffer[7] = curr;
				tx_buffer[8] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7];
				*tx_bytes=9;
			}
			break;
		case CMD_EXT_DEBUG:
			{
				tx_buffer[2] = 0xd2;
				tx_buffer[3] = (uint8_t)last_error;
				tx_buffer[4] = (uint8_t)(idle_mode_sleep_delay/256);
				tx_buffer[5] = (uint8_t)sensor_ticks_while_calibrating_endpoint;
				tx_buffer[6] = (uint8_t)sensor_ticks_while_stopped;
				uint16_t curr = highest_motor_current / 16;
				if (curr > 255) {
					curr = 255;	// maximum reported value is 4 amps
				}
				tx_buffer[7] = curr;
				tx_buffer[8] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7];
				*tx_bytes=9;
			}
			break;
		case CMD_EXT_SENSOR_DEBUG:
			{
				tx_buffer[2] = 0xd3;
				tx_buffer[3] = hall_sensor_1_ticks >> 8;
				tx_buffer[4] = hall_sensor_1_ticks & 0xff;
				tx_buffer[5] = hall_sensor_2_ticks >> 8;
				tx_buffer[6] = hall_sensor_2_ticks & 0xff;
				tx_buffer[7] = pwm_when_stalled;
				tx_buffer[8] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7];
				*tx_bytes=9;
			}
			break;
		case CMD_EXT_GET_LOCATION:
			{
				tx_buffer[2] = 0xd1;
				tx_buffer[3] = location >> 8;
				tx_buffer[4] = location & 0xff;
				tx_buffer[5] = target_location >> 8;
				tx_buffer[6] = target_location & 0xff;
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;
		case CMD_EXT_ENTER_BOOTLOADER:
			{
				command = EnterBootloader;
				status = Bootloader;
			}
			// fall-thru (send 'entering bootloader' status)
		case CMD_EXT_GET_STATUS:
			{
				tx_buffer[2] = 0xda;
				tx_buffer[3] = status;
				uint16_t curr = get_motor_current();
				if ( (curr > 0) && (curr < 16)) {
					// return at least the minimum (16 mA) if non-zero
					curr = 1;
				} else {
					curr /= 16;
				}
				if (curr > 255) {
					curr = 255;	// maximum reported value is 4 amps
				}
				tx_buffer[4] = (uint8_t)curr;
				tx_buffer[5] = (uint8_t)(get_rpm()*4); // extended speed is with 2 bits of floating point precision
				float pos2 = location_to_position100() * 256;
				int pos = pos2;
				tx_buffer[6] = pos >> 8;
				tx_buffer[7] = pos & 0xff;
				tx_buffer[8] = curr_pwm;
				tx_buffer[9] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7] ^ tx_buffer[8];
				*tx_bytes=10;
			}
			break;
		case CMD_EXT_GET_LIMITS:
			{
				tx_buffer[2] = 0xdb;
				tx_buffer[3] = calibrating | (orientation<<1);
				tx_buffer[4] = max_curtain_length >> 8;
				tx_buffer[5] = max_curtain_length & 0xff;
				tx_buffer[6] = full_curtain_length >> 8;
				tx_buffer[7] = full_curtain_length & 0xff;
				tx_buffer[8] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7];
				*tx_bytes=9;
			}
			break;
		default:
			cmd_handled=0;
	}

	if (!cmd_handled) {
		// one byte commands with parameter
		if (cmd1 == CMD_EXT_SET_SPEED) {
			if (cmd2 > 1) {
				default_speed = cmd2;
				if (target_speed != 0)
					target_speed = cmd2;
			}
		} else if (cmd1 == CMD_EXT_SET_DEFAULT_SPEED) {
			if (cmd2 > 1) {
				motor_write_setting(DEFAULT_SPEED_EEPROM, cmd2);
				default_speed = cmd2;
				dance();
			}
		} else if (cmd1 == CMD_GO_TO) {
			if (!calibrating) {
				target_location = position100_to_location(cmd2);
				if (target_location < location) {
					command = MotorUp;
				} else {
					command = MotorDown;
				}
			}
		} else if ((cmd1 & 0xf0) == CMD_EXT_GO_TO) {
			if (!calibrating) {
				uint16_t pos = ((cmd1 & 0x0f)<<8) + cmd2;
				float pos2 = ((float)pos)/16;
				target_location = position100_to_location(pos2);
				if (target_location < location) {
					command = MotorUp;
				} else {
					command = MotorDown;
				}
			}
		} else if ((cmd1 & 0xf0) == CMD_EXT_SET_LOCATION) {
			// There is only room for 12 bits of data, so we have omitted 1 least-significant bit
			uint16_t loc = (((cmd1 & 0x0f)<<8) + cmd2) << 1;
			location = loc;
			calibrating = 0;
		} else if (cmd1 == CMD_EXT_SET_MINIMUM_VOLTAGE) {
			motor_write_setting(MINIMUM_VOLTAGE_EEPROM, cmd2);
			minimum_voltage = cmd2;
		} else if (cmd1 == CMD_EXT_SET_AUTO_CAL) {
			motor_write_setting(AUTO_CAL_EEPROM, cmd2);
			auto_calibration = cmd2;
		} else if (cmd1 == CMD_EXT_SET_ORIENTATION) {
			motor_write_setting(ORIENTATION_EEPROM, cmd2);
			orientation = cmd2;
		} else if (cmd1 == CMD_EXT_SET_MAX_MOTOR_CURRENT) {
			uint16_t curr = cmd2 * 16;
			motor_write_setting(MAX_MOTOR_CURRENT_EEPROM, curr);
			max_motor_current = curr;
		} else if (cmd1 == CMD_EXT_SET_STALL_DETECTION_TIMEOUT) {
			uint16_t timeout = cmd2 * 8;
			motor_write_setting(STALL_DETECTION_TIMEOUT_EEPROM, timeout);
			stall_detection_timeout = timeout;
		} else if (cmd1 == CMD_EXT_SET_SLEEP_DELAY) {
			uint16_t delay = cmd2 * 256;
			motor_write_setting(IDLE_MODE_SLEEP_DELAY_EEPROM, delay);
			idle_mode_sleep_delay = delay;
		} else if ((cmd1 & 0xf0) == CMD_EXT_GO_TO_LOCATION) {
			// There is only room for 12 bits of data, so we have omitted 1 least-significant bit
			target_location = (((cmd1 & 0x0f)<<8) + cmd2) << 1;
			if (target_location < location) {
				command = MotorUp;
			} else {
				command = MotorDown;
			}
		} else if (cmd1 == CMD_EXT_SET_SLOWDOWN_FACTOR) {
			slowdown_factor = cmd2;
		} else if (cmd1 == CMD_EXT_SET_MIN_SLOWDOWN_SPEED) {
			min_slowdown_speed = cmd2;
		} else if (cmd1 == CMD_EXT_PING) {
			if (cmd2 == 0) {
				blink += 1;
			} else {
				tx_buffer[2] = 0xba;
				tx_buffer[3] = 0x00;
				tx_buffer[4] = 0xff;
				tx_buffer[5] = 0x9a;
				tx_buffer[6] = cmd2+1;
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
		}
	}

	if ( (cmd != CMD_GET_STATUS) && (cmd != CMD_EXT_GET_STATUS) ) {
		last_command = cmd;
	}

	return 1;
}

void motor_init() {

	motor_stop();

	reset_sleep_timer();

	location = max_curtain_length; // assume we are at bottom position

	if (auto_calibration) {
		calibrating = 1;
		command = MotorUp;
	} else {
		calibrating = 0;
		command = NoCommand;
	}
}




