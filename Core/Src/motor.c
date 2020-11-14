/*
 * motor.c
 *
 *  Created on: Aug 29, 2020
 *      Author: markojuhanne
 */
#include "main.h"
#include "motor.h"
#include "eeprom.h"

enum motor_status status;
enum motor_direction direction;

#define DEG_TO_LOCATION(x) (GEAR_RATIO * x / 360)



/* LOCATION is the spatial position of the curtain measured in motor revolutions. Due to additional gear mechanism, it takes
 * GEAR_RATIO revolutions in order to actually reach 1 full revolution of the curtain rod. Motor revolution is detected by HALL sensor,
 * which generates 1 interrupt (tick) per motor revolution.
 *
 * POSITION itself is a measure of curtain position reported by float between 0.0 (fully closed) and 100.0 (fully open) and can be
 * calculated from LOCATION with location_to_position100 (and vice versa with position100_to_location).
 *
 * Maximum POSITION is affected by user-customizable max curtain length (configured via CMD_SET_MAX_CURTAIN_LENGTH). In addition to this,
 * there is the "absolute" limit of full (factory defined) curtain length. However these limits can be ignored using CMD_OVERRIDE_XXX commands and also be
 * re-configured with CMD_SET_MAX_CURTAIN_LENGTH / CMD_SET_FULL_CURTAIN_LENGTH commands.
 */
uint32_t target_location = 0;
int32_t location = 0;
uint32_t full_curtain_length = DEFAULT_FULL_CURTAIN_LEN;
uint32_t max_curtain_length;

uint16_t minimum_voltage;	// value is minimum voltage * 16 (float stored as interger value)

/* the motor driver gate PWM duty cycle is initially 80/255 when first energized and then adjusted according to target_speed */
#define INITIAL_PWM 80

uint8_t default_speed;
uint8_t target_speed = 0; // target RPM
uint8_t curr_pwm = 0;  // PWM setting

/*
 * When resetting we forget our position and wait until motor has stalled. When that happens, assume that we are in top
 * position and reset location to 0.
 */
uint8_t resetting = 0;

/*
 * count how many milliseconds since previous HALL sensor interrupt occurred
 * in order to calculate hall_sensor_interval (and RPM) and also to detect motor stalling
 */
uint32_t hall_sensor_idle_time = 0;

uint32_t hall_sensor_ticks = 0;	// how many hall sensor ticks(signals) after movement

uint32_t hall_sensor_interval = 0; // how many milliseconds between hall sensor ticks (used to calculate motor RPM)

uint32_t movement_started_timestamp = 0;	// used for stall detection grace period (let motor some time to energize before applying stall detecion)

enum motor_command command; // for deferring execution to main loop since we don't want to invoke HAL_Delay in UARTinterrupt handler

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
#define CMD_RESET_CURTAIN_LENGTH	0xfa00	// will cause maximum curtain length to be reseted to factory setting (full curtain length) and to be stored to flash memory

#define CMD_GET_STATUS 	0xcccc
#define CMD_GET_STATUS2 0xcccd
#define CMD_GET_STATUS3 0xccce
#define CMD_GET_STATUS4 0xccdd

// ------ Commands supported only by our custom firmware -------

// commands with 1 parameter
#define CMD_EXT_GO_TO				0x10	// target position is the lower 4 bits of the 1st byte + 2nd byte (12 bits of granularity), where lower 4 bits is the decimal part
#define CMD_EXT_SET_SPEED 			0x20	// setting speed via this command will not alter non-volatile memory (so it's safe for limited write-cycle flash memory)
#define CMD_EXT_SET_DEFAULT_SPEED 	0x30	// default speed will be stored to flash memory
#define CMD_EXT_SET_MINIMUM_VOLTAGE	0x40	// minimum voltage. Will be stored to flash memory
#define CMD_EXT_SET_LOCATION		0x50	// location is the lower 4 bits of the 1st byte + 2nd byte (1 sign bit + 11 bits of integer part)

// commands without parameter
#define CMD_EXT_GET_LOCATION 		0xccd0
#define CMD_EXT_GET_VERSION 		0xccdc
#define CMD_EXT_GET_STATUS 			0xccde
#define CMD_EXT_GET_LIMITS 			0xccdf

/****************** EEPROM variables ********************/

typedef enum eeprom_var_t {
	MAX_CURTAIN_LEN_EEPROM = 0,
	FULL_CURTAIN_LEN_EEPROM = 1,
	MINIMUM_VOLTAGE_EEPROM = 2,
	DEFAULT_SPEED_EEPROM = 3
} eeprom_var_t;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888};


void motor_set_default_settings() {
	max_curtain_length = DEFAULT_FULL_CURTAIN_LEN; // by default, max_curtain_length is full_curtain_length
	full_curtain_length = DEFAULT_FULL_CURTAIN_LEN;
	minimum_voltage = DEFAULT_MINIMUM_VOLTAGE;
	default_speed = DEFAULT_TARGET_SPEED;
}

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
	if (EE_ReadVariable(VirtAddVarTab[DEFAULT_SPEED_EEPROM], &tmp) != 0) {
		tmp = default_speed = DEFAULT_TARGET_SPEED;
		EE_WriteVariable(VirtAddVarTab[DEFAULT_SPEED_EEPROM], tmp);
	} else {
		default_speed = tmp;
	}
}

void motor_write_setting( eeprom_var_t var, uint16_t value ) {
	uint16_t tmp;
	if ( (status == Stopped) || (status == Stalled) ) {
		// motor has to be stopped to change non-volatile settings (writing to FLASH should occur uninterrupted)
		EE_ReadVariable(VirtAddVarTab[var], &tmp);
		if (tmp != value) {
			EE_WriteVariable(VirtAddVarTab[var], value);
		}
	}
}


uint32_t position100_to_location( float position ) {
	if (position > 100)
		return 100;
	return position*max_curtain_length/100;
}


float location_to_position100() {
	if (resetting) {
		// When resetting we forget our position and return 50% instead
		return 50;
	}
	if (location < 0) {
		return 0;
	}
	if (location > max_curtain_length) {
		return 100;
	}
	return 100*(float)location/max_curtain_length;
}


float get_rpm() {
	// There might be condition when right after the movement starts, the Hall sensor signal comes abruptly fast and
	// motor speed is calculated to be erroneously high. To prevent stalling (because motor PWM is dropped too low),
	// we report motor RPMS only after 2 hall sensor signals so that RPM can be calculated correctly
	if ( (hall_sensor_ticks > 1) && (hall_sensor_interval) )
		return 60*1000/GEAR_RATIO/hall_sensor_interval;
	return 0;
}


void process_location() {
	if (direction == Up) {
		location--;
		if (location <= target_location) {
			motor_stop();
		}
	} else if (direction == Down) {
		location++;

		if(location >= target_location) {
			motor_stop();
		}
	}
}

void hall_sensor_callback( uint8_t sensor ) {
	if (sensor == HALL_1_SENSOR) {
		hall_sensor_ticks++;
		hall_sensor_interval = hall_sensor_idle_time;	// update time passed between hall sensor interrupts
		hall_sensor_idle_time = 0;
		if (!resetting)
			process_location();
	} else if (sensor == HALL_2_SENSOR) {
		// We actually use only HALL #1 sensor for RPM calculation and stall detection
	}
}


/* Called every 10ms by TIM3 */
void motor_adjust_rpm() {
	if (status == Moving) {
		uint32_t speed = get_rpm();
		if (speed < target_speed) {
			if (curr_pwm < 255) {
				curr_pwm ++;
				if (direction == Up)
					TIM1->CCR4 = curr_pwm;
				else
					TIM1->CCR1 = curr_pwm;
			}
		}

		if (speed > target_speed) {
			if (curr_pwm > 0) {
				curr_pwm --;
				if (direction == Up)
					TIM1->CCR4 = curr_pwm;
				else
					TIM1->CCR1 = curr_pwm;
			}
		}
	}
}


/*
 * This is periodically (every 1 millisecond) called by SysTick_Handler
 */
void motor_stall_check() {
	if (status == Moving) {
		// Count how many milliseconds since previous HALL sensor interrupt
		// in order to calculate RPM and detect motor stalling
		hall_sensor_idle_time ++;
		if (HAL_GetTick() - movement_started_timestamp > MOVEMENT_GRACE_PERIOD) {
			// enough time has passed since motor is energized -> apply stall detection

			if (hall_sensor_idle_time > HALL_TIMEOUT) {
				// motor has stalled/stopped
				motor_stopped();
				hall_sensor_idle_time = 0;
			}
		}
	}
}

void motor_stopped() {
	if (status != Stopped) {
		// motor has stalled!

		hall_sensor_interval = 0;
		if (resetting) {
			// we reached top position
			resetting = 0;
		}

		// If motor has stalled, we assume that we have reached the top position.
		location = 0;

		// De-energize the motor just in case..
		motor_stop();
		status = Stalled;
		hall_sensor_idle_time = 0;
	}
}


void motor_stop() {

	// Make sure that all mosfets are off
	pwm_stop(LOW1_PWM_CHANNEL);
	pwm_stop(LOW2_PWM_CHANNEL);
	// Remember also to set GPIO_PULLDOWN in HAL_TIM_MspPostInit ! (generated automatically by CubeMX)

	HAL_GPIO_WritePin(HIGH_1_GATE_GPIO_Port, HIGH_1_GATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_RESET);
	TIM1->CCR1 = 0;
	TIM1->CCR4 = 0;
	status = Stopped;
	direction = None;
	curr_pwm = 0;
	hall_sensor_interval = 0;
	hall_sensor_ticks = 0;
	target_speed = 0;
}



void motor_up(uint8_t motor_speed) {

	motor_stop();
	HAL_Delay(10);
	// reset stall detection timeout
	hall_sensor_idle_time = 0;
	hall_sensor_ticks = 0;
	movement_started_timestamp = HAL_GetTick();

	// turn on LOW2 PWM and HIGH1
	pwm_start(LOW2_PWM_CHANNEL);
	target_speed = motor_speed;
	TIM1->CCR4 = INITIAL_PWM;
	curr_pwm = INITIAL_PWM;
	HAL_GPIO_WritePin(HIGH_1_GATE_GPIO_Port, HIGH_1_GATE_Pin, GPIO_PIN_SET);
	direction = Up;
	status = Moving;
}

void motor_down(uint8_t motor_speed) {

	motor_stop();
	HAL_Delay(10);
	// reset stall detection timeout
	hall_sensor_idle_time = 0;
	hall_sensor_ticks = 0;
	movement_started_timestamp = HAL_GetTick();

	// turn on LOW1 PWM and HIGH2
	pwm_start(LOW1_PWM_CHANNEL);
	target_speed = motor_speed;
	TIM1->CCR1 = INITIAL_PWM;
	curr_pwm = INITIAL_PWM;
	HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_SET);
	direction = Down;
	status = Moving;
}

uint8_t check_voltage() {
	if (minimum_voltage != 0) {
		uint16_t voltage = get_voltage() / 30;
		if (voltage < minimum_voltage)
			return 0;
	}
	return 1;
}

void motor_process() {
	if (command == MotorUp) {
		motor_up(default_speed);
		command = NoCommand;
	} else if (command == MotorDown) {
		motor_down(default_speed);
		command = NoCommand;
	} else if(command == Stop) {
		motor_stop();
		command = NoCommand;
	}
}


uint8_t calculate_battery() {
	return 0x12; // TODO
}



uint8_t handle_command(uint8_t * rx_buffer, uint8_t * tx_buffer, uint8_t burstindex, uint8_t * tx_bytes) {
	uint8_t cmd1, cmd2;
	cmd1 = rx_buffer[3];
	cmd2 = rx_buffer[4];
	uint16_t cmd = (cmd1 << 8) + cmd2;

	uint8_t cmd_handled = 1;

	switch (cmd) {

		case CMD_GET_STATUS:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xd8;
				tx_buffer[3] = calculate_battery();
				tx_buffer[4] = (uint8_t)( get_voltage()/16);  // returned value is voltage*30
				tx_buffer[5] = (uint8_t)get_rpm();
				tx_buffer[6] = location_to_position100();
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;

		case CMD_UP:
			{
				target_location = 0;
				command = MotorUp;
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
				target_location -= DEG_TO_LOCATION(17);
				if (target_location < 0)
					target_location = 0;
				command = MotorUp;
			}
			break;

		case CMD_DOWN_17:
			{
				target_location += DEG_TO_LOCATION(17);
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
				target_location -= DEG_TO_LOCATION(90);
				command = MotorUp;
			}
			break;

		case CMD_OVERRIDE_DOWN_90:
			{
				target_location += DEG_TO_LOCATION(90);
				command = MotorDown;
			}
			break;

		case CMD_OVERRIDE_UP_6:
			{
				target_location -= DEG_TO_LOCATION(6);
				command = MotorUp;
			}
			break;

		case CMD_OVERRIDE_DOWN_6:
			{
				target_location += DEG_TO_LOCATION(6);
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
			}
			break;


		case CMD_RESET_CURTAIN_LENGTH:
			{
				motor_write_setting(MAX_CURTAIN_LEN_EEPROM, full_curtain_length);
				max_curtain_length = full_curtain_length;
				resetting = 1;
			}
			break;

		case CMD_EXT_GET_VERSION:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xd0;
				tx_buffer[3] = VERSION_MAJOR;
				tx_buffer[4] = VERSION_MINOR;
				tx_buffer[5] = minimum_voltage;
				tx_buffer[6] = 0;
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;

		case CMD_EXT_GET_LOCATION:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xd1;
				tx_buffer[3] = location >> 8;
				tx_buffer[4] = location & 0xff;
				tx_buffer[5] = target_location >> 8;
				tx_buffer[6] = target_location & 0xff;
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;

		case CMD_EXT_GET_STATUS:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xda;
				tx_buffer[3] = status;
				tx_buffer[4] = (uint8_t)(get_motor_current());
				tx_buffer[5] = (uint8_t)get_rpm();
				float pos2 = location_to_position100() * 256;
				int pos = pos2;
				tx_buffer[6] = pos >> 8;
				tx_buffer[7] = pos & 0xff;
				tx_buffer[8] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7];
				*tx_bytes=9;
			}
			break;

		case CMD_EXT_GET_LIMITS:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xdb;
				tx_buffer[3] = resetting;
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
			if (cmd2 > 0) {
				motor_write_setting(DEFAULT_SPEED_EEPROM, cmd2);
				default_speed = cmd2;
			}
		} else if (cmd1 == CMD_GO_TO) {
			if (!resetting) {
				target_location = position100_to_location(cmd2);
				if (target_location < location) {
					command = MotorUp;
				} else {
					command = MotorDown;
				}
			}
		} else if ((cmd1 & 0xf0) == CMD_EXT_GO_TO) {
			if (!resetting) {
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
			int16_t loc = ((cmd1 & 0x0f)<<8) + cmd2;
			location = loc;
			resetting = 0;
		} else if (cmd1 == CMD_EXT_SET_MINIMUM_VOLTAGE) {
			motor_write_setting(MINIMUM_VOLTAGE_EEPROM, cmd2);
			minimum_voltage = cmd2;
		}
	}

	return 1;
}

void motor_init() {
	resetting = 0;
	direction = None;
	command = NoCommand;
	motor_stop();

	location = max_curtain_length; // assume we are at bottom position

#ifdef AUTO_RESET
	resetting = 1;
	motor_up();
#endif
}




