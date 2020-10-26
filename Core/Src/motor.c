/*
 * motor.c
 *
 *  Created on: Aug 29, 2020
 *      Author: markojuhanne
 */
#include "main.h"
#include "motor.h"

enum motor_status status;
enum motor_direction direction;

#define INITIAL_PWM 80
#define GEAR_RATIO 189
#define DEG_TO_LOCATION(x) (GEAR_RATIO * x / 360)

uint32_t hard_lower_limit = GEAR_RATIO * (13 + 265.0/360);
uint32_t soft_lower_limit;

uint8_t default_speed = DEFAULT_TARGET_SPEED;
uint8_t target_speed = 0; // target RPM

uint32_t target_location = 0;

uint8_t curr_pwm = 0;  // PWM setting
int32_t location = 0;	// location is the position of the curtain measured in HALL sensor ticks
uint8_t resetting = 0;

uint32_t hall_sys_ticks = 0;
uint32_t hall_interval = 0; // how many milliseconds between hall sensor ticks

uint32_t hall_1_tick = 0;
uint32_t hall_2_tick = 0;

uint32_t movement_started_timestamp = 0;

enum motor_command command; // for deferring execution to main loop since we don't want to invoke HAL_Delay in UARTinterrupt handler

#define CMD_GO_TO		0xdd
#define CMD_EXT_GO_TO	0x10	// target position is the lower 4 bits of the 1st byte + 2nd byte (12 bits of granularity), where lower 4 bits is the decimal part
#define CMD_SET_SPEED 	0x20
#define CMD_UP 		0x0add
#define CMD_DOWN 	0x0aee
#define CMD_UP_17 	0x0a0d
#define CMD_DOWN_17	0x0a0e
#define CMD_STOP	0x0acc

#define CMD_OVERRIDE_UP_90		0xfad1
#define CMD_OVERRIDE_DOWN_90	0xfad2
#define CMD_OVERRIDE_UP_6		0xfad3
#define CMD_OVERRIDE_DOWN_6		0xfad4
#define CMD_SET_SOFT_LIMIT		0xfaee
#define CMD_SET_HARD_LIMIT		0xfacc
#define CMD_RESET_SOFT_LIMIT	0xfa00


#define CMD_GET_STATUS 	0xcccc
#define CMD_GET_STATUS2 0xcccd
#define CMD_GET_STATUS3 0xccce
#define CMD_GET_STATUS4 0xccdd
#define CMD_GET_EXT_STATUS 0xccde
#define CMD_GET_EXT_LIMITS 0xccdf

uint32_t position100_to_location( float position ) {
	if (position > 100)
		return 100;
	return position*soft_lower_limit/100;
}


float location_to_position100() {
	if (resetting)
		return 50;
	if (location < 0) {
		return 0;
	}
	if (location > soft_lower_limit) {
		return 100;
	}
	return 100*location/soft_lower_limit;
}


float get_rpm() {
	if (hall_interval)
		return 60*1000/189/hall_interval;
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
	if (sensor==HALL_1_SENSOR) {
		hall_1_tick++;
		hall_interval = hall_sys_ticks;
		hall_sys_ticks = 0;
		if (!resetting)
			process_location();
	} else if (sensor==HALL_2_SENSOR) {
		hall_2_tick++;
	}
}


void motor_adjust_rpm() {
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

void motor_stall_check() {
	if (status == Moving) {
		// Count how many milliseconds since previous HALL sensor interrupt
		// in order to calculate RPM and detect motor stalling
		hall_sys_ticks ++;
		if (HAL_GetTick() - movement_started_timestamp > MOVEMENT_GRACE_PERIOD) {
			// enough time has passed since motor is energized -> apply stall detection

			if (hall_sys_ticks > HALL_TIMEOUT) {
				// motor has stalled/stopped
				motor_stopped();
				hall_sys_ticks = 0;
			}
		}
	}
}

void motor_stopped() {
	if (status != Stopped) {
		// motor has stalled!

		hall_interval = 0;
		if (resetting) {
			// we reached top position
			resetting = 0;
			location = 0;
		}
		motor_stop();
		hall_sys_ticks = 0;
	}
}


void motor_stop() {
	pwm_stop(LOW1_PWM_CHANNEL);
	pwm_stop(LOW2_PWM_CHANNEL);


	// make sure that all mosfets are off

	// remember also to set GPIO_PULLDOWN in HAL_TIM_MspPostInit !

	//HAL_GPIO_WritePin(LOW_1_GATE_GPIO_Port, LOW_1_GATE_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(LOW_2_GATE_GPIO_Port, LOW_2_GATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HIGH_1_GATE_GPIO_Port, HIGH_1_GATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_RESET);
	TIM1->CCR1 = 0;
	TIM1->CCR4 = 0;
	status = Stopped;
	direction = None;
	curr_pwm = 0;
	hall_interval=0;
	target_speed = 0;
}



void motor_up(uint8_t motor_speed) {
	// reset stall detection timeout
	hall_sys_ticks = 0;
	movement_started_timestamp = HAL_GetTick();
	motor_stop();
	HAL_Delay(10);

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

	// reset stall detection timeout
	hall_sys_ticks = 0;
	movement_started_timestamp = HAL_GetTick();
	motor_stop();
	HAL_Delay(10);

	// turn on LOW1 PWM and HIGH2
	pwm_start(LOW1_PWM_CHANNEL);
	target_speed = motor_speed;
	TIM1->CCR1 = INITIAL_PWM;
	curr_pwm = INITIAL_PWM;
	HAL_GPIO_WritePin(HIGH_2_GATE_GPIO_Port, HIGH_2_GATE_Pin, GPIO_PIN_SET);
	direction = Down;
	status = Moving;
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
				tx_buffer[4] = (uint8_t)( get_voltage()/16);
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
				target_location = soft_lower_limit;
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
				if (target_location > soft_lower_limit)
					target_location = soft_lower_limit;
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

		case CMD_SET_SOFT_LIMIT:
			{
				soft_lower_limit = location;
			}
			break;

		case CMD_SET_HARD_LIMIT:
			{
				hard_lower_limit = location;
			}
			break;

		case CMD_RESET_SOFT_LIMIT:
			{
				soft_lower_limit = hard_lower_limit;
				resetting=1;
			}
			break;

		case CMD_GET_EXT_STATUS:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xda;
				tx_buffer[3] = status;
				tx_buffer[4] = (uint8_t)(get_motor_current());
				uint16_t pos = location_to_position100() * 256;
				tx_buffer[5] = pos >> 8;
				tx_buffer[6] = pos & 0xff;
				tx_buffer[7] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6];
				*tx_bytes=8;
			}
			break;
		case CMD_GET_EXT_LIMITS:
			{
				tx_buffer[0] = 0x00;
				tx_buffer[1] = 0xff;
				tx_buffer[2] = 0xdb;
				tx_buffer[3] = resetting;
				tx_buffer[4] = soft_lower_limit >> 8;
				tx_buffer[5] = soft_lower_limit & 0xff;
				tx_buffer[6] = hard_lower_limit >> 8;
				tx_buffer[7] = hard_lower_limit & 0xff;
				tx_buffer[8] = tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[6] ^ tx_buffer[7];
				*tx_bytes=9;
			}
			break;

		default:
			cmd_handled=0;

	}

	if (!cmd_handled) {

		// one byte commands with parameter

		if (cmd1 == CMD_SET_SPEED) {
			default_speed = cmd2;
			if (target_speed != 0)
				target_speed = cmd2;

		} else if (cmd1 == CMD_GO_TO) {
			target_location = position100_to_location(cmd2);
			if (target_location < location) {
				command = MotorUp;
			} else {
				command = MotorDown;
			}
		} else if ((cmd1 & 0xf0) == CMD_EXT_GO_TO) {
			uint16_t pos = ((cmd1 & 0x0f)<<8) + cmd2;
			float pos2 = ((float)pos)/16;
			target_location = position100_to_location(pos2);
			if (target_location < location) {
				command = MotorUp;
			} else {
				command = MotorDown;
			}
		}

	}

	return 1;

}

void motor_init() {
	resetting = 0;
	direction = None;
	command = NoCommand;
	soft_lower_limit = hard_lower_limit;
	motor_stop();

#ifdef AUTO_RESET
	resetting = 1;
	motor_up();
#endif
}




