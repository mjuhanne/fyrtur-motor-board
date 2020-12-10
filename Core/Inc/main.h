/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

// SLIM_BINARY disables few functionalities (mainly EEPROM code) so that the binary can be built with Debug mode. Normally keep this commented
//#define SLIM_BINARY

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
    volatile uint8_t  flag;     /* Timeout event flag */
    uint16_t timer;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t hall_sys_ticks;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void hall_sensor_callback( uint8_t sensor, uint8_t value );
void motor_stopped();
void motor_stall_check();
void pwm_start(uint32_t channel);
void pwm_stop(uint32_t channel);
uint16_t get_voltage();
uint16_t get_motor_current();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HIGH_2_GATE_Pin GPIO_PIN_4
#define HIGH_2_GATE_GPIO_Port GPIOA
#define HIGH_1_GATE_Pin GPIO_PIN_5
#define HIGH_1_GATE_GPIO_Port GPIOA
#define VOLTAGE_SENSE_Pin GPIO_PIN_6
#define VOLTAGE_SENSE_GPIO_Port GPIOA
#define HALL_1_OUT_Pin GPIO_PIN_7
#define HALL_1_OUT_GPIO_Port GPIOA
#define HALL_1_OUT_EXTI_IRQn EXTI4_15_IRQn
#define HALL_2_OUT_Pin GPIO_PIN_0
#define HALL_2_OUT_GPIO_Port GPIOB
#define HALL_2_OUT_EXTI_IRQn EXTI0_1_IRQn
#define MOTOR_CURRENT_Pin GPIO_PIN_1
#define MOTOR_CURRENT_GPIO_Port GPIOB
#define LOW_1_GATE_Pin GPIO_PIN_8
#define LOW_1_GATE_GPIO_Port GPIOA
#define LOW_2_GATE_Pin GPIO_PIN_11
#define LOW_2_GATE_GPIO_Port GPIOA
#define PWR_EN_Pin GPIO_PIN_12
#define PWR_EN_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define BUT_Pin GPIO_PIN_7
#define BUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define HALL_1_SENSOR 0
#define HALL_2_SENSOR 1

#define LOW1_PWM_CHANNEL TIM_CHANNEL_1
#define LOW2_PWM_CHANNEL TIM_CHANNEL_4

#define UART_DMA_BUF_SIZE        64      /* DMA circular buffer size in bytes */
#define DMA_TIMEOUT_MS      10      /* DMA Timeout duration in msec */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
