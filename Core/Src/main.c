/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 2*32


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

extern idle_mode_sleep_delay;

/* USER CODE BEGIN PV */

DMA_Event_t dma_uart_rx = {0,0,UART_DMA_BUF_SIZE};

uint8_t uart_dma_rx_buffer[UART_DMA_BUF_SIZE]; // circular DMA rx buffer

uint8_t uart_rx_buffer[UART_DMA_BUF_SIZE]; // contains newly received data
uint16_t uart_rx_buffer_len = 0;
uint8_t uart_tx_buffer[16];

uint8_t blink;
uint8_t uart_tx_busy;

uint16_t adc_buf[ADC_BUF_LEN];

uint16_t motor_current;

/* 
 * Used to track when motor is idle and no commands have been issued. 
 * After IDLE_MODE_SLEEP_DELAY milliseconds sleep mode is entered.
 * If set to 0 then sleep mode is disabled temporarily (e.g. during movement and calibration)
 */
uint32_t idle_timestamp;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Calculate the average current of 32 previous measurements
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	uint32_t avg = 0;
	for (int i=0;i<ADC_BUF_LEN/2;i++) {
		avg += adc_buf[i*2+1] * 2;	// current in mA
	}
	avg /= ADC_BUF_LEN/2;
	motor_current = avg;
}

uint16_t get_voltage() {
	return adc_buf[0];	// values stored in adc_buf are voltages * 30 * 16;
}

uint16_t get_motor_current() {
	return motor_current;
}

void pwm_start( uint32_t channel ) {
	if (htim1.Instance != NULL)
		HAL_TIM_PWM_Start(&htim1, channel);

}

void pwm_stop( uint32_t channel ) {
	if (htim1.Instance != NULL)
		HAL_TIM_PWM_Stop(&htim1, channel);
}

#ifndef SLIM_BINARY
void blink_led(int duration, int count) {
	for (int i=0;i<count;i++) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(duration);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(duration);
	}
}
#endif

/* Called every 10ms by TIM3 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	motor_adjust_rpm();
}

uint8_t uart_tx_done() {
  return !uart_tx_busy;
}

void uart_start_rx_DMA() {
  if(HAL_UART_Receive_DMA(&huart1, (uint8_t*)uart_dma_rx_buffer, UART_DMA_BUF_SIZE) != HAL_OK)
      {
          Error_Handler();
      }
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);  // disable half-transfer interrupt
  // Reset also the buffer pointer
  dma_uart_rx.prevCNDTR = UART_DMA_BUF_SIZE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t i, pos, start, length;
	uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);

	/* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
	if(dma_uart_rx.flag && currCNDTR == UART_DMA_BUF_SIZE)
	{
		dma_uart_rx.flag = 0;
		return;
	}

	/* Determine start position in DMA buffer based on previous CNDTR value */
	start = (dma_uart_rx.prevCNDTR < UART_DMA_BUF_SIZE) ? (UART_DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

	if(dma_uart_rx.flag)    /* Timeout event */
	{
		/* Determine new data length based on previous DMA_CNDTR value:
		 *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
		 *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
		*/
		length = (dma_uart_rx.prevCNDTR < UART_DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (UART_DMA_BUF_SIZE - currCNDTR);
		dma_uart_rx.prevCNDTR = currCNDTR;
		dma_uart_rx.flag = 0;

		if ( (length==0) && (uart_rx_buffer_len>0) ) {
			// There was incomplete packet waiting for the rest of the data which never came..
			// Send ERROR MSG: Send back the number of bytes received and first four received bytes
			uart_tx_buffer[0] = 0xde;
			uart_tx_buffer[1] = 0xad;
			uart_tx_buffer[2] = uart_rx_buffer_len;
			uart_tx_buffer[3] = uart_rx_buffer[0];
			uart_tx_buffer[4] = uart_rx_buffer[1];
			uart_tx_buffer[5] = uart_rx_buffer[2];
			uart_tx_buffer[6] = uart_rx_buffer[3];
			uart_tx_buffer[7] = uart_tx_buffer[3] ^ uart_tx_buffer[4] ^ uart_tx_buffer[5] ^ uart_tx_buffer[6];
      uart_tx_busy = 1;
			HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 8);
			uart_rx_buffer_len = 0;
		}
	}
	else                /* DMA Rx Complete event */
	{
		length = UART_DMA_BUF_SIZE - start;
		dma_uart_rx.prevCNDTR = UART_DMA_BUF_SIZE;
	}

	/* Copy and Process new data */
	// Acknowledge also the possible previously received (incomplete) packet
	for(i=0,pos=start; i<length; ++i,++pos)
	{
		uart_rx_buffer[uart_rx_buffer_len + i] = uart_dma_rx_buffer[pos];
	}
	length += uart_rx_buffer_len;

	// handle the packet(s)
	pos=0;
	uint16_t len = length;
	while (len>=6) {
		uint8_t tx_bytes=0;
		if (handle_command(&uart_rx_buffer[pos], uart_tx_buffer, &tx_bytes )) {
			if (tx_bytes) {
				uart_tx_buffer[0] = 0x00;
				uart_tx_buffer[1] = 0xff;
        uart_tx_busy = 1;
				HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, tx_bytes);
			}
//      blink += 1;
			pos += 6;
			len -= 6;
		}
	}
	if (pos < length) {
		// Incomplete packet (either true loss of data or we are waiting
		// for more data from DMA but arrived at the end of the cyclic DMA buffer
		// while mid-packet, and must wait for the next DMA interrupt for the rest of the data

		// lets save the first chunk
		len = length - pos;
		if (pos != 0) {
			for (int i=0;i<len;i++)
				uart_rx_buffer[i] = uart_rx_buffer[pos+i];
		}
		uart_rx_buffer_len = len;

		/* Start DMA timer */
		dma_uart_rx.timer = DMA_TIMEOUT_MS;
	} else {
		uart_rx_buffer_len = 0;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  uart_tx_busy = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	//Error_Handler();

	// Sometimes HAL_UART_ERROR_FE occurs on power on. Manage this..
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);

  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  if(huart->Instance == USART1) {
    // Restart the RX DMA
    uart_start_rx_DMA();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == HALL_1_OUT_Pin) {
		  hall_sensor_callback(HALL_1_SENSOR, HAL_GPIO_ReadPin( HALL_1_OUT_GPIO_Port, HALL_1_OUT_Pin));
	} else if (GPIO_Pin == HALL_2_OUT_Pin) {
		  hall_sensor_callback(HALL_2_SENSOR, HAL_GPIO_ReadPin( HALL_2_OUT_GPIO_Port, HALL_2_OUT_Pin));
	}
}

uint8_t sleep_timer_enabled() {
	return (idle_timestamp != 0);
}

void disable_sleep_timer() {
	idle_timestamp = 0; // disable sleep mode timer
}

void reset_sleep_timer() {
	idle_timestamp = HAL_GetTick();
}

uint8_t sleep_timer_timeout() {
  if (idle_mode_sleep_delay > 0) {
    if (idle_timestamp == 0)
      return 0;
    if (HAL_GetTick() - idle_timestamp > idle_mode_sleep_delay) 
      return 1;
  }
  return 0;
}

/*
 * Original FW: Sleep mode current consumption 0.350 mA @ 7V (ST-Link connected)
 * 
 * This FW: 
 *  run (normal mode): 13 mA
 *  sleep mode: 1.7 mA (not used currently)
 *  stop mode: 0.337 mA (ST-Link connected)
 */
void enter_sleep_mode() {

  // Stop TIM3 (motor RPM adjust timer) and ADC
  HAL_TIM_Base_Stop_IT(&htim3);
  HAL_ADC_Stop_DMA(&hadc);

  // Disable HALL sensors and voltage sensor (LM321 op amp)
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);

#ifdef BLINK_LEDS_WHEN_SLEEP_MODE_CHANGES
  blink_led(100,3);
#endif

  // stop SysTick
  HAL_SuspendTick();

  // Deinitialize UART and use UART1_RX only for wake-up interrupt
  HAL_UART_DeInit(&huart1);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // APB peripheral power interface clock needs to be enabled
  __HAL_RCC_PWR_CLK_ENABLE();

  // --- Go to sleep (Stop mode) ------
  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  // ---- Now we are awake ---

  // Is this needed?
  // SystemClock_Config();

  // Restart SysTick
  HAL_ResumeTick();
 
  MX_USART1_UART_Init();

  uart_start_rx_DMA();

#ifdef BLINK_LEDS_WHEN_SLEEP_MODE_CHANGES
  blink_led(100,3);
#endif

  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buf, ADC_BUF_LEN);
  __HAL_DMA_DISABLE_IT(hadc.DMA_Handle, DMA_IT_HT);  // disable half-transfer interrupt

  HAL_TIM_Base_Start_IT(&htim3);

  // Enable HALL sensors and voltage sensor (LM321 op amp)
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  htim1.Instance = NULL;
  uart_tx_busy = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buf, ADC_BUF_LEN);
  __HAL_DMA_DISABLE_IT(hadc.DMA_Handle, DMA_IT_HT);  // disable half-transfer interrupt

  HAL_TIM_Base_Start_IT(&htim3);

  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

#ifndef SLIM_BINARY
  /* EEPROM Init */
  if (EE_Init() != HAL_OK) {
	  // Initing FLASH failed! This should not happen! We will try to continue anyway by setting default values
	  motor_set_default_settings();
  } else {
	  motor_load_settings();
  }
#else
  motor_set_default_settings();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Enable HALL sensors and voltage sensor (LM321 op amp)
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);

  // Start UART receiver in DMA mode
  uart_start_rx_DMA();

  motor_init();

#ifndef SLIM_BINARY
  blink_led(500,2);
#endif

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (blink) {
    uint8_t _blink = blink;
    blink = 0;

#ifndef SLIM_BINARY
		blink_led(100,_blink);
#endif
	}

	motor_process();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 256;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 80;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* UART1 IDLE Interrupt Configuration */
  SET_BIT(USART1->CR1, USART_CR1_IDLEIE);

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HIGH_2_GATE_Pin|HIGH_1_GATE_Pin|PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HIGH_2_GATE_Pin HIGH_1_GATE_Pin PWR_EN_Pin */
  GPIO_InitStruct.Pin = HIGH_2_GATE_Pin|HIGH_1_GATE_Pin|PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_1_OUT_Pin */
  GPIO_InitStruct.Pin = HALL_1_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_1_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_2_OUT_Pin */
  GPIO_InitStruct.Pin = HALL_2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_2_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT_Pin */
  GPIO_InitStruct.Pin = BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
