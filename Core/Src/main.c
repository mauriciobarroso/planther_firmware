/* USER CODE BEGIN Header */
/**
 **************************
 * @file           : main.c
 * @brief          : Main program body
 **************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 **************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "shtc3.h"
#include "bh1750.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	SENSORS_GROUP_1 = 0x0,
	SENSORS_GROUP_2 = 0x1,
	SENSORS_GROUP_3 = 0x2
} sensors_group_t;

typedef enum {
	SENSORS_THRESHOLD_NO_EVENT   = 0x0,
	SENSORS_THRESHOLD_TEMP_LOW   = 0x1,
	SENSORS_THRESHOLD_TEMP_HIGH  = 0x2,
	SENSORS_THRESHOLD_SOIL_LOW   = 0x4,
	SENSORS_THRESHOLD_SOIL_HIGH  = 0x8,
	SENSORS_THRESHOLD_HUM_LOW    = 0x10,
	SENSORS_THRESHOLD_HUM_HIGH   = 0x20,
	SENSORS_THRESHOLD_LIGHT_LOW  = 0x40,
	SENSORS_THRESHOLD_LIGHT_HIGH = 0x80,
} sensors_threshold_t;

typedef enum {
	SENSORS_EVENT_1 = SENSORS_THRESHOLD_TEMP_HIGH | SENSORS_THRESHOLD_SOIL_LOW,   /* Thermal and water stress */
	SENSORS_EVENT_2 = SENSORS_THRESHOLD_LIGHT_HIGH | SENSORS_THRESHOLD_SOIL_LOW,  /* Light and water stress */
	SENSORS_EVENT_3 = SENSORS_THRESHOLD_SOIL_LOW,                                 /* Water stress */
	SENSORS_EVENT_4 = SENSORS_THRESHOLD_TEMP_LOW | SENSORS_THRESHOLD_SOIL_HIGH,   /* Cold waterlogging */
	SENSORS_EVENT_5 = SENSORS_THRESHOLD_TEMP_HIGH | SENSORS_THRESHOLD_HUM_HIGH,   /* Heat stress and illness */
	SENSORS_EVENT_6 = SENSORS_THRESHOLD_TEMP_LOW | SENSORS_THRESHOLD_HUM_LOW,     /* Risk of dehydration due to cold */
	SENSORS_EVENT_7 = SENSORS_THRESHOLD_LIGHT_HIGH | SENSORS_THRESHOLD_TEMP_HIGH, /* Thermal and light stress */
	SENSORS_EVENT_8 = SENSORS_THRESHOLD_NO_EVENT                                  /* Optimal conditions */
} sensors_event_t;

typedef struct {
	uint16_t high;
	uint16_t low;
} sensors_levels_t;

typedef struct {
	sensors_levels_t soil;	/* Soil moisture */
	sensors_levels_t temp;	/* Temperature */
	sensors_levels_t hum;	/* Relative humidity */
	sensors_levels_t light;	/* Ambient light */
} sensors_thresholds_t;

typedef struct {
	uint16_t soil;	/* Soil moisture */
	uint16_t temp;	/* Temperature */
	uint16_t hum;	/* Relative humidity */
	uint16_t light;	/* Ambient light */
} sensors_values_t;

typedef struct {
	sensors_thresholds_t thresholds;
	sensors_values_t values;
} sensors_t;

typedef enum {
	LED_COLOR_OFF          = 0x000000,
    LED_COLOR_RED          = 0x001000, // (0,  16, 0)
    LED_COLOR_ORANGE       = 0x081800, // (8,  24, 0)
    LED_COLOR_TURQUOISE    = 0x100010, // (16, 0,  16)
    LED_COLOR_BLUE         = 0x000010, // (0,  0,  16)
    LED_COLOR_YELLOW       = 0x101000, // (16, 16, 0)
    LED_COLOR_PURPLE       = 0x001010, // (0,  16, 16)
    LED_COLOR_WHITE        = 0x0B0B0B, // (10, 10, 10)
    LED_COLOR_GREEN        = 0x100000, // (16, 0,  0)
} led_color_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define UART_DEBUG

/* Set the Timeout value */
#define TIMEOUT_DEFAULT	(uint16_t)(9999) /* 10 s */

#define SOIL_MOISTURE_MAX 1800
#define SOIL_MOISTURE_MIN 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

LPTIM_HandleTypeDef hlptim1;
LPTIM_HandleTypeDef hlptim2;

TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim15_ch1;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
static shtc3_t shtc3;
static bh1750_t bh1750;

volatile uint16_t pwm_buf[24 + 50];
volatile int pwm_flag = 0;

static uint8_t threshold = 0;

static sensors_t sensors;

static uint32_t events[8] = {
		SENSORS_EVENT_1, /* Thermal and water stress */
		SENSORS_EVENT_2, /* Light and water stress */
		SENSORS_EVENT_3, /* Water stress */
		SENSORS_EVENT_4, /* Cold waterlogging */
		SENSORS_EVENT_5, /* Heat stress and illness */
		SENSORS_EVENT_6, /* Risk of dehydration due to cold */
		SENSORS_EVENT_7, /* Thermal and light stress */
		SENSORS_EVENT_8  /* Optimal conditions */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TSC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
static void MX_LPTIM2_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
static void stop2_enter(void);
static void perif_stop(void);
static void perif_start(void);
static void buzzer_set(bool enable);
static void led_set (led_color_t color);

static void sensors_set_thresholds(sensors_t *sensors);
static uint8_t sensors_get_threshold(sensors_t *sensors);
static uint32_t sensors_process_event(sensors_t *sensors);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart4, (uint8_t *)&ch,1, 1000);
	return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_LPTIM1_Init();
  MX_TSC_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_LPTIM2_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n*** System init successfully ***\r\n");

  shtc3_init(&shtc3, &hi2c1, SHTC3_I2C_ADDR);
  bh1750_init(&bh1750, &hi2c1, BH1750_I2C_ADDR);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	perif_start();

	/* Read the sensors */
	sensors_set_thresholds(&sensors);
	threshold = sensors_get_threshold(&sensors);
	threshold = 25;
	printf("soil:%d  temp:%d, hum:%d  light:%d  threshold:%d\r\n",
		  sensors.values.soil, sensors.values.temp, sensors.values.hum, sensors.values.light, threshold);

	perif_stop();

    /* Set the alarms status */
	sensors_process_event(&sensors);

    stop2_enter();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.Period = 65535;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  hlptim1.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief LPTIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM2_Init(void)
{

  /* USER CODE BEGIN LPTIM2_Init 0 */

  /* USER CODE END LPTIM2_Init 0 */

  LPTIM_OC_ConfigTypeDef sConfig1 = {0};

  /* USER CODE BEGIN LPTIM2_Init 1 */

  /* USER CODE END LPTIM2_Init 1 */
  hlptim2.Instance = LPTIM2;
  hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim2.Init.Period = 4 - 1;
  hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim2.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig1.Pulse = 1;
  sConfig1.OCPolarity = LPTIM_OCPOLARITY_HIGH;
  if (HAL_LPTIM_OC_ConfigChannel(&hlptim2, &sConfig1, LPTIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig1.Pulse = 1;
  sConfig1.OCPolarity = LPTIM_OCPOLARITY_LOW;
  if (HAL_LPTIM_OC_ConfigChannel(&hlptim2, &sConfig1, LPTIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM2_Init 2 */

  /* USER CODE END LPTIM2_Init 2 */
  HAL_LPTIM_MspPostInit(&hlptim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 20 - 1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim15, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.ChannelIOs = TSC_GROUP7_IO2;
  htsc.Init.SamplingIOs = TSC_GROUP7_IO1;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SYSCFG_DisableTSCComparatorMode();
  /* USER CODE BEGIN TSC_Init 2 */
  HAL_TSC_IODischarge(&htsc, ENABLE);
  HAL_Delay(1); /* 1 ms is more than enough to discharge all capacitors */

  if (HAL_TSC_Start(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEL_EN_GPIO_Port, SEL_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB0 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_EN_Pin */
  GPIO_InitStruct.Pin = LED_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NRST_Pin BOOT0_Pin */
  GPIO_InitStruct.Pin = NRST_Pin|BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_1_Pin */
  GPIO_InitStruct.Pin = SEL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SEL_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_0_Pin */
  GPIO_InitStruct.Pin = SEL_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SEL_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_EN_Pin */
  GPIO_InitStruct.Pin = SEL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEL_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim) {
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, TIMEOUT_DEFAULT);
}

static void stop2_enter(void)
{
	HAL_SuspendTick();
	HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);
	SystemClock_Config();
	HAL_ResumeTick();
}

static void perif_stop(void)
{
	HAL_TSC_DeInit(&htsc);
	HAL_UART_DeInit(&huart4);
	HAL_I2C_DeInit(&hi2c1);
//	HAL_TIM_PWM_DeInit(&htim15);
//	HAL_DMA_DeInit(&hdma_tim15_ch1);
//	__HAL_RCC_DMA1_CLK_DISABLE();

	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWREx_EnableUltraLowPowerMode();
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

	//L011 only has Ports A B and C
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_ALL;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_ALL;
	GPIO_InitStructure.Pin &= ~(GPIO_PIN_2);
	GPIO_InitStructure.Pin &= ~(GPIO_PIN_4);
	GPIO_InitStructure.Pin &= ~(GPIO_PIN_7);
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_ALL;
	GPIO_InitStructure.Pin &= ~(GPIO_PIN_14);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOF_CLK_DISABLE();
}

static void perif_start(void)
{
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TSC_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_USART4_UART_Init();
}

static void sensors_set_thresholds(sensors_t *sensors)
{
	HAL_GPIO_WritePin(SEL_EN_GPIO_Port, SEL_EN_Pin, GPIO_PIN_SET);

	uint8_t sel0 = HAL_GPIO_ReadPin(SEL_0_GPIO_Port, SEL_0_Pin) & 0x1;
	uint8_t sel1 = (HAL_GPIO_ReadPin(SEL_1_GPIO_Port, SEL_1_Pin) << 1) & 0x2;

	HAL_GPIO_WritePin(SEL_EN_GPIO_Port, SEL_EN_Pin, GPIO_PIN_RESET);

	sensors_group_t group = sel0 | sel1;

	printf("mode:%d\r\n", group);

	switch (group) {
	case SENSORS_GROUP_1:
		sensors->thresholds.soil.low = 10;
		sensors->thresholds.soil.high = 20;
		sensors->thresholds.hum.low = 30;
		sensors->thresholds.hum.high = 50;
		sensors->thresholds.temp.low = 18;
		sensors->thresholds.temp.high = 30;
		sensors->thresholds.light.high = 30000;
		break;
	case SENSORS_GROUP_2:
		sensors->thresholds.soil.low = 40;
		sensors->thresholds.soil.high = 60;
		sensors->thresholds.hum.low = 50;
		sensors->thresholds.hum.high = 80;
		sensors->thresholds.temp.low = 20;
		sensors->thresholds.temp.high = 28;
		sensors->thresholds.light.high = 10000;
		break;
	case SENSORS_GROUP_3:
		sensors->thresholds.soil.low = 20;
		sensors->thresholds.soil.high = 60;
		sensors->thresholds.hum.low = 30;
		sensors->thresholds.hum.high = 70;
		sensors->thresholds.temp.low = 15;
		sensors->thresholds.temp.high = 30;
		sensors->thresholds.light.high = 10000;
		break;
	default:
		break;
	}
}

static uint8_t sensors_get_threshold(sensors_t *sensors)
{
	/* Read soil moisture sensor */
	uint32_t soil = 0;
	HAL_TSC_IODischarge(&htsc, ENABLE);
	HAL_Delay(1); /* 1 ms is more than enough to discharge all capacitors */

	if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED) {
		for (uint8_t i = 0; i < 32; i++) {
			soil += HAL_TSC_GroupGetValue(&htsc, TSC_GROUP5_IDX);
		}
	}

	uint16_t tmp = (uint16_t)(soil >>= 5);

	if (tmp >= SOIL_MOISTURE_MAX) {
		sensors->values.soil = 0;
	}
	else if (tmp <= SOIL_MOISTURE_MIN) {
		sensors->values.soil = 100;
	}
	else {
		sensors->values.soil = (SOIL_MOISTURE_MAX - soil) * 100 / (SOIL_MOISTURE_MAX - SOIL_MOISTURE_MIN);
	}



	/* Read temperature and humidity sensor*/
	float temp, hum;
	shtc3_get_temp_and_hum(&shtc3, &temp, &hum);
	shtc3_sleep(&shtc3);
	sensors->values.temp = (uint16_t)temp;
	sensors->values.hum = (uint16_t)hum;

	/* Read ambient light sensors */
	float light;
	bh1750_power_on(&bh1750);
	bh1750_get_light_intensity(&bh1750, BH1750_MEAS_MODE_ONETIME_4LX_RES, &light);
	bh1750_power_down(&bh1750);
	sensors->values.light = (uint16_t)light;

	/* Compare the current sensors values against their thresholds  and get
	 * the current active thresholds */
	uint8_t threshold = 0;

	if (sensors->values.temp < sensors->thresholds.temp.low) {
		threshold |= SENSORS_THRESHOLD_TEMP_LOW;
	}
	else if (sensors->values.temp > sensors->thresholds.temp.high) {
		threshold |= SENSORS_THRESHOLD_TEMP_HIGH;
	}

	if (sensors->values.hum < sensors->thresholds.hum.low) {
		threshold |= SENSORS_THRESHOLD_HUM_LOW;
	}
	else if (sensors->values.hum > sensors->thresholds.hum.high) {
		threshold |= SENSORS_THRESHOLD_HUM_HIGH;
	}

	if (sensors->values.soil < sensors->thresholds.soil.low) {
		threshold |= SENSORS_THRESHOLD_SOIL_LOW;
	}
	else if (sensors->values.soil > sensors->thresholds.soil.high) {
		threshold |= SENSORS_THRESHOLD_SOIL_HIGH;
	}

	if (sensors->values.light > sensors->thresholds.light.high) {
		threshold |= SENSORS_THRESHOLD_LIGHT_HIGH;
	}

	return threshold;
}

static uint32_t sensors_process_event(sensors_t *sensors)
{
	uint32_t led_time = 100;
	uint32_t led_color = LED_COLOR_OFF;
	uint32_t event = SENSORS_EVENT_8;
	uint32_t timeout = 9999;
	bool buzzer_en = true;

	for (uint8_t i = 0; i < 8; i++) {
		if ((events[i] & threshold) == events[i]) {
			event = events[i];
			break;
		}
	}

	/**/
	switch (event) {
	case SENSORS_EVENT_1:
		led_color = LED_COLOR_BLUE;
		break;
	case SENSORS_EVENT_2:
		led_color = LED_COLOR_ORANGE;
		break;
	case SENSORS_EVENT_3:
		led_color = LED_COLOR_PURPLE;
		break;
	case SENSORS_EVENT_4:
		led_color = LED_COLOR_RED;
		break;
	case SENSORS_EVENT_5:
		led_color = LED_COLOR_TURQUOISE;
		break;
	case SENSORS_EVENT_6:
		led_color = LED_COLOR_WHITE;
		break;
	case SENSORS_EVENT_7:
		led_color = LED_COLOR_YELLOW;
		break;
	case SENSORS_EVENT_8:
		led_color = LED_COLOR_GREEN;
		led_time = 200;
		buzzer_en = false;
		timeout = 29330;
		break;
	default:
		break;
	}

	/**/
	led_set(led_color);
	buzzer_set(buzzer_en);
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, led_time);
	stop2_enter();
	buzzer_set(false);
	led_set(LED_COLOR_OFF);

	/**/
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, timeout);
}

static void buzzer_set(bool enable)
{
	if (enable) {
		HAL_LPTIM_PWM_Start(&hlptim2, LPTIM_CHANNEL_1);
		HAL_LPTIM_PWM_Start(&hlptim2, LPTIM_CHANNEL_2);
	}
	else {
		HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
		HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_2);
	}
}

static void led_set(led_color_t color)
{
	if (color != LED_COLOR_OFF) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_SET);
		HAL_Delay(1);

		uint8_t g = (uint8_t)((color & 0xFF0000) >> 16);
		uint8_t r = (uint8_t)((color & 0x00FF00) >> 8);
		uint8_t b = (uint8_t)(color & 0x0000FF);
		uint8_t delta = 1;

		if (sensors.values.light < 100) {
			delta = 8;
		}
		else if (sensors.values.light < 1000) {
			delta = 4;
		}

		color = ((g / delta) << 16) | ((r / delta) << 8) | (b / delta);

		uint32_t pwm_buf_len = 0;

		for (int i = 23; i >= 0; i--)
		{
			if (color & (1 << i)) {
				pwm_buf[pwm_buf_len] = 14;  // 2/3 of 90
			}
			else {
				pwm_buf[pwm_buf_len] = 6;  // 1/3 of 90
			}

			pwm_buf_len++;
		}

		for (int i = 0; i < 50; i++) {
			pwm_buf[pwm_buf_len++] = 0;
		}

		HAL_TIM_PWM_Start_DMA(&htim15, TIM_CHANNEL_1, (uint32_t *)pwm_buf, pwm_buf_len);

		while (!pwm_flag) {}
	}
	else {
		HAL_TIM_PWM_DeInit(&htim15);
		HAL_DMA_DeInit(&hdma_tim15_ch1);
		__HAL_RCC_DMA1_CLK_DISABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
		__HAL_RCC_GPIOC_CLK_DISABLE();
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim15, TIM_CHANNEL_1);
	pwm_flag = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
