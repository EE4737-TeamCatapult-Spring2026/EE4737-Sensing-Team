/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "hx711.h"
#include "hts221.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {OK, BUSY, SENSING_ERROR} SensingStatus;
typedef enum {NoError, ThermistorError, CurrentSensorError, HumiditySensorError, StrainGaugeError, HardFault} SensingErrorStatus;
typedef enum {NoRisk, FrozenPipeRisk, RoofStrainRisk, BothRisks} RiskStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRAIN_A_LBS 13956.6f
#define STRAIN_B_LBS 3489.15f

#define SLAVE_RX_BUFFER_SIZE  1
#define SLAVE_TX_BUFFER_SIZE  16
#define SENSING_DELAY 100

#define STATUS_RQ 0x01
#define INFO_RQ 0x02
#define RESET 0x03
#define E_STOP 0x04

#define SOLAR_V_REF 0.33f
#define SOLAR_V_DIV_CONV 21.0f
#define SOLAR_V_OFFSET 1.0f
#define SOLAR_RESISTANCE 210000
#define OP_AMP_SCALE 51
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
HX711 strainScale;
HTS221_HandleTypeDef humiditySensor;

uint8_t rxBuffer[SLAVE_RX_BUFFER_SIZE];
uint8_t txBuffer[SLAVE_TX_BUFFER_SIZE];
//volatile uint8_t newPacketReceived = 0;

// Strain gauge offset variables
float offsetA, offsetB = 0.0f;

// Initialize error/risk status variables
// Setup is incomplete, so the slice is busy until a first round of data is collected
SensingStatus sensingSliceStatus = BUSY;
SensingErrorStatus errorStatus = NoError;
RiskStatus riskStatus = NoRisk;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void ReconfigChannel(ADC_HandleTypeDef adc, uint32_t channel);
void ProcessI2CPacket(float temp, float humidity, float strain, float power);
void SensorErrorCheck(float temp1, float temp2, float humidity, float strainA, float strainB, float solar_v, float solar_i);
void RiskCheck(float temp, float humidity, float strain);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize SPI comms
  MX_SPI1_Init();

  // Enables the interrupt-driven listener to loaf
//  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
//    {
//      /* Transfer error in reception process */
//      Error_Handler();
//    }
  HAL_I2C_Slave_Receive_IT(&hi2c1, rxBuffer, SLAVE_RX_BUFFER_SIZE);

  // Initialize thermistor variables
  uint32_t adc_therm_1 = 0u;
  uint32_t adc_therm_2 = 0u;
  float therm_1_volt = 0.0f;
  float therm_2_volt = 0.0f;
  uint16_t adc_max_range = 4095;
  float therm_ref_volt = 3.3f;
  uint16_t R2 = 51000;
  float R_therm_1 = 0.0f;
  float R_therm_2 = 0.0f;
  float T_therm_1 = 0.0f;
  float T_therm_2 = 0.0f;

  // Initialize power sensor variables
  float solar_current = 0.0f;
//  float solar_current_adc = 0.0f;
  float solar_voltage = 0.0f;
  float solar_voltage_adc = 0.0f;
//  float solar_sensitivity = (float)(25 * (10^-3));

  // ADC and other value computation variables
  float volt_per_bit = therm_ref_volt/(float)adc_max_range; //conversion for bits to volts
  float avgTemp = 0.0f;
  float avgStrain = 0.0f;
  float solar_power = 0.0f;

  // Configure CLK and Data pins
  hx711_init(&strainScale, GPIOA, GPIO_PIN_11,   // CLK
                          GPIOA, GPIO_PIN_8,   // DAT
                          HX711_GAIN_A_128);

  // Initialize HTS221
  HTS221_Init(&humiditySensor, &hspi1);

  // Tare channel A strain gauge
  strainScale.gain = HX711_GAIN_A_128;
  hx711_read_raw(&strainScale);  // dummy to arm channel A
  hx711_tare(&strainScale, 10);
  offsetA = strainScale.offset;  // save channel A offset

  // Tare channel B strain gauge
  strainScale.gain = HX711_GAIN_B_32;
  hx711_read_raw(&strainScale);  // dummy to arm channel B
  hx711_tare(&strainScale, 10);
  offsetB = strainScale.offset;  // save channel B offset

  // Initialize strain gauge values
  volatile float strainA, strainB = 0.0f;

  // Setup complete, set sensing slice status as OK
  sensingSliceStatus = OK;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // --- ADC1: Thermistor 1 on IN4 ---
	 ReconfigChannel(hadc1, ADC_CHANNEL_4);
	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	 adc_therm_1 = HAL_ADC_GetValue(&hadc1);
	 therm_1_volt = volt_per_bit * adc_therm_1;
	 R_therm_1 = (therm_ref_volt * R2 / therm_1_volt) - R2;

	 // Final Temp for Thermistor 1
	 T_therm_1 = -21.8f * logf(R_therm_1) + 228.29f;

	 // --- ADC2: Thermistor 2 on IN2 ---
	 ReconfigChannel(hadc2, ADC_CHANNEL_2);
	 HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	 adc_therm_2 = HAL_ADC_GetValue(&hadc2);
	 therm_2_volt = volt_per_bit * adc_therm_2;
	 R_therm_2 = (therm_ref_volt * R2 / therm_2_volt) - R2;

	 // Final Temp for Thermistor 2
	 T_therm_2 = -21.8f * logf(R_therm_2) + 228.29f;

	 // --- GPIO: Strain Gauges ---
	 strainScale.gain = HX711_GAIN_A_128;
	 hx711_set_scale(&strainScale, STRAIN_A_LBS); // Calibrated to pounds
	 strainScale.offset = offsetA;
	 hx711_read_raw(&strainScale);          // dummy read to set channel A for next read

	 // Final Strain for Gauge 1
	 strainA = hx711_get_units(&strainScale, 5);

	 strainScale.gain = HX711_GAIN_B_32;
	 hx711_set_scale(&strainScale, STRAIN_B_LBS); // Calibrated to pounds
	 strainScale.offset = offsetB;
	 hx711_read_raw(&strainScale);          // dummy read to set channel B for next read

	 // Final Strain for Gauge 2
	 strainB = hx711_get_units(&strainScale, 5);

	 // --- GPIO: Humidity Sensor ---
	 // Data stored in humiditySensor.humidity and humiditySensor.temperature
	 HTS221_Read(&humiditySensor);

	 // --- ADC2: Solar Panel Current on IN3 ---
//	 ReconfigChannel(hadc2, ADC_CHANNEL_3);
//	 HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//	 solar_current_adc = volt_per_bit * HAL_ADC_GetValue(&hadc2) / OP_AMP_SCALE;

//	 // Convert current ADC reading to proper scale
//	 solar_current = (solar_current_adc - SOLAR_V_REF) / solar_sensitivity;

	 // --- ADC2: Solar Panel Voltage on IN4 ---
	 ReconfigChannel(hadc2, ADC_CHANNEL_4);
	 HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	 solar_voltage_adc = volt_per_bit * HAL_ADC_GetValue(&hadc2);

	 // If no voltage ADC reading, return 0 volts
	 if(solar_voltage_adc == 0)
	 {
		 solar_voltage = 0;
	 }
	 else // Else, convert voltage ADC reading to proper scale
	 {
		 solar_voltage = (solar_voltage_adc * SOLAR_V_DIV_CONV) + SOLAR_V_OFFSET;
	 }

	 // Compute current based off voltage
	 solar_current = solar_voltage / SOLAR_RESISTANCE;

	 // Process collected data (average temp and strain values, compute power)
	 avgTemp = (T_therm_1 + T_therm_2 + humiditySensor.temperature) / 3;
	 avgStrain = (strainA + strainB) / 2;
	 solar_power =  solar_voltage * solar_current;

	 // Perform checks for sensor errors and risks
	 SensorErrorCheck(T_therm_1, T_therm_2, humiditySensor.humidity, strainA, strainB, solar_voltage, solar_current);
	 RiskCheck(avgTemp, humiditySensor.humidity, avgStrain);

	 // Send I2C packet to loaf
	 ProcessI2CPacket(avgTemp, humiditySensor.humidity, avgStrain, solar_power);

	 // END OF LOOP
	 HAL_Delay(SENSING_DELAY);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0x20;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Reconfigure ADC channel for ADC readings
void ReconfigChannel(ADC_HandleTypeDef adc, uint32_t channel)
{

	// Stop ADC
	HAL_ADC_Stop(&adc);

	// Update configuration to given channel
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&adc, &sConfig) != HAL_OK)
    {
    	Error_Handler();
    }

    // Restart ADC
    HAL_ADC_Start(&adc);

}

// Process an incoming packet from the GUI and send the expected response
void ProcessI2CPacket(float temp, float humidity, float strain, float power)
{
	// Read received packet and determine what is being requested
	if(HAL_I2C_Slave_Receive(&hi2c1, rxBuffer, SLAVE_RX_BUFFER_SIZE, SENSING_DELAY) != HAL_OK)
	{
		// If no data read, skip processing
		return;
	}
	uint8_t rqType = rxBuffer[0];

	switch(rqType)
	{
	case STATUS_RQ:
		// Populate TX buffer with status
		bzero(txBuffer, SLAVE_TX_BUFFER_SIZE);
		memcpy(&txBuffer[0], &sensingSliceStatus, sizeof(sensingSliceStatus));

		// If in error status, concatenate the error being experienced
		if(sensingSliceStatus == SENSING_ERROR)
		{
			memcpy(&txBuffer[1], &errorStatus, sizeof(errorStatus));
		}

		// If at risk, concatenate the risk being experienced
		if(riskStatus != NoRisk)
		{
			memcpy(&txBuffer[2], &riskStatus, sizeof(riskStatus));
		}

		// Transmit packet data
		HAL_I2C_Slave_Transmit(&hi2c1, txBuffer, SLAVE_TX_BUFFER_SIZE / 2, SENSING_DELAY);
		break;

	case INFO_RQ:
		// Populate TX buffer with sensor data
		// Done in order TEMP HUMIDITY STRAIN POWER
		bzero(txBuffer, SLAVE_TX_BUFFER_SIZE);
		memcpy(&txBuffer[0], &temp, sizeof(temp));
		memcpy(&txBuffer[4], &humidity, sizeof(humidity));
		memcpy(&txBuffer[8], &strain, sizeof(strain));
		memcpy(&txBuffer[12], &power, sizeof(power));

		// Transmit packet data
		HAL_I2C_Slave_Transmit_IT(&hi2c1, txBuffer, SLAVE_TX_BUFFER_SIZE);
		break;

	case RESET:
		// De-initialize and re-initialize all sensors, reset I2C/SPI comms
		hx711_power_down(&strainScale);
		hx711_power_up(&strainScale);
		HTS221_Init(&humiditySensor, &hspi1);
	    MX_GPIO_Init();
	    MX_ADC1_Init();
	    MX_ADC2_Init();
	    MX_I2C1_Init();
	    MX_SPI1_Init();
		break;

	case E_STOP:
		// Stop all operations, loop forever until power is cut off
		while(1);
		break;
	}

//	// Reset newPacketReceived
//	newPacketReceived = 0;
	return;
}

/**
 * Performs a 'sanity' check on sensors and their outputted values
 * If any condition is met, the errorStatus and sensingSliceStatus variables are updated
 * Note that only one type of error can be reported at a time!
 */
void SensorErrorCheck(float temp1, float temp2, float humidity, float strainA, float strainB, float solar_v, float solar_i)
{
	// Delay to allow sensors to write values to variables
	HAL_Delay(SENSING_DELAY);

	// Thermistor Error Check
	// When unplugged, thermistors report values of about -48 degrees C
	if((temp1 < -30) || (temp2 < -30))
	{
		errorStatus = ThermistorError;
		return;
	}
	else
	{
		errorStatus = NoError;
	}

	// Current Sensor Error Check
	// If current or voltage is reported to be <= 0, chances of misconfiguration are high
	if(solar_i <= 0 || solar_v <= 0)
	{
		errorStatus = CurrentSensorError;
		return;
	}
	else
	{
		errorStatus = NoError;
	}

	// Humidity Sensor Error Check
	// If humidity values do not change on new read, HTS221 is not reading new values
	float oldHumidity = humidity;
	HTS221_Read(&humiditySensor);
	float newHumidity = humiditySensor.humidity;
	if(newHumidity - oldHumidity == 0)
	{
		errorStatus = HumiditySensorError;
		return;
	}
	else
	{
		errorStatus = NoError;
	}

	// Strain Gauge Error Check
	// If values do not change on new read, HX711 is not reading new values
	 strainScale.gain = HX711_GAIN_A_128;
	 hx711_set_scale(&strainScale, STRAIN_A_LBS); // Calibrated to pounds
	 strainScale.offset = offsetA;
	 hx711_read_raw(&strainScale);          // dummy read to set channel A for next read

	 //Strain for Gauge 1
	 float newStrainA = hx711_get_units(&strainScale, 5);

	 strainScale.gain = HX711_GAIN_B_32;
	 hx711_set_scale(&strainScale, STRAIN_B_LBS); // Calibrated to pounds
	 strainScale.offset = offsetB;
	 hx711_read_raw(&strainScale);          // dummy read to set channel B for next read

	 //Strain for Gauge 2
	 float newStrainB = hx711_get_units(&strainScale, 5);

	 if((newStrainA - strainA == 0) || (newStrainB - strainB == 0))
	 {
	 	 errorStatus = StrainGaugeError;
	 	 return;
	 }
     else
	 {
	 	 errorStatus = NoError;
	 }
}

/**
 * Checks if any risk conditions have been met based on data given
 * If any condition is met, the riskStatus and sensingSliceStatus variables are updated
 */
void RiskCheck(float temp, float humidity, float strain)
{
	// Frozen Pipe Risk check
	// Conditions based off of that water freezes at and below 0 deg. C
	// And that leaking/bursting pipes increase humidity, about 60% according to EPA
	if(((temp <= 0) && (humidity >= 60.0f)) && !((errorStatus == ThermistorError) || (errorStatus == HumiditySensorError)))
	{
		riskStatus |= FrozenPipeRisk;
	}
	else
	{
		riskStatus &= !FrozenPipeRisk;
	}

	// Roof Strain Risk check
	// According to GAF Roofing, roofs can withstand 20 lbs/ft^2
	// Strain gauges are 1.2cm x 7.5cm = 0.00968752 ft^2, which is 0.1937504 lbs per gauge
	// Thus we assume risk occurs at approx. 0.25 lbs per gauge
	// Strain can be negative - for this check, we will take the magnitude
	if(strain < 0)
	{
		strain *= -1;
	}

	if(strain >= 0.25f && !(errorStatus == StrainGaugeError))
	{
		riskStatus |= RoofStrainRisk;
	}
	else
	{
		riskStatus &= !RoofStrainRisk;
	}
}

///**
// * Called when the slave has finished RECEIVING data from master.
// * Re-arm for the next receive, and set a flag for main loop processing.
// */
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == I2C1) {
//        newPacketReceived = 1;
//
//        // Re-arm for next packet
//        HAL_I2C_Slave_Receive_IT(&hi2c1, rxBuffer, SLAVE_RX_BUFFER_SIZE);
//    }
//}
//
///**
// * Called when the slave has finished TRANSMITTING data to master.
// * Re-arm the receiver to listen for the next command.
// */
//void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == I2C1) {
//        // Re-arm to receive the next request
//        HAL_I2C_Slave_Receive_IT(&hi2c1, rxBuffer, SLAVE_RX_BUFFER_SIZE);
//    }
//}
//
///**
// * Called on I2C error (bus error, arbitration loss, timeout, etc.)
// */
//void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == I2C1) {
//        // Reset and re-arm — common for ARLO or BERR recovery
//        HAL_I2C_DeInit(hi2c);
//        HAL_I2C_Init(hi2c);
//        HAL_I2C_Slave_Receive_IT(&hi2c1, rxBuffer, SLAVE_RX_BUFFER_SIZE);
//    }
//}

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
  sensingSliceStatus = SENSING_ERROR;
  errorStatus = HardFault;
  while (1)
  {
//	  if (newPacketReceived)
//	  	 {
	  		 ProcessI2CPacket(0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF);
//	  	 }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
