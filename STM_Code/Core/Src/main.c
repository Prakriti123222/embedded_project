/* USER CODE BEGIN Header */
/**
 **********
 * @file           : main.c
 * @brief          : Main program body
 **********
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 **********
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Define ADC handle and configuration structures
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig;
char buffer[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float mmse(float x, float y, int n) {
	float sum = 0.0, diff;
	diff = x - y;
	return diff * diff;
	return sum / n;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// Initialize HAL, clock, ADC and USB
	HAL_Init();
	SystemClock_Config();
	MX_ADC1_Init();
	MX_USB_DEVICE_Init();

	// Start ADC conversion
	HAL_ADC_Start(&hadc1);
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
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// used for collecting samples from sensor
	float beats[600] = { 0 };

	// heart rate of normal person
	float base[600] = { 80.00, 69.30, 69.00, 70.93, 72.87, 68.40, 71.33, 73.70,
			74.57, 71.80, 73.37, 71.27, 67.37, 69.43, 67.77, 72.60, 74.13,
			71.77, 70.97, 73.47, 71.20, 67.57, 69.93, 68.13, 66.47, 65.87,
			64.83, 64.20, 67.83, 65.90, 66.63, 64.73, 65.63, 65.60, 64.80,
			63.83, 65.43, 64.67, 63.93, 65.70, 66.30, 66.23, 65.00, 66.87,
			66.30, 64.23, 66.70, 66.47, 65.00, 64.57, 65.57, 62.67, 64.23,
			66.47, 62.73, 65.97, 65.63, 66.80, 64.30, 69.97, 64.33, 67.97,
			66.67, 65.40, 65.13, 67.47, 66.03, 64.67, 65.37, 64.77, 65.67,
			64.87, 69.57, 65.77, 64.20, 64.67, 66.77, 65.27, 66.73, 68.20,
			64.33, 65.63, 66.03, 65.23, 67.70, 68.17, 69.63, 67.43, 67.83,
			68.53, 67.90, 67.03, 68.80, 66.70, 66.63, 66.87, 66.07, 66.50,
			70.43, 68.83, 65.80, 66.37, 66.40, 67.70, 72.07, 70.77, 68.90,
			68.57, 67.40, 69.73, 68.13, 72.27, 71.23, 70.50, 69.90, 69.60,
			70.43, 71.20, 73.77, 67.93, 70.53, 70.47, 72.40, 71.67, 73.23,
			71.87, 70.87, 70.40, 72.97, 70.53, 73.23, 72.43, 73.33, 73.43,
			73.10, 74.03, 73.47, 74.33, 75.10, 75.13, 74.17, 75.67, 74.07,
			79.07, 75.07, 76.67, 76.70, 75.50, 77.57, 79.37, 79.10, 75.57,
			78.23, 77.53, 75.83, 76.80, 81.10, 75.43, 77.20, 78.37, 77.13,
			78.57, 81.07, 76.33, 78.40, 78.70, 76.83, 79.87, 79.50, 76.80,
			78.37, 80.50, 80.37, 83.03, 81.63, 80.77, 83.37, 84.00, 81.43,
			84.13, 81.07, 79.33, 78.83, 85.43, 81.07, 83.60, 76.63, 78.80,
			75.57, 76.90, 77.03, 78.50, 77.03, 74.80, 78.70, 76.07, 75.27,
			80.57, 75.37, 76.33, 76.30, 76.17, 75.97, 77.80, 75.80, 77.83,
			78.93, 78.00, 80.57, 82.40, 73.83, 77.80, 80.03, 79.13, 80.60,
			79.93, 77.50, 78.23, 80.13, 80.13, 81.40, 77.60, 76.60, 81.67,
			79.13, 78.03, 80.13, 75.07, 77.80, 75.77, 75.27, 79.07, 81.17,
			85.10, 81.73, 75.00, 74.70, 78.87, 77.47, 74.90, 73.27, 79.57,
			80.17, 82.27, 77.40, 81.20, 80.97, 79.13, 79.97, 83.57, 80.33,
			80.30, 80.73, 80.97, 79.73, 81.90, 78.70, 81.27, 79.27, 79.43,
			81.87, 81.50, 79.33, 78.47, 80.37, 79.63, 81.80, 82.70, 78.00,
			79.77, 80.70, 77.93, 79.83, 83.23, 79.57, 78.40, 81.17, 78.30,
			81.17, 83.00, 79.97, 81.73, 81.07, 82.97, 80.43, 84.33, 79.53,
			80.97, 79.47, 78.77, 80.20, 81.63, 78.87, 79.10, 80.13, 82.17,
			81.37, 78.47, 84.57, 80.33, 82.53, 83.47, 81.57, 81.07, 83.23,
			78.13, 80.57, 80.40, 79.73, 80.57, 83.27, 75.17, 83.00, 81.10,
			74.87, 85.50, 83.93, 77.03, 77.77, 78.80, 76.97, 81.17, 82.17,
			76.13, 82.23, 80.57, 78.73, 81.03, 83.10, 77.07, 79.80, 79.17,
			78.13, 82.03, 81.00, 79.30, 78.17, 78.30, 80.33, 79.83, 79.60,
			79.17, 81.33, 79.60, 80.27, 82.63, 79.60, 79.97, 78.93, 77.80,
			77.77, 80.03, 81.13, 79.17, 78.43, 80.03, 78.60, 80.33, 84.37,
			79.47, 83.87, 80.83, 79.50, 82.77, 84.93, 79.67, 79.93, 80.20,
			78.77, 80.10, 84.17, 78.80, 80.53, 80.00, 80.17, 81.80, 83.87,
			80.83, 79.00, 80.23, 81.40, 81.30, 81.20, 83.83, 76.07, 85.33,
			85.10, 84.00, 85.60, 89.83, 85.37, 83.50, 85.70, 84.87, 84.67,
			86.90, 87.17, 84.37, 85.87, 86.13, 83.63, 87.97, 83.10, 81.23,
			81.83, 83.47, 80.23, 81.23, 83.70, 79.97, 82.63, 81.00, 81.07,
			82.17, 82.70, 81.07, 81.50, 78.70, 80.70, 79.07, 81.33, 82.93,
			78.90, 81.87, 81.30, 80.43, 83.53, 85.50, 81.37, 83.30, 80.80,
			81.83, 82.47, 84.87, 81.63, 79.47, 83.30, 84.50, 82.93, 85.13,
			80.40, 80.60, 81.13, 80.47, 82.87, 80.87, 79.47, 81.90, 81.27,
			82.40, 81.20, 79.27, 77.93, 80.47, 80.13, 83.30, 78.13, 80.83,
			80.37, 81.93, 82.03, 82.27, 77.63, 76.87, 80.23, 78.83, 82.90,
			80.87, 79.10, 79.70, 79.40, 79.60, 79.53, 82.77, 78.93, 78.27,
			79.10, 81.63, 80.70, 84.50, 80.70, 79.73, 82.37, 80.97, 82.50,
			84.33, 84.83, 83.90, 83.33, 82.57, 79.33, 79.70, 83.63, 81.60,
			79.33, 79.73, 82.43, 81.27, 85.57, 81.17, 81.07, 82.30, 82.63,
			81.47, 83.03, 86.30, 82.13, 83.20, 83.83, 77.43, 84.57, 88.03,
			83.73, 84.83, 86.90, 85.33, 88.37, 90.90, 87.73, 84.03, 87.83,
			86.47, 85.80, 87.67, 86.20, 85.17, 85.70, 86.70, 85.33, 84.63,
			88.17, 86.43, 83.83, 84.63, 83.87, 85.33, 86.73, 86.33, 83.63,
			85.97, 85.17, 82.60, 83.37, 87.90, 85.37, 85.87, 84.80, 83.90,
			85.57, 85.97, 88.27, 88.03, 83.53, 84.17, 85.43, 86.63, 85.77,
			83.30, 86.37, 86.43, 84.03, 86.80, 84.77, 87.13, 86.27, 85.77,
			92.33, 93.60, 82.30, 85.60, 83.30, 84.10, 85.93, 84.67, 86.70,
			83.43, 85.07, 86.20, 84.47, 86.00, 84.17, 86.60, 87.10, 83.97,
			83.57, 85.20, 84.07, 84.47, 87.17, 87.10, 85.33, 84.17, 86.30,
			84.70, 86.10, 89.23, 84.37, 84.63, 84.30, 83.00 };
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		while (HAL_GPIO_ReadPin(GPIOB, input_Pin) == 1) {
			HAL_GPIO_WritePin(GPIOB, blue_Pin, SET); // on when start reading

			int high = 0;  // to check for mmse

			int limit_cross_low = 0;  // to check how many values were <60
			int limit_cross_high = 0;    // to check how many values were >100

			int curr = 0;
			while (curr < 600 && HAL_GPIO_ReadPin(GPIOB, input_Pin) == 1) {

				HAL_ADC_Start(&hadc1); // start the adc
				HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion
				// Read analog value from ADC
				uint16_t value = HAL_ADC_GetValue(&hadc1);

				beats[curr] = (float) value;

				// checking for limit_crosses
				if (beats[curr] < 50.0) {
					limit_cross_low++;
				} else if (beats[curr] > 110.0) {
					limit_cross_high++;
				}

				// transmitting data to the serial terminal
				sprintf(buffer, "%d\n", value);
				CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
				curr++;
				if (curr < 600) {
					HAL_Delay(100);
				}
			}

			HAL_GPIO_WritePin(GPIOB, blue_Pin, RESET);
			HAL_Delay(100);

			// for mmse matching
			int i = 0;
			while (i < 600 && HAL_GPIO_ReadPin(GPIOB, input_Pin) == 1) {
				float mmse_curr = mmse(base[curr], beats[curr], 600);
				if (mmse_curr > 10.0) {
					high++;
				}
				i++;
			}

			if (HAL_GPIO_ReadPin(GPIOB, input_Pin) == 1) {
				// result of mmse calculation
				if (high > 50) {
					HAL_GPIO_WritePin(GPIOB, red_Pin, SET);
					HAL_GPIO_WritePin(GPIOB, green_Pin, RESET);
					HAL_GPIO_WritePin(GPIOB, blue_Pin, RESET);
				} else {
					HAL_GPIO_WritePin(GPIOB, green_Pin, SET);
					HAL_GPIO_WritePin(GPIOB, red_Pin, RESET);
					HAL_GPIO_WritePin(GPIOB, blue_Pin, RESET);
				}

				// result of threshold calculation
				if (limit_cross_low > 10) {
					HAL_GPIO_WritePin(GPIOF, low_hr_Pin, SET);
					HAL_GPIO_WritePin(GPIOE, high_hr_Pin, RESET);
				}
				if (limit_cross_high > 10) {
					HAL_GPIO_WritePin(GPIOE, high_hr_Pin, SET);
					HAL_GPIO_WritePin(GPIOF, low_hr_Pin, RESET);
				}

				HAL_Delay(10000);
				HAL_GPIO_WritePin(GPIOB, red_Pin, RESET);
				HAL_GPIO_WritePin(GPIOB, green_Pin, RESET);
				HAL_GPIO_WritePin(GPIOB, blue_Pin, RESET);
				HAL_GPIO_WritePin(GPIOF, low_hr_Pin, RESET);
				HAL_GPIO_WritePin(GPIOE, high_hr_Pin, RESET);
			}

		}

		// if enable is disconnected, resetting all LEDs
		HAL_GPIO_WritePin(GPIOB, red_Pin, RESET);
		HAL_GPIO_WritePin(GPIOB, green_Pin, RESET);
		HAL_GPIO_WritePin(GPIOB, blue_Pin, RESET);
		HAL_GPIO_WritePin(GPIOF, low_hr_Pin, RESET);
		HAL_GPIO_WritePin(GPIOE, high_hr_Pin, RESET);
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */
	// Initialize ADC handle and configuration structures
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 0;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	// Initialize ADC channel configuration structure
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfig.Offset = 0;

	// Configure ADC and ADC channel
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

	/* USER CODE BEGIN ETH_Init 0 */

	/* USER CODE END ETH_Init 0 */

	static uint8_t MACAddr[6];

	/* USER CODE BEGIN ETH_Init 1 */

	/* USER CODE END ETH_Init 1 */
	heth.Instance = ETH;
	MACAddr[0] = 0x00;
	MACAddr[1] = 0x80;
	MACAddr[2] = 0xE1;
	MACAddr[3] = 0x00;
	MACAddr[4] = 0x00;
	MACAddr[5] = 0x00;
	heth.Init.MACAddr = &MACAddr[0];
	heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
	heth.Init.TxDesc = DMATxDscrTab;
	heth.Init.RxDesc = DMARxDscrTab;
	heth.Init.RxBuffLen = 1524;

	/* USER CODE BEGIN MACADDRESS */

	/* USER CODE END MACADDRESS */

	if (HAL_ETH_Init(&heth) != HAL_OK) {
		Error_Handler();
	}

	memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
	TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM
			| ETH_TX_PACKETS_FEATURES_CRCPAD;
	TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
	/* USER CODE BEGIN ETH_Init 2 */

	/* USER CODE END ETH_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, green_Pin | red_Pin | blue_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(low_hr_GPIO_Port, low_hr_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(high_hr_GPIO_Port, high_hr_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : green_Pin red_Pin blue_Pin */
	GPIO_InitStruct.Pin = green_Pin | red_Pin | blue_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : low_hr_Pin */
	GPIO_InitStruct.Pin = low_hr_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(low_hr_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : high_hr_Pin */
	GPIO_InitStruct.Pin = high_hr_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(high_hr_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : input_Pin */
	GPIO_InitStruct.Pin = input_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(input_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
