/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Conversion factor for voltage to torque mapping
#define VOLTAGE_TO_TORQUE_FACTOR 0.17f
// Torque resolution based on ADC resolution (12 bits)
#define TORQUE_RESOLUTION 0.0244f
// Filter coefficient (adjust as needed)
#define FILTER_ALPHA 0.1f
#define ALPHA 0.1f

// Global variable
float torque_Value;
float torqueVoltage;
float voltage;
float filteredTorqueValue = 0.0f;  // Initialize the filtered torque value
uint32_t adcValue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float applyLowPassFilter(float rawValue, float previousFilteredValue) {
	  return (FILTER_ALPHA * rawValue) + ((1.0f - FILTER_ALPHA) * previousFilteredValue);
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
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */

	// Declare adcValue variable here



		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
		while (1)
		{
			// Start ADC conversion
			if (HAL_ADC_Start(&hadc3) != HAL_OK) {
				Error_Handler();
			}

			// Wait for conversion to complete
			if (HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY) != HAL_OK) {
				Error_Handler();
			}

			// Read ADC value
			 adcValue = HAL_ADC_GetValue(&hadc3);

			// Convert ADC value to voltage (assuming 3.3V reference voltage)
			 voltage = adcValue * (3.3f / 4096.0f);

			// Apply conversion factor to map voltage to torque sensor range
			 torqueVoltage = voltage * VOLTAGE_TO_TORQUE_FACTOR;



			filteredTorqueValue = applyLowPassFilter(torqueVoltage, filteredTorqueValue);
			// Calculate torque value based on the torque resolution
			 torque_Value = torqueVoltage * TORQUE_RESOLUTION  ;

			// Apply signal filtering techniques here if needed

			// Now 'torqueValue' contains the calculated torque value in Nm
		}
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */


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
		__HAL_RCC_PWR_CLK_ENABLE();
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
				|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
		{
			Error_Handler();
		}
	}

	/**
	 * @brief ADC3 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_ADC3_Init(void)
	{

		/* USER CODE BEGIN ADC3_Init 0 */

		/* USER CODE END ADC3_Init 0 */

		ADC_ChannelConfTypeDef sConfig = {0};

		/* USER CODE BEGIN ADC3_Init 1 */

		/* USER CODE END ADC3_Init 1 */

		/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
		 */
		hadc3.Instance = ADC3;
		hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
		hadc3.Init.Resolution = ADC_RESOLUTION_12B;
		hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
		hadc3.Init.ContinuousConvMode = ENABLE;
		hadc3.Init.DiscontinuousConvMode = DISABLE;
		hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc3.Init.NbrOfConversion = 1;
		hadc3.Init.DMAContinuousRequests = DISABLE;
		hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		if (HAL_ADC_Init(&hadc3) != HAL_OK)
		{
			Error_Handler();
		}

		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		/* USER CODE BEGIN ADC3_Init 2 */

		/* USER CODE END ADC3_Init 2 */

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
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin : USER_IN_Pin */
		GPIO_InitStruct.Pin = USER_IN_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(USER_IN_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN MX_GPIO_Init_2 */
		/* USER CODE END MX_GPIO_Init_2 */
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
		__disable_irq();
		while (1)
		{

		}
	}
	/* USER CODE END Error_Handler_Debug */

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
