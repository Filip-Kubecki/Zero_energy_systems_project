/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l1xx_hal_def.h"
#include "stm32l1xx_hal_uart.h"
#include "string.h"

#include "ILPS28QSW.h"
#include "HDC3022-Q1.h"
#include "TMP119.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static char UART_TX_BUFFER[64]; // Buffor do komunikacji UART - wysyłania
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uint8_to_bin_str(uint8_t value, char* output){
    for (int i = 7; i >= 0; i--){
        uint8_t mask = (1 << i);

        if (value & mask){
        	output[7 - i] = '1';
        }
        else{
        	output[7 - i] = '0';
        }
    }
    output[8] = '\0';
}

void sep(){
	HAL_UART_Transmit(&huart2, (uint8_t*)"------------------\r\n", 20, HAL_MAX_DELAY);
};

void temperature_sensor_ID(){
	uint16_t device_id_16b;		// Zmienna przetrzymująca odczyt z rejestru Device_ID

	HAL_StatusTypeDef status_id = TMP119_read_device_id(&device_id_16b);

	if (status_id == HAL_OK){
	// Usunięcie 3 pierwszych bitów - pozostawienie ID urządzenia
		uint16_t did  = device_id_16b & 0x1FFF;

		// Nagłówek
		sprintf(UART_TX_BUFFER, "TMP119:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);

		// Wysłanie przez UART ID urządzenia
		sprintf(UART_TX_BUFFER, "Device DID: %X\r\n", did);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);

		// Usunięcie pierwszych 13 bitów - pozostawienie numeru rewizji
		uint16_t rev  = device_id_16b & 0xE000;
		rev = rev >> 13;

		// Wysłanie przez UART numeru rewizji urzadzenia
		sprintf(UART_TX_BUFFER, "Device REV: %X\r\n\r\n", rev);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
	}
	else{
		// W przypadku gdy nie uda się odczytać danych przez I2C
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
};

void temperature_sensor_read_temperature(float* temp){
	HAL_StatusTypeDef status = TMP119_read_temperature(temp);

	if (status == HAL_OK){
		sprintf(UART_TX_BUFFER, "Temperature: %.2f C\r\n", *temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
	}
	else{
		// W przypadku gdy nie uda się odczytać danych przez I2C
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
};

void pressure_sensor_ID(){
	uint8_t device_id;		// Zmienna przetrzymująca odczyt z rejestru Device_ID
	HAL_StatusTypeDef status_id = ILPS28QSW_read_who_am_i(&device_id);

	if (status_id == HAL_OK){
		// Nagłówek
		sprintf(UART_TX_BUFFER, "ILPS28QSW:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);

		// Wysłanie przez UART ID urządzenia
		sprintf(UART_TX_BUFFER, "Device DID: 0x%X\r\n\r\n", device_id);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
	}
	else{
		// W przypadku gdy nie uda się odczytać danych przez I2C
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
};

void pressure_sensor_ctrl_regs(){
	uint8_t ctrl_reg_data[3];
	HAL_StatusTypeDef status_id = ILPS28QSW_read_ctrl_regs(ctrl_reg_data);

	if (status_id == HAL_OK){
		sprintf(UART_TX_BUFFER, "ILPS28QSW control registers:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);

		for (int i = 0; i < 3; ++ i) {
			char binary_string_buffer[9];
			uint8_to_bin_str(ctrl_reg_data[i], binary_string_buffer);
			sprintf(UART_TX_BUFFER, "CTRL_REG%d: %s\r\n", (i+1), binary_string_buffer);
			HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
		}
	}
	else{
		// W przypadku gdy nie uda się odczytać danych przez I2C
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
};

void pressure_sensor_read_pressure(float* pressure){
	HAL_StatusTypeDef status = ILPS28QSW_read_pressure(pressure);
//	HAL_StatusTypeDef status = HAL_OK;

	if (status == HAL_OK){
		sprintf(UART_TX_BUFFER, "Pressure: %.2f hPa\r\n", *pressure);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
	}
	else{
		// W przypadku gdy nie uda się odczytać danych przez I2C
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
}

void pressure_sensor_init(){
	HAL_StatusTypeDef status_init = ILPS28QSW_init();

	if (status_init == HAL_OK){
		// Jeśli inicjalizacja się nie powiodła, wyświetl błąd
		HAL_UART_Transmit(&huart2, (uint8_t*)"Pressure INIT complete\r\n", 24, HAL_MAX_DELAY);
	}else{
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
}

void humidity_sensor_read_id(){
	uint16_t device_id;		// Zmienna przetrzymująca odczyt z rejestru Device_ID
  HAL_StatusTypeDef status = HDC3022_read_device_id(&device_id);

  if(status == HAL_OK){
		sprintf(UART_TX_BUFFER, "HDC3022_Q1:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);

		sprintf(UART_TX_BUFFER, "Device manufacturer ID: 0x%X\r\n\r\n", device_id);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
  }else{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Erro\r\n", 32, HAL_MAX_DELAY);
  }
}

void humidity_sensor_read_humidity(float* humidity, float* temp){
	HAL_StatusTypeDef status = HDC3022_read_humidity_and_temperature(humidity, temp);

	if (status == HAL_OK){
		sprintf(UART_TX_BUFFER, "Humidity: %.2f %% RH, %.2f C \r\n", *humidity, *temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
	}
	else{
		// W przypadku gdy nie uda się odczytać danych przez I2C
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	sep(); // Linia do oddzielnenia kolejnych komunikatów UART

  // I2C reseet
  HDC3022_general_call_reset();
  // Soft reset czujnika wilgotności
  HDC3022_soft_reset();

  // Zczytaj identyfikator i rewizje czujnika temperatury - TMP119
	temperature_sensor_ID();


  // Zczytaj identyfikator i rewizje czujnika ciśnienia - ILPS28QSW
	pressure_sensor_ID();
  // Zczytaj wartości rejestrów controli czujnika ciśnienia
  // informacje o trybach pracy/pozwala sterować czujnikiem
	pressure_sensor_ctrl_regs();
  // Inicjalizacja pracy w trybie 1 [Hz]
	pressure_sensor_init();

  // Zczytaj identyfikator producenta czujnika wilgotności - HDC3022-Q1
  humidity_sensor_read_id();

	sep(); // Linia do oddzielnenia kolejnych komunikatów UART

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	float	temp		= 0.0f;		// Wartość temperatury
	float	pressure	= 0.0f;		// Wartość ciśnienia
	float	humidity	= 0.0f;		// Wartość ciśnienia


	// Odczyt temperatury z sensora TMP119
	temperature_sensor_read_temperature(&temp);

	// Odczyt ciśnienia z sensora ILPS28QSW
	pressure_sensor_read_pressure(&pressure);

	// Odczyt wilgotności z sensora HDC3022-Q1
  humidity_sensor_read_humidity(&humidity, &temp);

	HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
