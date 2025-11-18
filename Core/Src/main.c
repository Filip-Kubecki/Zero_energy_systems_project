/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file              : main.c
  * @brief             : Główny program dla niskoenergetycznego monitora parametrów wnętrza obudowy fotolitografu.
  *
  * @description
  * Ten plik zawiera główną logikę aplikacji dla urządzenia niskiej mocy,
  * zaprojektowanego do monitorowania kluczowych parametrów środowiskowych
  * wewnątrz obudowy maszyny fotolitograficznej.
  *
  * System okresowo budzi się ze stanu niskiego poboru mocy, aby odczytać
  * dane z następujących czujników I2C:
  *
  * - TMP119:       Precyzyjny pomiar temperatury
  * - ILPS28QSW:    Ciśnienie baryczne i temperatura
  * - HDC3022-q1:   Wilgotność i temperatura
  * - TCS3720:      Natężenie światła (kanał Clear) TODO: niech zaczytuje też kanał 3 - BLUE
  * - SEN54:        Jakość powietrza (PM, VOC)
  *
  * Zebrane dane są następnie przesyłane po UART do urządzenia nadawczego
  *
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
#include "SEN54.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"
#include "stm32l1xx_hal_flash_ex.h"
#include "stm32l1xx_hal_pwr.h"
#include "stm32l1xx_hal_uart.h"

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

// Pliki nagłówkowe do driverów sensorów
#include "TCS3720.h"
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
#define SENSOR_READ_CYCLE_TIME  3000      // Ustala co ile wykonuje się pomiar z czujników
#define SEN54_WARMUP_TIME       60000     // Czas rozgrzewania się czujnika SEN54 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

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
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uint8_to_bin_str(uint8_t value, char* output_buffer) {
    for (int i = 7; i >= 0; i--) {
        uint8_t mask = (1 << i);
        if (value & mask) {
            output_buffer[7 - i] = '1';
        } else {
            output_buffer[7 - i] = '0';
        }
    }
    output_buffer[8] = '\0';
}

void sep(const char* value){
    uint16_t len = strlen(value);

    if (len > 0){
        HAL_UART_Transmit(&huart2, (uint8_t*)value, len, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
};

HAL_StatusTypeDef uart_print_check_stat(HAL_StatusTypeDef* status, const char* format, ...){
  // Sprawdza czy status jest poprawny
	if (*status != HAL_OK){
    const char* err_msg = "I2C STATUS ERROR\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
    return *status;
  }

  va_list args;
  va_start(args, format);
  // Długość ciągu znaków na podstawie ilości argumentów i wpisanie argumentów do bufforu UART
  int len = vsprintf(UART_TX_BUFFER, format, args);
  va_end(args);

  // Sprawdza czy podane zostały jakiekolwiek argumenty
  if (len > 0){
      // Wyślij sformatowany string o dokładnej długości
      HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, len, HAL_MAX_DELAY);
  }

  return HAL_OK;
}

// CZUJNIK TEMPERATURY ---------------------------------------------------------
void temperature_sensor_ID(){
	uint16_t device_id;		// Zmienna przetrzymująca identyfikator producenta
	uint8_t rev;		// Zmienna przetrzymująca numer rewizji urządzenia

	HAL_StatusTypeDef status_id = TMP119_read_device_id_and_rev(&device_id, &rev);

  uart_print_check_stat(&status_id, "Device DID: %X\r\nDevice REV: %X \r\n", device_id, rev);
};

void temperature_sensor_init(){
	HAL_StatusTypeDef status = TMP119_init();

	if (status == HAL_OK){
		sprintf(UART_TX_BUFFER, "Initialized correctly.\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)UART_TX_BUFFER, strlen(UART_TX_BUFFER), HAL_MAX_DELAY);
	}
	else{
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Error\r\n", 18, HAL_MAX_DELAY);
	}
}

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

// CZUJNIK CIŚNIENIA -----------------------------------------------------------
void pressure_sensor_ID(){
	uint8_t device_id;		// Zmienna przetrzymująca odczyt z rejestru Device_ID
	HAL_StatusTypeDef status_id = ILPS28QSW_read_who_am_i(&device_id);

  uart_print_check_stat(&status_id, "ILPS28QSW:\r\nDevice DID: 0x%X\r\n", device_id);
};

void pressure_sensor_ctrl_regs(){
	uint8_t ctrl_reg_data[3];
	HAL_StatusTypeDef status_id = ILPS28QSW_read_ctrl_regs(ctrl_reg_data);

  char buffer_str0[9];
  char buffer_str1[9];
  char buffer_str2[9];

  uint8_to_bin_str(ctrl_reg_data[0], buffer_str0);
  uint8_to_bin_str(ctrl_reg_data[1], buffer_str1);
  uint8_to_bin_str(ctrl_reg_data[2], buffer_str2);

  uart_print_check_stat(
    &status_id,
    "Control registers:\r\n%s\r\n%s\r\n%s\r\n",
    buffer_str0,
    buffer_str1,
    buffer_str2
  );
};

void pressure_sensor_read_pressure_and_temp(float* pressure, float* temp){
	HAL_StatusTypeDef status = ILPS28QSW_read_pressure(pressure);

  if(status != HAL_OK){
    sep("ERROR: pressure read I2C error.");
  }

	status = ILPS28QSW_read_temp(temp);

  uart_print_check_stat(&status, "Pressure: %.2f hPa, %.2f C\r\n", *pressure, *temp);
}

void pressure_sensor_read_pressure(float* pressure){
	HAL_StatusTypeDef status = ILPS28QSW_read_pressure(pressure);

  uart_print_check_stat(&status, "Pressure: %.2f hPa\r\n", *pressure);
}

void pressure_sensor_read_temp(float* temp){
	HAL_StatusTypeDef status = ILPS28QSW_read_temp(temp);

  uart_print_check_stat(&status, "Temp: %.2f C\r\n", *temp);
}

void pressure_sensor_init(){
	HAL_StatusTypeDef status = ILPS28QSW_init();

  uart_print_check_stat(&status, "Pressure INIT complete\r\n");
}

// CZUJNIK WILGOCI -----------------------------------------------------------
void humidity_sensor_read_id(){
	uint16_t device_id;		// Zmienna przetrzymująca odczyt z rejestru Device_ID
  HAL_StatusTypeDef status = HDC3022_read_device_id(&device_id);

  uart_print_check_stat(&status, "HDC3022-Q1:\r\nDevice manufactureer ID: 0x%X\r\n", device_id);
}

void humidity_sensor_read_humidity_and_temp(float* humidity, float* temp){
	HAL_StatusTypeDef status = HDC3022_read_humidity_and_temperature(humidity, temp);

  uart_print_check_stat(&status, "Humidity: %.2f %% RH, %.2f C \r\n", *humidity, *temp);
}

// CZUJNIK NATĘŻENIA ŚWIATŁA -------------------------------------------------
void light_sensor_read_id(){
	uint8_t device_id;		// Zmienna przetrzymująca odczyt z rejestru Device_ID
  HAL_StatusTypeDef status = TCS3720_read_device_id(&device_id);

  uart_print_check_stat(&status, "TCS3720:\r\nDevice ID: 0x%X\r\n", device_id);
}

void light_sensor_read_light_intensity(uint16_t* light_intensity){
  HAL_StatusTypeDef status = TCS3720_read_light_intensity(light_intensity);

  uart_print_check_stat(&status, "Light intensity: %u\r\n", *light_intensity);
}

// Czujnik środowiskowy ------------------------------------------------------
void environment_sensor_read_all(float* pm1, float* pm2_5, float* pm4, float* pm10, float* voc, float* temp, float* humidity){
  HAL_StatusTypeDef status = SEN54_read_measurement_values(
    pm1,
    pm2_5,
    pm4,
    pm10,
    voc,
    temp,
    humidity
  );

  uart_print_check_stat(
    &status, 
    "PM1.0: %.2f\r\nPM2.5: %.2f\r\nPM4: %.2f\r\nPM10: %.2f\r\nVOC: %.2f\r\nTEMP: %.2f C\r\nHUMIDITY: %.2f %% RH\r\n",
     *pm1,
     *pm2_5,
     *pm4,
     *pm10,
     *voc,
     *temp,
     *humidity
    );
}

void environment_sensor_wake_up(){
  HAL_StatusTypeDef status = SEN54_start_measurement();

  if (status != HAL_OK) {
    sep("SEN54: error on startup");
  }
}

void environment_sensor_put_to_sleep(){
  HAL_StatusTypeDef status = SEN54_stop_measurement();

  if (status != HAL_OK) {
    sep("SEN54: error when putting to sleep");
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
// Disable clocks for ALL peripherals you are not using
// --- AHB Bus Peripherals ---
  // __HAL_RCC_GPIOA_CLK_DISABLE(); // USED (USART2, LD2, Debug)
  // __HAL_RCC_GPIOB_CLK_DISABLE(); // USED (I2C1, I2C2)
  // __HAL_RCC_GPIOC_CLK_DISABLE(); // USED (B1 Button, LSE)
  __HAL_RCC_GPIOD_CLK_DISABLE(); // Not used
  __HAL_RCC_GPIOE_CLK_DISABLE(); // Not used
  __HAL_RCC_GPIOH_CLK_DISABLE(); // Not used
  __HAL_RCC_GPIOF_CLK_DISABLE(); // Not used
  __HAL_RCC_GPIOG_CLK_DISABLE(); // Not used
  
  __HAL_RCC_DMA1_CLK_DISABLE();  // Not used
  __HAL_RCC_CRC_CLK_DISABLE();   // Not used
  
  // Disable Flash interface clock (CPU will run from cache)
  __HAL_RCC_FLITF_CLK_DISABLE();


  // --- APB1 Bus Peripherals ---
  __HAL_RCC_TIM2_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM4_CLK_DISABLE();
  __HAL_RCC_TIM5_CLK_DISABLE();
  __HAL_RCC_TIM6_CLK_DISABLE();
  __HAL_RCC_TIM7_CLK_DISABLE();
  
  __HAL_RCC_WWDG_CLK_DISABLE();  // Window Watchdog
  __HAL_RCC_SPI2_CLK_DISABLE();
  __HAL_RCC_SPI3_CLK_DISABLE();
  
  // __HAL_RCC_USART2_CLK_DISABLE(); // USED (Communication)
  __HAL_RCC_USART3_CLK_DISABLE();
  __HAL_RCC_UART4_CLK_DISABLE();
  __HAL_RCC_UART5_CLK_DISABLE();
  
  // __HAL_RCC_I2C1_CLK_DISABLE(); // USED (Sensors)
  // __HAL_RCC_I2C2_CLK_DISABLE(); // USED (Sensors)
  
  __HAL_RCC_USB_CLK_DISABLE();
  __HAL_RCC_DAC_CLK_DISABLE();
  // __HAL_RCC_PWR_CLK_ENABLE();  // USED (For STOP Mode)
  // __HAL_RCC_COMP_CLK_ENABLE(); // USED (Part of SYS)


  // --- APB2 Bus Peripherals ---
  // __HAL_RCC_SYSCFG_CLK_ENABLE(); // USED (For EXTI / Interrupts)
  __HAL_RCC_TIM9_CLK_DISABLE();
  __HAL_RCC_TIM10_CLK_DISABLE();
  __HAL_RCC_TIM11_CLK_DISABLE();
  
  __HAL_RCC_ADC1_CLK_DISABLE();
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_USART1_CLK_DISABLE();
  
  // Power down the Flash memory during STOP mode
  __HAL_FLASH_SLEEP_POWERDOWN_ENABLE();

  // Po konfiguracji mikrokontrolera
  // uint32_t last_routine_call_time = 0;
  uint32_t warmup_time_counter = HAL_GetTick();

	sep("---------------------------------------");


  // CZUJNIK ŚRODOWISKOWY -------------------------------------------------------
  // Włącz czujnik - rozpoczyna cykl mierzenia i rozgrzewanie czujnika (min 1 minuta na rozgrzanie)
  environment_sensor_wake_up();

  // CZUJNIK TEMPERATURY -------------------------------------------------------
  // Zczytaj identyfikator i rewizje czujnika temperatury - TMP119
  sep("TMP119:");
  temperature_sensor_init();
	temperature_sensor_ID(); sep("");

  // CZUJNIK CIŚNIENIA ---------------------------------------------------------
  // Zczytaj identyfikator i rewizje czujnika ciśnienia - ILPS28QSW
	pressure_sensor_ID();
  // Zczytaj wartości rejestrów controli czujnika ciśnienia
  // informacje o trybach pracy/pozwala sterować czujnikiem
	pressure_sensor_ctrl_regs();
  // Inicjalizacja pracy w trybie 1 [Hz]
	pressure_sensor_init(); sep("");

  // CZUJNIK WILGOCI -----------------------------------------------------------
  // Zczytaj identyfikator producenta czujnika wilgotności - HDC3022-Q1
  humidity_sensor_read_id(); sep("");

  // CZUJNIK NATĘŻENIA ŚWIATŁA -------------------------------------------------
  light_sensor_read_id(); sep("");

	sep("---------------------------------------");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // DEFINIOWANIE ZMIENNYCH --------------------------------------------
    uint32_t current_time = HAL_GetTick(); 

    float	temp		= 0.0f;		        // Wartość temperatury  [°C]
    float	pressure	= 0.0f;		      // Wartość ciśnienia    [hPa]
    float	humidity	= 0.0f;		      // Wartość wilgotności  [%RH]
    uint16_t light_intensity	= 0;  // Wartość natężenia światła (wartość prosto z czujnika od 0 do 2047 - )
    // Czujnik środowiskowy
    float pm1        = 0.0f;  // Wartość PM1.0 [μg/m³]
    float pm2_5      = 0.0f;  // Wartość PM2.5 [μg/m³]
    float pm4        = 0.0f;  // Wartość PM4 [μg/m³]
    float pm10       = 0.0f;  // Wartość PM10 [μg/m³]
    float voc        = 0.0f;  // Wartość VOC w zakresie 0 - 500 gdzie 100 to wartość normalna (baseline)
                              // baseline jest ustalany na podstawie pomiarów z ostatnich 24 [h] - normalna wartość dla pomieszczenia

    // KOD --------------------------------------------------------------

    // Główna rutyna - pomiar z czujników na magistrali I2C1
    // if (current_time - last_routine_call_time >= SENSOR_READ_CYCLE_TIME) {
      // Zapisz obecny czas do zmiennej
      // last_routine_call_time = current_time;

      // Wykonaj time event - rutyne
    
      // Odczyt temperatury z sensora TMP119
      temperature_sensor_read_temperature(&temp);

      // Odczyt ciśnienia z sensora ILPS28QSW
      pressure_sensor_read_pressure_and_temp(&pressure, &temp);

      // Odczyt wilgotności z sensora HDC3022-Q1
      humidity_sensor_read_humidity_and_temp(&humidity, &temp);

      // Odczyt natężenia światła z sensora TCS3720
      light_sensor_read_light_intensity(&light_intensity);
      sep("");
    // }

    // Poboczna rutyna - odpowiedzialna za pomiar z czujnika SEN54
    if(warmup_time_counter != -1 && (current_time - warmup_time_counter) >= SEN54_WARMUP_TIME ){
      // Zmiana wartości na -1 aby zaznaczyć że poboczna rutyna wykonała się już raz i nie musi wykonywać się ponownie
      // Aby wykonać ją ponownie trzeba zmienić wartość zmiennej na obecny czas
      warmup_time_counter = -1;

      // Odczyt wszystkich wartości z czujnika środowiskowego
      sep("-------------------------------------------------");
      sep("SEN54:");
      environment_sensor_read_all(&pm1,&pm2_5, &pm4, &pm10, &voc, &temp, &humidity);
      environment_sensor_put_to_sleep();
      sep("-------------------------------------------------");
    }

    // Sprawdzenie czy zakończyła się transmisja UART
    while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);

    // === PREPARE FOR SLEEP =======================
    // Zatrzymaj SysTick - inaczej po 1ms wszystko znowu się obudzi
    HAL_SuspendTick();
    // Ustawia alarm zegara RTC na 10s - 0x500B
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x500B, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

    // === STOP MODE ==============================
    // Ustawianie trybu STOP na mikrokontrolerze
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    // === WAKE UP ===============================
    // Reinicjalizacja zegarów mikrokontrolera - bez tego nie zadziałą UART i I2C
    // Po wybudzeniu jedyne zegary jakie działają to te niskoenergetyczne (MSI)
    // jednak większość magistrali nie jest w stanie operować przy tej prędkości
    // i wymaga szybszych zegarów które zostały uśpione
    SystemClock_Config();
    // Przywraca SysTick
    HAL_ResumeTick();
    // Wyłącza zegar RTC aby ponownie nie wysyłał przerwania po określonym czasie
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
