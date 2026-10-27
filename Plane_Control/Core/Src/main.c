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
#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include "ds1307_for_stm32_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MANUAL = 0,
    AUTOPILOT = 1,
} control_mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_MAX_DELAY 1000
#define ADC_MAX_DELAY 1000

#define readMask  0x80
#define writeMask 0x00

// Bank 0
#define ICM_REG_WHO_AM_I              0x00
#define ICM_REG_USER_CTRL             0x03
#define ICM_REG_LP_CONFIG             0x05
#define ICM_REG_PWR_MGMT_1            0x06
#define ICM_REG_ACCEL_XOUT_H          0x2D
#define ICM_REG_EXT_SLV_SENS_DATA_00  0x3B
#define ICM_REG_BANK_SEL              0x7F

// Bank 3
#define ICM_REG_I2C_MST_ODR_CONFIG    0x00
#define ICM_REG_I2C_MST_CTRL          0x01
#define ICM_REG_I2C_SLV0_ADDR         0x03
#define ICM_REG_I2C_SLV0_REG          0x04
#define ICM_REG_I2C_SLV0_CTRL         0x05
#define ICM_REG_I2C_SLV0_DO           0x06

// Magnetometr AK09916
#define AK09916_I2C_ADDR              0x0C
#define AK09916_ST1                   0x10
#define AK09916_CNTL2                 0x31
#define AK09916_CNTL3                 0x32

// Gyroscope
#define ICM_REG_GYRO_XOUT_H 0x33
#define ICM_REG_GYRO_XOUT_L 0x34

#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG 0x01
#define GYRO_RANGE 0x02

// RTC
#define DS1307_ADDR (0x68 << 1)
#define DEC_TO_BCD(val)    ((((val) / 10) << 4) | ((val) % 10))


// Other definitions
#define FONT Font_6x8



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t TMP102_ADDR = 0x48 << 1; // Use 8-bit address
static const uint8_t REG_TEMP = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
control_mode_t mode = 0;
volatile char mode_rx[1];
int16_t probeX;
uint8_t tx_buf[2];
uint8_t rx_buf[2];
int16_t magX, magY, magZ;
float gyro_angle = 0.0f;
uint16_t last_time = 0;


void ICM_select_bank(uint8_t bank){
	uint8_t tx_buf[2];
	tx_buf[0] = writeMask | ICM_REG_BANK_SEL;
    tx_buf[1] = bank << 4;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void ICM_write_reg(uint8_t REG_NAME, uint8_t val){
	uint8_t tx_buf[2];
	tx_buf[0] = writeMask | REG_NAME;
	tx_buf[1] = val;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx_buf, 2, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

}

void ICM_read_reg(uint8_t REG_NAME, uint8_t *buffer, uint8_t len){
	uint8_t reg_addr[2];
	reg_addr[0] = readMask | REG_NAME;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, reg_addr, 1, 100);
	HAL_SPI_Receive(&hspi1, buffer, len, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void ICM_mag_write(uint8_t reg, uint8_t val) {
    ICM_select_bank(3);
    ICM_write_reg(ICM_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    ICM_write_reg(ICM_REG_I2C_SLV0_REG, reg);
    ICM_write_reg(ICM_REG_I2C_SLV0_DO, val);
    ICM_write_reg(ICM_REG_I2C_SLV0_CTRL, 0x80 | 1);
    HAL_Delay(20);
}

uint16_t ADC_read(ADC_HandleTypeDef adc_port){
	HAL_ADC_Start(&adc_port);
	HAL_ADC_PollForConversion(&adc_port, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&adc_port);
}

float TMP_read(I2C_HandleTypeDef i2c_port, uint8_t SENSOR_ADDR, uint8_t REG_ADDR){
	HAL_StatusTypeDef ret;
	uint8_t buf_tmp[16];
	uint16_t val;
	float temp_c;

	buf_tmp[0] = REG_ADDR;


	ret = HAL_I2C_Master_Transmit(&i2c_port, SENSOR_ADDR, buf_tmp, 1, 200);
	if ( ret != HAL_OK ) {
		temp_c = 4096.0; // Tx error flag
	}
	else {
		// Read 2 bytes from the temperature register
		ret = HAL_I2C_Master_Receive(&i2c_port, SENSOR_ADDR, buf_tmp, 2, 200);
		  if ( ret != HAL_OK ) {
		  	temp_c = -4096.0; // Rx error flag
		  }
		  else {
		  	//Combine the bytes
		  	val = ((int16_t)buf_tmp[0] << 4) | (buf_tmp[1] >> 4);
		  	// Convert to 2's complement, since temperature can be negative
		  	if ( val > 0x7FF ) {
		  	    val |= 0xF000;
		  	}
		  	// Convert to float temperature value (Celsius)
		  	temp_c = val * 0.0625;
		  	// Convert temperature to decimal format
		  	temp_c *= 100;
		  }
	}
	return(temp_c);
}

void toggleMode(void){
	mode = (control_mode_t)!mode;
}

HAL_StatusTypeDef DS1307_SetTime(I2C_HandleTypeDef *i2c_port, uint8_t year, uint8_t month, uint8_t date, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    uint8_t buffer[8];

    // 1. Pierwszy bajt to adres rejestru startowego w pamięci DS1307
    buffer[0] = 0x00;

    // 2. Konwersja danych na format BCD i wypełnienie bufora
    buffer[1] = DEC_TO_BCD(sec) & 0x7F;       // Reg 0x00: Sekundy, Bit 7 (CH) = 0 (Zegar działa)
    buffer[2] = DEC_TO_BCD(min);              // Reg 0x01: Minuty
    buffer[3] = DEC_TO_BCD(hour) & 0x3F;      // Reg 0x02: Godziny (Bit 6 = 0 wymusza tryb 24-godzinny)
    buffer[4] = DEC_TO_BCD(day);               // Reg 0x03: Dzień tygodnia (1-7)
    buffer[5] = DEC_TO_BCD(date);              // Reg 0x04: Dzień miesiąca
    buffer[6] = DEC_TO_BCD(month);             // Reg 0x05: Miesiąc
    buffer[7] = DEC_TO_BCD(year);              // Reg 0x06: Rok (dwie ostatnie cyfry)

    // 3. Wysłanie całej paczki danych przez HAL_I2C_Master_Transmit
    // Wysyłamy 8 bajtów (1 bajt adresu rejestru + 7 bajtów danych czasu)
    return HAL_I2C_Master_Transmit(i2c_port, DS1307_ADDR, buffer, 8, HAL_MAX_DELAY);
}

int16_t gyro_read(void){
	int16_t gyroXRead;
	ICM_read_reg(ICM_REG_GYRO_XOUT_H, rx_buf, 1);
	gyroXRead = rx_buf[0] << 8;
	ICM_read_reg(ICM_REG_GYRO_XOUT_L, rx_buf, 1);
	gyroXRead |= rx_buf[0];

	return(gyroXRead);
}

void Update_Gyro_Angle(void) {
    uint32_t current_time = HAL_GetTick();

    float dt = (float)(current_time - last_time) / 1000.0f;
    last_time = current_time;

    if (dt > 0.1f) dt = 0.0f;

    // Odczyt prawidłowej wartości int16_t ze znakiem
    int16_t raw_gyro = gyro_read();

    // Pojedyncze przeliczenie na dps dla zakresu 1000 dps
    float gyro_dps = (float)raw_gyro / 32.8f;

    // Całkowanie
    gyro_angle += gyro_dps * dt;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	uint16_t rawPot1;
	uint16_t rawPot2;
	uint16_t servoControl;
	uint16_t motorControl;


	char msg[80];
	char tx_buffer[30];
	const char* mode_names[] = {
	    "MANUAL",
	    "AUTO"
	};
	char retVal;
	char dispText[100];


	uint8_t buf[6];
	uint8_t mag_buf[9];
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;





	float potentiometer1Mapped = 0;
	float potentiometer2Mapped = 0;

	float servoDegrees;
	float motorPercentage;
	float tempC;
	//float gyroVal;


	//uint8_t currentVal;

	 	// to hold communication errors if such occur
	 			// to hold the character string for UART communication or the bytes from the TMP102 chip				// to hold raw value from the TMP102 chip
				// to hold the temperature value in Celcius
	char tempDispText[24];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */



  // Initialization of timers, interrupts and oled
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_UART_Receive_IT (&huart2, (uint8_t*)mode_rx, 1);

  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  //-----------------------ACCELEROMETER-----------------------------
  // Select register bank 0 and reset the sensor
  ICM_select_bank(0);
  HAL_Delay(5);
  ICM_write_reg(ICM_REG_PWR_MGMT_1, 0x80);
  HAL_Delay(100);

  // 1. Read sensor register and check it
  do {
  	ICM_read_reg(ICM_REG_WHO_AM_I, rx_buf, 1);
  	sprintf(msg, "I am 0x%02X\r\n", rx_buf[0]);
  	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  	HAL_Delay(50);
  } while (rx_buf[0] != 0xEA);

  // 2. Sensor Wakeup and I2C configuration
  ICM_write_reg(ICM_REG_PWR_MGMT_1, 0x01);          // Wakeup, Auto clock speed
  HAL_Delay(10);

  // 3. set up gyroscope parameters - 1000 dps
  ICM_select_bank(2);
  ICM_write_reg(GYRO_SMPLRT_DIV, 0x00);
  ICM_write_reg(GYRO_CONFIG, GYRO_RANGE << 1);
  ICM_select_bank(0);
  /*
  ICM_read_reg(ICM_REG_GYRO_XOUT_H, rx_buf, 1);
  gyroXRead = rx_buf[0] << 8;
  ICM_read_reg(ICM_REG_GYRO_XOUT_L, rx_buf, 1);
  gyroXRead |= rx_buf[0];

  gyroXVal = (float)gyroXRead/1000.0f;
  sprintf(msg, "Angle: %.1f\r\n", gyroXVal);


  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 200);
  */


  /*
  ICM_write_reg(ICM_REG_LP_CONFIG, 0x40);           // Turn on I2C_MST_CYCLE (actvates I2C_MST_ODR_CONFIG!)
  ICM_write_reg(ICM_REG_USER_CTRL, 0x30);           // SPI only (0x10) + I2C Master Enable (0x20)

  ICM_select_bank(3);
  ICM_write_reg(ICM_REG_I2C_MST_CTRL, 0x17);        // Clock I2C 345.6 kHz + STOP signal between readings
  ICM_write_reg(ICM_REG_I2C_MST_ODR_CONFIG, 0x04);  // Odświeżanie I2C Master ~68.75 Hz

  // 3. Magnetometer reset and set up of continous read mode with f = 100 Hz
  ICM_mag_write(AK09916_CNTL3, 0x01);               // Soft reset AK09916
  ICM_mag_write(AK09916_CNTL2, 0x08);               // Continuous read mode 4 (100 Hz)

  // 4. SLV0 Read 9 bytes from 0x10
  ICM_write_reg(ICM_REG_I2C_SLV0_ADDR, readMask | AK09916_I2C_ADDR);
  ICM_write_reg(ICM_REG_I2C_SLV0_REG, AK09916_ST1);
  ICM_write_reg(ICM_REG_I2C_SLV0_CTRL, 0x80 | 9);

  ICM_select_bank(0);

  */

  DS1307_Init(&hi2c1);
  DS1307_SetClockHalt(0);
  /*
  DS1307_SetDate(26);
  DS1307_SetMonth(10);
  DS1307_SetYear(2026);
  DS1307_SetDayOfWeek(1);
  DS1307_SetHour(15);
  DS1307_SetMinute(21);
  DS1307_SetSecond(0);
  */


  /*
  if (DS1307_SetTime(&hi2c1, 26, 10, 26, 2, 14, 11, 0) == HAL_OK) {
	  	  sprintf(tx_buffer, "Setting time completed");

      } else {
    	  sprintf(tx_buffer, "dupa");
          // Błąd komunikacji I2C - sprawdź rezystory pull-up lub połączenia SCL/SDA
      }
  HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //--------------------------------------------------
	  // Read values from potetiometers, map it and set PWM signal accordingly


	  // Potentiometer 1 read and servo controll
	  rawPot1 = ADC_read(hadc1);
	  potentiometer1Mapped = (((float)rawPot1 / 4096.0f) * 1500.0f) + 750.0f;
	  servoControl = (uint16_t)potentiometer1Mapped;
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servoControl);

	  // Potentiometer 2 read and motor controll
	  rawPot2 = ADC_read(hadc2);
	  potentiometer2Mapped = (((float)rawPot2 / 4096.0f) * 1000.0f);
	  motorControl = (uint16_t)potentiometer2Mapped;

	  // If value of PWM is 25% or less, do not set output timer
	  // This is due to the fact that it might damage microcontroller
	  if (motorControl > 250){
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, motorControl);
	  }
	  else{
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	  }


	  //--------------------------------------------------
	  // Read data from magnetometer and send it over UART to laptop

	  // Read data from magnetometer (and accel)
	  /*
	  ICM_read_reg(ICM_REG_ACCEL_XOUT_H, buf, 2);
	  probeX = (int16_t)((buf[0] << 8) | buf[1]);
	  ICM_read_reg(ICM_REG_EXT_SLV_SENS_DATA_00, mag_buf, 9);
	  if ((mag_buf[0] & 0x01) && !(mag_buf[8] & 0x08)) {
	        magX = (int16_t)((mag_buf[2] << 8) | mag_buf[1]);
	        magY = (int16_t)((mag_buf[4] << 8) | mag_buf[3]);
	        magZ = (int16_t)((mag_buf[6] << 8) | mag_buf[5]);
	  }
		*/
	  Update_Gyro_Angle();
	  //gyroVal = gyro_angle;

	  // 4. Wysłanie danych przez UART
	  sprintf(msg, "AccX:%hd | Mag X:%hd\r\n", probeX, magX);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  //--------------------------------------------------
	  //status = HAL_I2C_Mem_Read(&hi2c1, TMP102_ADDR << 1, REG_TEMP, I2C_MEMADD_SIZE_8BIT, tempC, 2, 200);
	  tempC = TMP_read(hi2c1, TMP102_ADDR, REG_TEMP);


	  //--------------------------------------------------
	  // Display relevant data on oled

	  servoDegrees = ((float)rawPot1 / 4096.0f) * 180.0f;
	  motorPercentage = ((float)rawPot2 / 4096.0f) * 100.0f;

	  sprintf(dispText, "%5.1f deg | %5.1f %%", servoDegrees, motorPercentage);
	  ssd1306_SetCursor(1, 1);
	  retVal = ssd1306_WriteString(dispText, FONT, White);

	  ssd1306_SetCursor(1, 12);
	  sprintf(tempDispText, " %u.%02u C  |  %s    ", ((unsigned int)tempC / 100), ((unsigned int)tempC % 100), mode_names[mode]);
	  retVal = ssd1306_WriteString(tempDispText, FONT, White);

	  ssd1306_SetCursor(1, 23);
	  seconds = DS1307_GetSecond();
	  minutes = DS1307_GetMinute();
	  hours = DS1307_GetHour();
	  sprintf(dispText, "%02u:%02u:%02u  |  %3.1f", hours, minutes, seconds, gyro_angle);
	  retVal = ssd1306_WriteString(dispText, FONT, White);

	  ssd1306_UpdateScreen();

	  // Wysyłanie przez UART

	  //HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
	  //HAL_Delay(100);

	  //--------------------------------------------------
	  sprintf(msg, "Angle: %.1f\r\n", gyro_angle);


	  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 200);

	  //HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
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
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ChipSelect_Pin */
  GPIO_InitStruct.Pin = ChipSelect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ChipSelect_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if((GPIO_PIN = GPIO_PIN_13)){
		toggleMode();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(mode_rx[0] == 'A'){
		mode = 1;
	}
	else if(mode_rx[0] == 'M'){
		mode = 0;
	}

	HAL_UART_Receive_IT(&huart2, (uint8_t*)mode_rx, 1);

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
