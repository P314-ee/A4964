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

#include "A4964.h"
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
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
uint32_t rbs_diagnostic;
uint32_t rbs_motor_speed;
uint32_t rbs_current;
uint32_t rbs_voltage;
uint32_t rbs_temperature;
uint32_t rbs_demand_input;
uint32_t rbs_peak_duty;
uint32_t rbs_phase_adv;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Definition of the 29:Readback Select registers as a complete write command. 
#define REG_RBS_AMP ((uint16_t) 0b1110110000000101)   // RBS[2:0] [ 010 ] Avr. supply current
#define REG_RBS_VOLT ((uint16_t)0b1110110000000110) // RBS[2:0] [ 011 ] Supply Voltage
#define REG_RBS_TEMP ((uint16_t) 0b1110110000001001)  // RBS[2:0] [ 100 ] Chip temperatute 
#define REG_RBS_APADV ((uint16_t) 0b1110110000001110)  // RBS[2:0] [ 111 ] Applied phase advance
#define REG_RBS_MSPEED ((uint16_t) 0b1110110000000011)  // RSB[2:0] [ 001 ] Motor speed
// Definition of the 25: System register, the OPM 1 for the 'Stand-alone with SPI'
#define REG_OPM_1 ((uint16_t) 0b1100101001000000)  // REG_25_SYS_OPM
// Definition of the 31: Read Only regiter 
#define REG_RD_ONLY ((uint16_t) 0b1111100000000000) // First 15-11 bits are adress 10-0 value in bin
#define REG_RBS0_DIAG ((uint16_t) 0b1110110000000001)  //Definitioin of the Redback Select REG for DIAG output 


// ------------VARIABLES------------
// rdng stays for "reading value"

uint16_t rdng_amp;
uint16_t rdng_volt;
uint16_t rdng_temp;
uint16_t rdng_apadv;
uint16_t rdng_mspeed;
uint16_t rdng_diag;

// ____________VARIABLES END____________

void voltage(){
  uint16_t sendData = REG_RBS_VOLT;
  uint16_t rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  

  HAL_Delay(2);
  sendData = REG_RD_ONLY;
  rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  
}

void temperature(){
  uint16_t sendData = REG_RBS_TEMP;
  uint16_t rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  

  HAL_Delay(2);
  sendData = REG_RD_ONLY;
  rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  
}

void applied_phase_advance(){
// // This function prints out temperature value in serial
//   SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3)); // Start SPI dialog with settings for A4964
//   digitalWrite(PIN_CS, LOW);
//   SPI.transfer16(REG_RBS_APADV);  // Write the RBS in temperature rading mode
//   digitalWrite(PIN_CS, HIGH);

//   HAL_Delay(2);

//   SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3)); // Start SPI dialog with settings for A4964
//   digitalWrite(PIN_CS, LOW);
//   rdng_apadv = SPI.transfer16(REG_RD_ONLY);  // Read the register 31
//   digitalWrite(PIN_CS, HIGH);
}

void motor_speed(){
// // This function prints out motor speed value in serial
//   SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3)); // Start SPI dialog with settings for A4964
//   digitalWrite(PIN_CS, LOW);
//   SPI.transfer16(REG_RBS_MSPEED);  // Write the RBS in temperature rading mode
//   digitalWrite(PIN_CS, HIGH);

//   HAL_Delay(2);

//   SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3)); // Start SPI dialog with settings for A4964
//   digitalWrite(PIN_CS, LOW);
//   rdng_mspeed = SPI.transfer16(REG_RD_ONLY);  // Read the register 31
//   digitalWrite(PIN_CS, HIGH);
}

void diagnostics(){
  
  uint16_t sendData = REG_RBS0_DIAG;
  uint16_t rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  

  HAL_Delay(2);
  sendData = REG_RD_ONLY;
  rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  
}

void set_OPM_1(){

  
  uint16_t sendData = REG_OPM_1;
  uint16_t rvData = 0;
  
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&sendData, (uint8_t*)rvData, 1, HAL_MAX_DELAY); 
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_SET);
  
}
aaaaaaaaaaaaaaaaaaaaaaaaaaaa
void loop() { 
  set_OPM_1();
//  diagnostics();
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(A4964_WAKEUP_GPIO_Port, A4964_WAKEUP_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);

  HAL_GPIO_WritePin(A4964_WAKEUP_GPIO_Port, A4964_WAKEUP_Pin, GPIO_PIN_SET);
  A4964_init2();
  HAL_Delay(500);
  A4964_ReadAllRegs();
  A4964_Demand_Input (510);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // read all parameter
        // loop();
        rdng_amp = A4964_ReadBack_Select(RBS_CURRENT);
        rdng_volt = A4964_ReadBack_Select(RBS_VOLTAGE);
        rdng_temp = A4964_ReadBack_Select(RBS_TEMPERATURE);
        rdng_mspeed = A4964_ReadBack_Select(RBS_MOTOR_SPEED);
        rdng_diag = A4964_ReadBack_Select(RBS_DIAGNOSTIC);
        HAL_Delay(500); // 500 ms
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}
aaaaa
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A4964_WDOG_Pin|A4964_WAKEUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A4964_CS_GPIO_Port, A4964_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : A4964_WDOG_Pin A4964_WAKEUP_Pin */
  GPIO_InitStruct.Pin = A4964_WDOG_Pin|A4964_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : A4964_CS_Pin */
  GPIO_InitStruct.Pin = A4964_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A4964_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A4964_DIAG_Pin */
  GPIO_InitStruct.Pin = A4964_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A4964_DIAG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t SPI_Exchange_Byte(uint16_t data)
{
    uint16_t rx;
    uint16_t tx = data;
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&tx, (uint8_t*)&rx, 1, HAL_MAX_DELAY); 
    return rx;
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
