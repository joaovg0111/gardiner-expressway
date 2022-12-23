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
#include "DCMotor.h"
#include "QMC5883.h"
#include "Bluetooth_BLE_V4.2_JDY-18.h"
#include <math.h>
#include <string.h>
#include <Servo.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float x;
  float y;
} Point;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PERIOD 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
QMC_t compass;
Compass_config config;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void getPowersFromReading(ListDevices_t listOfDevices);
int getAverage(int nbOfEntries, int entries[]);
uint8_t isOutlierInArray(int nbOfEntries, int array[], int value);
Point get_position(float rss1, float rss2, float rss3);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t angle = 0;
uint8_t speed = 0;
uint8_t offset = 15;
Point b1 = {-19.866733, -43.964666};
int b1Power = 0;
int b2Power = 0;
int b3Power = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  // Initialize all peripherals
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  // Set up BLE communication and initialize motor and servo
  setupBLE(&huart3, &huart2);
  setDCMotorDirection(0x02);
  initDCMotor(&htim2, TIM_CHANNEL_2, PERIOD);
  set_servo_angle(&htim3,TIM_CHANNEL_2,PERIOD, offset);

  // Initialize variables for PID control
  int8_t error = 0;
  int8_t past_error = 0;
  int8_t P = 0;
  int8_t I = 0;
  int8_t D = 0;
  const float Kp = 1;
  const float Ki = 0.1;
  const float Kd = 1;

  int16_t theta_g = 0;

  // Initialize compass
  compass.ADDR_Control_RegisterA = ADDR_REG_A;
  compass.ADDR_Control_RegisterB = ADDR_REG_B;
  compass.ADDR_Mode_Register = ADDR_REG_MODE;
  compass.ADDR_Status_Register = ADDR_REG_STATUS;

  // Set compass configuration
  config.gain = _1_3;
  config.meas_mode = Positive;
  config.op_mode = Continuous_meas;
  config.output_rate = 15;
  config.samples_num = eight;

  // Initialize compass
  QMC_init(&compass,&hi2c1,&config);

  // Set initial motor speed and delay for 10 seconds
  setDCMotorSpeed(&htim2,TIM_CHANNEL_2,PERIOD,100);
  HAL_Delay(10000);

  // Infinite loop
  while (1) {
	// Read sensors
	ListDevices_t listOfDevices = masterScanForSlaves();
	getPowersFromReading(listOfDevices);
	Point myPosition = get_position(b1Power, b2Power, b3Power);
	free(listOfDevices.devices);
	// Read compass data
	if(!QMC_read(&compass)){
	  theta_g = (int16_t) compass.heading;

	  // Calculate error
	  error = (180.0/M_PI)*(M_PI/2 + atan2((b1.y - myPosition.y), (b1.x - myPosition.x)) - theta_g);

	  // Calculate PID control signal
	  if (error==0){
		I = 0;
	  }
	  P = error;
	  I = I + error;

	  if (I > 45){
		I = 45;
	  }
	  else if (I < -45){
		I = -45;
	  }

	  D = error - past_error;

	  uint8_t PID = (Kp*P) + (Ki*I) + (Kd*D);

	  past_error = error;

	  // Control motors
	  if (PID >= 0){
		angle -= PID;
	  }
	  else{
		angle += PID;
	  }

	  speed = 100;

	  // Set servo angle and motor speed
	  set_servo_angle(&htim3,TIM_CHANNEL_2,PERIOD, angle);
	  setDCMotorSpeed(&htim2,TIM_CHANNEL_2,PERIOD, speed);

	  // Delay for 50 milliseconds
	  HAL_Delay(50);
	}

  }
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 320;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 320;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_LATCH_Pin|DIR_EN_Pin|DIR_SER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_LATCH_Pin DIR_EN_Pin DIR_SER_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_LATCH_Pin|DIR_EN_Pin|DIR_SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_CLK_Pin */
  GPIO_InitStruct.Pin = DIR_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_CLK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void getPowersFromReading(ListDevices_t listOfDevices) {
	static int b1Powers[5] = {0,0,0,0,0};
	static int b2Powers[5] = {0,0,0,0,0};
	static int b3Powers[5] = {0,0,0,0,0};
	static uint8_t currentIndex = 0;

	uint8_t nbOfDevices = listOfDevices.nbOfDevices;
	Device_t* devices = listOfDevices.devices;
	uint8_t b1Changed = 0, b2Changed = 0, b3Changed = 0;

	// Get the powers from entries
	for (uint8_t i=0; i<nbOfDevices; i++) {
		if(strstr(devices[i].name, "B1") != NULL && !isOutlierInArray(5, b1Powers, devices[i].signalStrength)) {
			b1Powers[currentIndex] = devices[i].signalStrength;
			b1Changed = 1;
		} else if (strstr(devices[i].name, "B2")!= NULL && !isOutlierInArray(5, b2Powers, devices[i].signalStrength)) {
			b2Powers[currentIndex] = devices[i].signalStrength;
			b2Changed = 1;
		} else if (strstr(devices[i].name, "B3")!= NULL && !isOutlierInArray(5, b3Powers, devices[i].signalStrength)) {
			b3Powers[currentIndex] = devices[i].signalStrength;
			b3Changed = 1;
		}
	}

	uint8_t prevIndex = (currentIndex > 0) ? currentIndex-1 : 4;
	if(!b1Changed) b1Powers[currentIndex] = b1Powers[prevIndex];
	if(!b2Changed) b2Powers[currentIndex] = b2Powers[prevIndex];
	if(!b3Changed) b3Powers[currentIndex] = b3Powers[prevIndex];

	b1Power = getAverage(5, b1Powers);
	b2Power = getAverage(5, b2Powers);
	b3Power = getAverage(5, b3Powers);

	currentIndex = (currentIndex >= 4) ? 0 : currentIndex+1;
}

int getAverage(int nbOfEntries, int entries[]) {
	int nonNullEntries = 0;
	int sum = 0;
	for (int i=0; i<nbOfEntries; i++) {
		if(entries[i] != 0)
			nonNullEntries++;
		sum += entries[i];
	}
	return sum/nonNullEntries;
}

uint8_t isOutlierInArray(int nbOfEntries, int array[], int value) {
	int min = array[0], max = array[0];
	for (int i=1; i<nbOfEntries; i++) {
		if (array[i] > max) max = array[i];
		if (array[i] < min) min = array[i];
	}
	return (value > 1.1*max || value < 0.9*min);
}

Point get_position(float rss1, float rss2, float rss3) {

  int8_t P = -69; // Abstract Value, Must be measured

  int8_t N = 2;  // NI

  // getting the distance in meters
  float d1 = pow(10,((P - rss1)/(10*N)));
  float d2 = pow(10,((P - rss2)/(10*N)));
  float d3 = pow(10,((P - rss3)/(10*N)));

  // Define the 3 known points.
  const Point B1 = {-19.866733, -43.9364666};
  const Point B2 = {-19.866425, -43.964556 };
  const Point B3 = {-19.866572, -43.964556 };

  // Calculate the position of the unknown point.

  float A = (-2*B1.x+2*B2.x);
  float B = (-2*B1.y+2*B2.y);
  float C = pow(d1,2)-pow(d2,2)-pow(B1.x,2)+pow(B2.x,2)-pow(B1.y,2)+pow(B2.y,2);
  float D = (-2*B2.x+2*B3.x);
  float E = (-2*B2.y+2*B3.y);
  float F = pow(d2,2)-pow(d3,2)-pow(B2.x,2)+pow(B3.x,2)-pow(B2.y,2)+pow(B2.y,2);

  Point p = {
    .x = ((C*E) - (F*B)) / ((E*A) - (B*D)),
    .y = ((C*D) - (F*A)) / ((B*D) - (A*E))
  };

  return p;
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
