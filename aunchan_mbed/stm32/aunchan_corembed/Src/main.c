
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "mainpp.h"
#include <math.h>
/*------Initial Machanism------*/
#define WHEEL_RADIUS 0.031
#define WHEEL_SEPARATION 0.19

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*------ Motor and Encoder ------*/
typedef struct Encoder
{
	//uint8_t signalA;
	//uint8_t signalB;
	//uint8_t currentState;
	//uint8_t previousState;
	int steps;
	int pre_steps;
}Encoder;
typedef struct Motor
{
	int PWM;
	float velocity;
	float Kp;
	float Ki;
	float Kd;
	float error;
	float error_prior;
	float integral;
	float derivative;
	Encoder encoder;
	float angle,preAngle;
}Motor;
Motor M1 = {
		.Kp = 0.1,
		.Ki = 3,
		.Kd = 0.5,
		.error_prior = 0,
		.integral = 0,
		.encoder.pre_steps = 0,
		.preAngle = 0
};
Motor M2 = {
		.Kp = 0.1,
		.Ki = 3,
		.Kd = 0.5,
		.error_prior = 0,
		.integral = 0,
		.encoder.pre_steps = 0,
		.preAngle = 0
};
//float samplingRate = 0.001;
float speedWheel[2] = {0};
Twist msg;
Twist preMsg;


/*------ Imu ------*/
Imu mpu6050;
/*------ RTOS ------*/
int nowTick = 0;
int lastTickROS = 0;
int lastTickIMU = 0;
FlagStatus debugFlag = RESET;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void setPWM(TIM_HandleTypeDef * _timer, uint32_t _channel, uint16_t _pulse);
void control (int desired_vel, Motor* M);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t ADC_BUFF[2];
uint32_t rawBattery;
uint32_t rawTemp;
float battery;
float temp;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		rawBattery = ADC_BUFF[0];
		rawTemp = ADC_BUFF[1];
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_BUFF,2);
  HAL_ADC_Start_IT(&hadc1);

  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_Delay(10);

  setPWM(&htim8,TIM_CHANNEL_3,500);
  setPWM(&htim8,TIM_CHANNEL_1,500);

  mpu6050_init();
  setup();
  float yaw;
  //HAL_Delay(10000);
  uint32_t magnitude;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  nowTick = HAL_GetTick();
	  if ((nowTick - lastTickROS) >= 50) //20Hz
	  {
		  loop();
		  battery = (((float)rawBattery * 3.3 * 11.67) / 4095.0);
		  temp = ((((float)rawTemp / 4095 * 3600) - 1430) / 4.3) + 25;
		  //ros2pid(pid);
		  //M1.Kp = M2.Kp = pid[0];
		  //M1.Kp = M2.Kp = pid[1];
		  //M1.Kp = M2.Kp = pid[2];
		  //double siny = +2.0 * (mpu6050.quat[0] * mpu6050.quat[3] + mpu6050.quat[1] * mpu6050.quat[2]);
		  //double cosy = +1.0 - 2.0 * (mpu6050.quat[2] * mpu6050.quat[2] + mpu6050.quat[3] * mpu6050.quat[3]);
		  //yaw = atan2(siny, cosy);
		  //yaw   *= 180.0f / PI;
		  //char debug[100];
		  //sprintf(debug,"Yaw = %f\r\n",yaw);
		  //DebugROS(debug);
		  lastTickROS = nowTick;
	  }

	  if ((nowTick - lastTickIMU) >= 10) //100Hz
	  {
		  mpu6050_read(mpu6050.gyro, mpu6050.accel, mpu6050.quat);
	      mpu6050.gyro[0] = (mpu6050.gyro[0] * PI)/180;
	      mpu6050.gyro[1] = (mpu6050.gyro[1] * PI)/180;
	      mpu6050.gyro[2] = (mpu6050.gyro[2] * PI)/180;
	      mpu6050.accel[0] = mpu6050.accel[0] * 9.80665;
	      mpu6050.accel[1] = mpu6050.accel[1] * 9.80665;
	      mpu6050.accel[2] = mpu6050.accel[2] * 9.80665;
	      magnitude = sqrt(mpu6050.quat[0]*mpu6050.quat[0] + mpu6050.quat[1]*mpu6050.quat[1] + mpu6050.quat[2]*mpu6050.quat[2] + mpu6050.quat[3]*mpu6050.quat[3]);
	      //if(magnitude > 0 && magnitude <= 65536)
	      //{
			  mpu6050.quat[0] /= (float)magnitude;
			  mpu6050.quat[1] /= (float)magnitude;
			  mpu6050.quat[2] /= (float)magnitude;
			  mpu6050.quat[3] /= (float)magnitude;
			  imu2ros(mpu6050);
	      //}
		  //char debug[100];
		  //sprintf(debug,"%lu\r\n",magnitude);
		  //DebugROS(debug);
		  lastTickIMU = nowTick;

	  }

	  //float siny = +2.0 * (mpu6050.quat[0] * mpu6050.quat[3] + mpu6050.quat[1] * mpu6050.quat[2]);
	  //float cosy = +1.0 - 2.0 * (mpu6050.quat[2] * mpu6050.quat[2] + mpu6050.quat[3] * mpu6050.quat[3]);
	  //yaw = atan2(siny, cosy);
	  //float delta_x = ();

	  if(debugFlag == SET)
	  {
		  debugFlag = RESET;

		  ros2velocity(&msg);
		  /*if(msg.linear[0] * preMsg.linear[0] < 0 || msg.angular[2] * preMsg.angular[2] < 0)
		  {
			  M1.error = M2.error = 0;
			  M1.error_prior = M2.error_prior = 0;
			  M1.integral = M2.integral = 0;
			  M1.derivative = M2.derivative = 0;
		  }
		  preMsg = msg;*/


		  speedWheel[0] = -msg.linear[0] + (msg.angular[2] * WHEEL_SEPARATION / 2);
		  speedWheel[1] = -msg.linear[0] - (msg.angular[2] * WHEEL_SEPARATION / 2);
		  speedWheel[0] = speedWheel[0] / WHEEL_RADIUS;
		  speedWheel[1] = speedWheel[1] / WHEEL_RADIUS;

		  M1.angle = M1.encoder.steps * ((2 * 3.1415) / 780);
		  M1.velocity = (M1.angle - M1.preAngle) / 0.01;
		  M1.preAngle = M1.angle;
		  M2.angle = M2.encoder.steps * ((2 * 3.1415) / 780);
		  M2.velocity = (M2.angle - M2.preAngle) / 0.01;
		  M2.preAngle = M2.angle;
		  control(speedWheel[0],&M1);
		  control(speedWheel[1],&M2);

		  //char Debug[100];
		  //sprintf(Debug,"Desired = %f, Current =  %f ",speedWheel[0],M1.velocity);
		  //sprintf(Debug,"Desired = %f, Current =  %f\r\n",speedWheel[1],M2.velocity);
		  //DebugROS(Debug);

		  if(msg.linear[0] == 0 && msg.angular[2] == 0)
		  {
			  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			  HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_3);
			  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			  HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
			  M1.error = M2.error = 0;
			  M1.error_prior = M2.error_prior = 0;
			  M1.integral = M2.integral = 0;
			  M1.derivative = M2.derivative = 0;
		  }

		  else
		  {
			  setPWM(&htim8,TIM_CHANNEL_3,M1.PWM);
			  setPWM(&htim8,TIM_CHANNEL_1,M2.PWM);
		  }
	  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 720-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 3-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 72;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12){
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == 1)
			HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == 1 ? M1.encoder.steps++ : M1.encoder.steps--;
		else
			HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == 1 ? M1.encoder.steps-- : M1.encoder.steps++;
	}
	if(GPIO_Pin == GPIO_PIN_5){
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) == 1)
			HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == 1 ?  M2.encoder.steps-- : M2.encoder.steps++;
		else
			HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == 1 ?  M2.encoder.steps++ : M2.encoder.steps--;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6){
		 debugFlag = SET;
	}
}
void setPWM(TIM_HandleTypeDef * _timer, uint32_t _channel, uint16_t _pulse)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = _pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(_timer, &sConfigOC, _channel);

	HAL_TIM_PWM_Start(_timer, _channel);
	HAL_TIMEx_PWMN_Start(_timer,_channel);
}
void control (int desired_vel, Motor* M){
	M->error = desired_vel - M->velocity;
	//M->integral = M->integral + (M->error * samplingRate);
	M->integral = M->integral + M->error;
	//M->derivative = (M->error - M->error_prior) / samplingRate;
	M->derivative = M->error - M->error_prior;
	M->error_prior = M->error;
	//if (M->integral > 10) M->integral = 10;
		//if (M->integral < -10) M->integral = -10;
	M->PWM = M->Kp*M->error + M->Ki*M->integral + M->Kd*M->derivative;
	if (M->PWM >= 0)
		M->PWM = M->PWM <= 500 ? M->PWM + 500:1000;
	else{
		M->PWM = M->PWM * -1;
		M->PWM = M->PWM <= 500 ? 500 - M->PWM:0;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
