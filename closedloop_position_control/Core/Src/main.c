/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include "usbd_cdc_if.h"
#include "string.h"
#include "as5048a.h"
//#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
//static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char data[50];
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=12;
float voltage_power_supply=12;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
int period=1600;
int pole_pairs=11;
int index1=0;
uint16_t raw1,raw2,raw3;
float motor_target= M_PI/6;
float Ts=5E-3f;
float Kp=0.167f;
int dir=1;
//static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
//{
//  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
//}
//uint32_t getCurrentMicros(void)
//{
//  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
//  LL_SYSTICK_IsActiveCounterFlag();
//  uint32_t m = HAL_GetTick();
//  const uint32_t tms = SysTick->LOAD + 1;
//  __IO uint32_t u = tms - SysTick->VAL;
//  if (LL_SYSTICK_IsActiveCounterFlag()) {
//    m = HAL_GetTick();
//    u = tms - SysTick->VAL;
//  }
//  return (m * 1000 + (u * 1000) / tms);
//}

float _normalizeAngle(float angle){
  float a = fmod(angle, 2*M_PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*M_PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。
  //可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2M_PI 的符号相反。
  //也就是说，如果 angle 的值小于 0 且 _2M_PI 的值为正数，则 fmod(angle, _2M_PI) 的余数将为负数。
  //例如，当 angle 的值为 -M_PI/2，_2M_PI 的值为 2M_PI 时，fmod(angle, _2M_PI) 将返回一个负数。
  //在这种情况下，可以通过将负数的余数加上 _2M_PI 来将角度归一化到 [0, 2M_PI] 的范围内，以确保角度的值始终为正数。
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
  return _normalizeAngle(((float)(dir * pole_pairs)*shaft_angle)-zero_electric_angle);
}

void setPwm(float Ua, float Ub, float Uc) {

//	// 限制上限
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
	// 计算占空比
	// 限制占空比从0到1
	dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
	dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
	dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

	//写入PWM到PWM 0 1 2 通道
	TIM1->CCR1 = (uint32_t) roundf(dc_a*period);
	TIM1->CCR2 = (uint32_t) roundf(dc_b*period);
	TIM1->CCR3 = (uint32_t) roundf(dc_c*period);

}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el);
  Ubeta =   Uq*cos(angle_el);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

void setPhaseVoltage_test(float Ua,float Ub, float Uc) {


  // 克拉克逆变换
  Ua +=  voltage_power_supply/2;
  Ub +=  voltage_power_supply/2;
  Uc +=  voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}


//开环速度函数
float velocityOpenloop(float target_velocity){
//	uint32_t now_us = getCurrentMicros();
//	uint32_t now_us = HAL_GetTick();
//  Provides a tick value in microseconds.

  //计算当前每个Loop的运行时间间隔
//  float Ts = (now_us - open_loop_timestamp) * 1e-3f;
	float Ts=5E-3f;

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。
  //在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。
  //因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // Uq is not related to voltage limit
  float Uq = 5.5;

  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

//  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return Uq;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

   setPhaseVoltage(3,0,_electricalAngle(M_PI*1.5f,pole_pairs));
   HAL_Delay(3000);
   uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
   zero_electric_angle=_electricalAngle(M_PI*read_raw/MAX_ANGLE_VALUE,pole_pairs);
   setPhaseVoltage(0,0,_electricalAngle(M_PI*1.5f,pole_pairs));
   sprintf(data, "zero_electric_angle: %i \n", (int) floor(zero_electric_angle/M_PI*180));
   CDC_Transmit_FS((uint8_t*) data, strlen(data));
   HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2400-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CSn_Pin */
  GPIO_InitStruct.Pin = SPI1_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CSn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_Enable_Pin */
  GPIO_InitStruct.Pin = Motor_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_Enable_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2 )
  {



    index1+=1;
    uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
    float angle_now=(float)read_raw /(float)MAX_ANGLE_VALUE *2*M_PI*dir;
    float angle_error=motor_target-angle_now;

    angle_error=_normalizeAngle(angle_error);
    if (angle_error > M_PI){
    	angle_error-=2*M_PI;
    }
    setPhaseVoltage(_constrain(Kp*(angle_error)/M_PI*180,-voltage_power_supply/2,voltage_power_supply/2),0,_electricalAngle(angle_now,pole_pairs));
//    velocityOpenloop(5);
//    sprintf(data, "open loop control \n");
//    sprintf(data, "angle: %u \n", read_raw);
//    CDC_Transmit_FS((uint8_t*) data, strlen(data));
//    sprintf(data, "angle_now : %i \n", (int) floor(angle_now*180/M_PI));
//        CDC_Transmit_FS((uint8_t*) data, strlen(data));
    sprintf(data, "angle_error : %i \n", (int) floor(angle_error/M_PI*180));
    CDC_Transmit_FS((uint8_t*) data, strlen(data));

    if (index1 == 200){
    	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    	index1=0;
    }
  }
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
