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
#include "pid.h"
#include "drone.h"
#include "imu.h"
#include "filter.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* For semihosting on newlib */
extern void  initialise_monitor_handles(void);

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PIDController pid_roll;
PIDController pid_pitch;
PIDController pid_yaw;
float target_angles[3] = {0.0f, 0.0f, 0.0f};
char msg[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
float PID_Compute(PIDController *pid, float setpoint, float measurement);
void PWM_SetDuty(float duty);
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
  initialise_monitor_handles(); // Semihosting 활성화
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  printf("Hello World!\r\n");   // 출력 테스트

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 드론·센서·필터 초기화
   Drone_Init();
   IMU_Init();               // 드론 초기화 포함
   Filter_Init();      // 각 축별 칼만 필터 초기화

   // Roll/Pitch/Yaw 축별 PID 초기화
   PID_Init(&pid_roll,  1.0f, 0.1f, 0.05f);
   PID_Init(&pid_pitch, 1.0f, 0.1f, 0.05f);
   PID_Init(&pid_yaw,   1.0f, 0.1f, 0.05f);

   printf("=== Drone Simulation Start ===\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(100); // 100ms 주기

	 	  // (1) 드론 상태 갱신 (간단히 일정한 제어 입력 시뮬레이션)
	 	  Drone_Update(0.5f, 0.2f, -0.1f);  // 각속도 입력 (rad/s 단위)

	 	  // (2) 센서값 획득 (노이즈 포함된 IMU 측정값)
	 	 float raw_roll  = IMU_GetRawAngle(AXIS_ROLL);
	 	 float raw_pitch = IMU_GetRawAngle(AXIS_PITCH);
	 	 float raw_yaw   = IMU_GetRawAngle(AXIS_YAW);

	 	 float gyro_roll  = IMU_GetRawGyro(AXIS_ROLL);
	 	 float gyro_pitch = IMU_GetRawGyro(AXIS_PITCH);
	 	 float gyro_yaw   = IMU_GetRawGyro(AXIS_YAW);

	 	  // (3) 칼만 필터 업데이트 (void 반환)
	 	  Filter_Update(AXIS_ROLL,  raw_roll,  gyro_roll);
	 	  Filter_Update(AXIS_PITCH, raw_pitch, gyro_pitch);
	 	  Filter_Update(AXIS_YAW,   raw_yaw,   gyro_yaw);

	 	  // 4) 필터링된 각도 가져오기
	 	  float filt_roll  = Filter_GetAngle(AXIS_ROLL);
	 	  float filt_pitch = Filter_GetAngle(AXIS_PITCH);
	 	  float filt_yaw   = Filter_GetAngle(AXIS_YAW);

	 	  // (5) PID 연산
	 	  float out_roll  = PID_Compute(&pid_roll,  target_angles[AXIS_ROLL],  filt_roll);
	 	  float out_pitch = PID_Compute(&pid_pitch, target_angles[AXIS_PITCH], filt_pitch);
	 	  float out_yaw   = PID_Compute(&pid_yaw,   target_angles[AXIS_YAW],   filt_yaw);

	 	  // (6) PWM 출력 시뮬레이션
	 	  PWM_SetDuty(out_roll);
	 	  PWM_SetDuty(out_pitch);
	 	  PWM_SetDuty(out_yaw);

	 	  // (7) 상태 UART 출력
	 	 snprintf(msg, sizeof(msg),
	 	              "R: raw=%.2f, filt=%.2f, out=%.2f | "
	 	              "P: raw=%.2f, filt=%.2f, out=%.2f | "
	 	              "Y: raw=%.2f, filt=%.2f, out=%.2f\r\n",
	 	              raw_roll, filt_roll, out_roll,
	 	              raw_pitch, filt_pitch, out_pitch,
	 	              raw_yaw, filt_yaw, out_yaw);
	 	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PWM_SetDuty(float duty)
{
    // duty 범위 제한: 0 ~ 100%
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;

    // 실제 하드웨어 PWM 제어가 있다면 여기에 HAL_TIM_PWM_SetCompare 넣음
    // 시뮬레이션 용도로 UART로 출력만
    char pwm_msg[64];
    snprintf(pwm_msg, sizeof(pwm_msg), "[PWM] Duty = %.1f %%\r\n", duty);
    HAL_UART_Transmit(&huart2, (uint8_t*)pwm_msg, strlen(pwm_msg), HAL_MAX_DELAY);
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
