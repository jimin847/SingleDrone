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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h> // For bool type
#include "uart_commands.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "drone.h"
#include "imu.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_LOOP_PERIOD_MS 2 // 2ms, 500Hz control loop
#define LOG_PRINT_INTERVAL 850   // 예: 2ms * 850 = 1.7초마다 로그 출력 (조정 가능)
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
volatile bool control_loop_flag = false;
static uint32_t log_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void PWM_SetDuty(float duty); // 이 함수를 실제로 사용하지 않는다면 경고가 발생할 수 있습니다.
// Error_Handler()는 main.h 또는 HAL 드라이버에 의해 이미 선언되어 있을 것입니다.

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
  MX_USART2_UART_Init();
  UART_Commands_Init(&huart2);

  /* USER CODE BEGIN 2 */
  printf("Control Loop Period: %d ms\r\n", CONTROL_LOOP_PERIOD_MS);

  Drone_Init();
  IMU_Init();
  Filter_Init();

  float current_dt_for_init = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;
  float pid_output_min = -100.0f;
  float pid_output_max = 100.0f;
  float pid_integral_min = -500.0f; // 예: pid.h의 MAX_INTEGRAL 값 활용
  float pid_integral_max = 500.0f;  // 예: pid.h의 MIN_INTEGRAL 값 활용 (부호 확인)

  PID_Init(&pid_roll,  1.0f, 0.1f, 0.05f, current_dt_for_init, pid_output_min, pid_output_max, pid_integral_min, pid_integral_max);
  PID_Init(&pid_pitch, 1.0f, 0.1f, 0.05f, current_dt_for_init, pid_output_min, pid_output_max, pid_integral_min, pid_integral_max);
  PID_Init(&pid_yaw,   1.0f, 0.1f, 0.05f, current_dt_for_init, pid_output_min, pid_output_max, pid_integral_min, pid_integral_max);

  printf("=== Drone Simulation Start ===\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (control_loop_flag)
    {
      control_loop_flag = false;
      float current_dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;

      // 1. 현재 드론 상태 읽기 (센서 값)
            // 이 시점의 드론 상태는 이전 제어 루프에서 actual_..._rate_input에 의해 업데이트된 결과입니다.
            float current_raw_roll  = IMU_GetRawAngle(AXIS_ROLL);
            float current_raw_pitch = IMU_GetRawAngle(AXIS_PITCH);
            float current_raw_yaw   = IMU_GetRawAngle(AXIS_YAW);

            float current_gyro_roll  = IMU_GetRawGyro(AXIS_ROLL);
            float current_gyro_pitch = IMU_GetRawGyro(AXIS_PITCH);
            float current_gyro_yaw   = IMU_GetRawGyro(AXIS_YAW);

            // 2. 센서 값 필터링
            Filter_Update(AXIS_ROLL,  current_raw_roll,  current_gyro_roll,  current_dt);
            Filter_Update(AXIS_PITCH, current_raw_pitch, current_gyro_pitch, current_dt);
            Filter_Update(AXIS_YAW,   current_raw_yaw,   current_gyro_yaw,   current_dt);

            // 3. 필터링된 현재 각도 값 (PID 입력으로 사용)
            float filt_roll_for_pid  = Filter_GetAngle(AXIS_ROLL);
            float filt_pitch_for_pid = Filter_GetAngle(AXIS_PITCH);
            float filt_yaw_for_pid   = Filter_GetAngle(AXIS_YAW);

            // 4. PID 제어 연산
            float out_roll  = PID_Compute(&pid_roll,  target_angles[AXIS_ROLL],  filt_roll_for_pid);
            float out_pitch = PID_Compute(&pid_pitch, target_angles[AXIS_PITCH], filt_pitch_for_pid);
            float out_yaw   = PID_Compute(&pid_yaw,   target_angles[AXIS_YAW],   filt_yaw_for_pid);

            // 5. PID 출력을 드론의 다음 상태 업데이트를 위한 입력으로 사용 (제어 루프 피드백)
            float actual_roll_rate_input = out_roll;
            float actual_pitch_rate_input = out_pitch;
            float actual_yaw_rate_input = out_yaw;

            Drone_Update(actual_roll_rate_input, actual_pitch_rate_input, actual_yaw_rate_input, current_dt);
            // 이제 Drone_GetState()를 호출하면 이 제어 입력이 반영된 최신 상태를 얻을 수 있습니다.

            // 6. 상태 UART 출력 (로깅)
                  log_counter++;
                  if (log_counter >= LOG_PRINT_INTERVAL) {
                      log_counter = 0;
                      if (!UART_IsUserTypingCommand()) {
                          DroneState* current_drone_state = Drone_GetState(); // 최신 상태 가져오기

                          printf("--- Time: %.3fs ---\r\n", (float)HAL_GetTick()/1000.0f);
                          printf("Target:\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 target_angles[AXIS_ROLL], target_angles[AXIS_PITCH], target_angles[AXIS_YAW]);
                          // PID 입력으로 사용된 센서 값들
                          printf("Raw(PID_In):\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 current_raw_roll, current_raw_pitch, current_raw_yaw);
                          printf("Filt(PID_In):\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 filt_roll_for_pid, filt_pitch_for_pid, filt_yaw_for_pid);
                          printf("PID Out:\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 out_roll, out_pitch, out_yaw);
                          // 제어 출력이 반영된 후의 드론 상태 (각도 및 각속도)
                          printf("State(Angle):\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 current_drone_state->angle[AXIS_ROLL], current_drone_state->angle[AXIS_PITCH], current_drone_state->angle[AXIS_YAW]);
                          printf("State(AngVel):\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 current_drone_state->angVel[AXIS_ROLL], current_drone_state->angVel[AXIS_PITCH], current_drone_state->angVel[AXIS_YAW]);
                          printf("-----------------------\r\n\r\n");
                          fflush(stdout);
          }
      }
    }
    UART_Commands_Process(); // UART 명령어 처리
    /* USER CODE END 3 */
  } // 여기가 main 함수의 실제 끝입니다.
} // 여기가 main 함수의 실제 끝입니다. 이 중괄호 뒤에 다른 함수 정의가 와야 합니다.

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
static void MX_USART2_UART_Init(void){
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
    // 이 함수는 현재 사용되지 않고 있으며, 실제 PWM 로직으로 대체되어야 합니다.
    // 시뮬레이션 용도 또는 실제 PWM 타이머를 사용하여 구현합니다.
    // 예: __HAL_TIM_SET_COMPARE(&htimX, TIM_CHANNEL_Y, (uint32_t)duty_cycle_value);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("Error_Handler() called!\r\n");
  fflush(stdout); // 버퍼 비우기
  __disable_irq();
  while (1)
  {
     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // LD2 핀 (보통 PA5) 토글
     HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
  fflush(stdout); // 버퍼 비우기
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
