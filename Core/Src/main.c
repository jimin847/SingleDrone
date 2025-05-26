/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics. // 2024로 가정하고 진행합니다.
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
// #include "pid.h"      // 이미 main.h 또는 다른 곳에서 include 될 수 있음
// #include "drone.h"
// #include "imu.h"
// #include "filter.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h> // For bool type
#include "uart_commands.h" // 새로운 헤더 파일 추가


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 사용자 정의 헤더 파일 (필요 시)
#include "pid.h"
#include "drone.h"
#include "imu.h"
#include "filter.h"
/* USER CODE END Includes */

/* For semihosting on newlib (디버깅 시에만 사용 권장) */
// extern void  initialise_monitor_handles(void); // UART printf 사용 시 주석 처리 가능

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_LOOP_PERIOD_MS 2 // 2ms, 500Hz control loop (기존 100ms HAL_Delay 대신 사용)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // USART2 사용

/* USER CODE BEGIN PV */
PIDController pid_roll;
PIDController pid_pitch;
PIDController pid_yaw;
float target_angles[3] = {0.0f, 0.0f, 0.0f}; // uart_commands.c에서 extern으로 참조
char msg[256]; // UART 메시지 버퍼 크기 약간 늘림
volatile bool control_loop_flag = false;

static uint32_t log_counter = 0; // 로그 출력 빈도 조절용 카운터 추가
#define LOG_PRINT_INTERVAL 550   // 예: 50번 루프마다 로그 출력 (2ms * 50 = 100ms 간격)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
// static void MX_I2C1_Init(void); // I2C 초기화 함수 (CubeMX에서 생성된 경우)
// static void MX_TIM2_Init(void); // 타이머 초기화 함수 (CubeMX에서 생성된 경우)
// static void MX_TIM3_Init(void);
// static void MX_TIM4_Init(void);


/* USER CODE BEGIN PFP */
// float PID_Compute(PIDController *pid, float setpoint, float measurement); // pid.h 에 선언되어 있을 것으로 가정
// main.c 상단 USER CODE BEGIN PFP 섹션 또는 main.h 에 추가
void PWM_SetDuty(float duty);

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
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
  // initialise_monitor_handles(); // Semihosting 비활성화 (UART printf 사용 시)
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  // uart_commands 모듈 초기화 시 huart2 핸들 전달
    UART_Commands_Init(&huart2); // printf("System Initialized!...") 등은 이 함수 내부로 옮겨도 됨
  // MX_I2C1_Init();   // IMU 사용 시 필요
  // MX_TIM2_Init();   // PWM 사용 시 필요
  // MX_TIM3_Init();
  // MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  printf("Control Loop Period: %d ms\r\n", CONTROL_LOOP_PERIOD_MS);


  // 드론·센서·필터 초기화
   Drone_Init(); // Drone_t 구조체 및 관련 변수 초기화
   IMU_Init();   // IMU 센서(또는 시뮬레이션) 초기화
   Filter_Init(); // 각 축별 필터 초기화 (어떤 필터인지 filter.c/h 확인 필요)

   // PID 초기화 (수정된 PID_Init 사용)
   float current_dt_for_init = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;

   // 예시 출력 제한값 및 적분 제한값 (튜닝 필요)
   float pid_output_min = -100.0f;
   float pid_output_max = 100.0f;
   float pid_integral_min = -500.0f; // 예: MAX_INTEGRAL 매크로 값 활용
   float pid_integral_max = 500.0f;  // 예: MIN_INTEGRAL 매크로 값 활용

   // Roll/Pitch/Yaw 축별 PID 초기화
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
      control_loop_flag = false; // 플래그 먼저 리셋
      float current_dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f; // 초 단위 dt

      // (1) 드론 상태 갱신 (시뮬레이션용 가상 움직임) - dt 전달
      static float virtual_roll_rate_input = 0.0f;
      static float virtual_pitch_rate_input = 0.0f;
      static float virtual_yaw_rate_input = 0.0f;
      // Drone_Update의 입력이 '각속도'임을 명시 (함수 내부에서 dt를 곱해 각도 변화 계산)
      Drone_Update(virtual_roll_rate_input, virtual_pitch_rate_input, virtual_yaw_rate_input, current_dt);

      // (2) 센서값 획득 (시뮬레이션된 값)
      float raw_roll  = IMU_GetRawAngle(AXIS_ROLL);
      float raw_pitch = IMU_GetRawAngle(AXIS_PITCH);
      float raw_yaw   = IMU_GetRawAngle(AXIS_YAW);

      float gyro_roll  = IMU_GetRawGyro(AXIS_ROLL);
      float gyro_pitch = IMU_GetRawGyro(AXIS_PITCH);
      float gyro_yaw   = IMU_GetRawGyro(AXIS_YAW);

      // (3) 필터 업데이트 (칼만 필터) - dt 전달
      Filter_Update(AXIS_ROLL,  raw_roll,  gyro_roll,  current_dt);
      Filter_Update(AXIS_PITCH, raw_pitch, gyro_pitch, current_dt);
      Filter_Update(AXIS_YAW,   raw_yaw,   gyro_yaw,   current_dt);

      // 4) 필터링된 각도 가져오기
      float filt_roll  = Filter_GetAngle(AXIS_ROLL);
      float filt_pitch = Filter_GetAngle(AXIS_PITCH);
      float filt_yaw   = Filter_GetAngle(AXIS_YAW);

      // (5) PID 연산 - PID_Compute는 내부적으로 구조체의 dt 사용
      // target_angles는 UART나 다른 방식으로 설정되도록 수정 필요
      float out_roll  = PID_Compute(&pid_roll,  target_angles[AXIS_ROLL],  filt_roll);
      float out_pitch = PID_Compute(&pid_pitch, target_angles[AXIS_PITCH], filt_pitch);
      float out_yaw   = PID_Compute(&pid_yaw,   target_angles[AXIS_YAW],   filt_yaw);

//      // (6) PWM 출력 (시뮬레이션, 믹싱 로직 부재)
//      PWM_SetDuty(out_roll);

      // (7) 상태 UART 출력 (이전과 동일)
      log_counter++;
      if (log_counter >= LOG_PRINT_INTERVAL) {
                log_counter = 0;
                // 여러 줄로 로그 메시지 구성
                          printf("--- Time: %.3fs ---\r\n", (float)HAL_GetTick()/1000.0f);

                          printf("Raw   :\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 raw_roll, raw_pitch, raw_yaw);

                          printf("Filtered:\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 filt_roll, filt_pitch, filt_yaw);

                          printf("PID Out:\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                 out_roll, out_pitch, out_yaw);

                          // 믹싱 로직이 구현되면 여기에 모터 및 서보 명령 값도 출력
                          // 예: printf("Motors  :\tM0=%.2f,\tM1=%.2f\r\n", motor_cmd[0], motor_cmd[1]);
                          // 예: printf("Servos  :\tS0=%.1f,\tS1=%.1f,\tS2=%.1f,\tS3=%.1f\r\n", servo_cmd[0], ...);
                          printf("-----------------------\r\n\r\n"); // 로그 그룹 구분

                          // HAL_UART_Transmit을 직접 사용해야 한다면, snprintf로 전체 메시지를 만들어서 한 번에 전송
                          /*
                          char full_log_msg[512]; // 충분한 크기의 버퍼
                          int offset = 0;
                          offset += snprintf(full_log_msg + offset, sizeof(full_log_msg) - offset,
                                             "--- Time: %.3fs ---\r\n", (float)HAL_GetTick()/1000.0f);
                          offset += snprintf(full_log_msg + offset, sizeof(full_log_msg) - offset,
                                             "Raw   :\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                             raw_roll, raw_pitch, raw_yaw);
                          offset += snprintf(full_log_msg + offset, sizeof(full_log_msg) - offset,
                                             "Filtered:\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                             filt_roll, filt_pitch, filt_yaw);
                          offset += snprintf(full_log_msg + offset, sizeof(full_log_msg) - offset,
                                             "PID Out:\tRoll=%.2f,\tPitch=%.2f,\tYaw=%.2f\r\n",
                                             out_roll, out_pitch, out_yaw);
                          offset += snprintf(full_log_msg + offset, sizeof(full_log_msg) - offset,
                                             "-----------------------\r\n\r\n");
                          HAL_UART_Transmit(&huart2, (uint8_t*)full_log_msg, strlen(full_log_msg), 100); // Timeout 조절 필요
                          */
            }
          }
    // 비주기적 작업 (예: UART 명령어 수신)
    UART_Commands_Process();
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

/**
  * @brief  Retargets the C library printf function to the USART (USART2 in this case).
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// PWM_SetDuty 함수는 현재 UART로 값을 출력하는 시뮬레이션용으로 유지합니다.
// 실제 PWM 타이머 설정은 MX_TIMx_Init() 함수들이 CubeMX에 의해 생성되고,
// 해당 타이머 핸들(예: htim2)을 사용하여 __HAL_TIM_SET_COMPARE() 등으로 값을 설정해야 합니다.
void PWM_SetDuty(float duty)
{
    // duty 범위 제한: (예시, 실제 PID 출력 범위에 따라 달라짐)
    // if (duty < -100.0f) duty = -100.0f;
    // if (duty > 100.0f) duty = 100.0f;

//    // 시뮬레이션 용도로 UART로 출력만
//    char pwm_msg[64];
//    snprintf(pwm_msg, sizeof(pwm_msg), "[PWM Out] Val = %.2f\r\n", duty); // PID 출력값을 그대로 보여주도록 수정
//    HAL_UART_Transmit(&huart2, (uint8_t*)pwm_msg, strlen(pwm_msg), 10); // Timeout 줄임
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
  printf("Error_Handler() called!\r\n");
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
