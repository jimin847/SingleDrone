#include "uart_commands.h"
#include "main.h"     // Error_Handler, AXIS_ROLL 등 main.h 정의 사용
#include "pid.h"      // PID_Init, PIDController 등 PID 리셋 시 필요
#include "imu.h"      // AXIS_ROLL, AXIS_PITCH, AXIS_YAW 정의를 위해 추가!
#include <stdio.h>    // sscanf, printf 사용
#include <string.h>   // strncmp, strlen, strcspn 사용 (개행 문자 제거용)
#include <stdlib.h>   // atof (문자열 to float 변환)

// --- Private Variables ---
static UART_HandleTypeDef *g_huart_cmd; // 사용할 UART 핸들러 포인터
static uint8_t g_rx_buffer_char[1];              // 1 바이트씩 수신하기 위한 버퍼
static uint8_t g_command_buffer[UART_CMD_RX_BUFFER_SIZE]; // 명령어 누적 버퍼
static uint16_t g_command_buffer_idx = 0;
static volatile bool g_command_ready_flag = false; // 완성된 명령어 수신 플래그

// main.c 에 정의된 전역 변수들을 사용하기 위한 extern 선언
extern float target_angles[3];
extern PIDController pid_roll;
extern PIDController pid_pitch;
extern PIDController pid_yaw;
// extern volatile bool control_loop_flag; // CONTROL_LOOP_PERIOD_MS를 직접 사용하거나 main.h에 정의
#ifndef CONTROL_LOOP_PERIOD_MS // main.c에 정의된 값을 가져오지 못할 경우 대비
#define CONTROL_LOOP_PERIOD_MS 2 // 기본값 설정
#endif

// --- Function Definitions ---

void UART_Commands_Init(UART_HandleTypeDef *huart_handle_to_use) {
    g_huart_cmd = huart_handle_to_use;
    g_command_buffer_idx = 0;
    g_command_ready_flag = false;
    memset(g_command_buffer, 0, UART_CMD_RX_BUFFER_SIZE);

    printf("\r\nUART Command Interface Initialized.\r\n");
    printf("Available commands (terminate with Enter):\r\n");
    printf("  r<value>  : Set target Roll angle (e.g., r10.5)\r\n");
    printf("  p<value>  : Set target Pitch angle (e.g., p-5.0)\r\n");
    printf("  y<value>  : Set target Yaw angle (e.g., y0.0)\r\n");
    printf("  reset     : Reset target angles and PIDs\r\n");
    printf(">"); // 프롬프트

    // 첫 번째 1바이트 수신 인터럽트 시작
    if (HAL_UART_Receive_IT(g_huart_cmd, g_rx_buffer_char, 1) != HAL_OK) {
        printf("Error: Failed to start UART RX IT in UART_Commands_Init\r\n");
        // Error_Handler(); // 필요시 에러 처리
    }
}

// 이 함수는 stm32f1xx_it.c 의 HAL_UART_RxCpltCallback 에서 호출됨
void UART_CMD_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == g_huart_cmd->Instance) {
        // 수신된 바이트를 명령어 버퍼에 추가
        if (g_rx_buffer_char[0] == '\r' || g_rx_buffer_char[0] == '\n') { // 개행 문자(CR 또는 LF)를 만나면
            if (g_command_buffer_idx > 0) { // 버퍼에 내용이 있으면 명령어 완성
                g_command_buffer[g_command_buffer_idx] = '\0'; // Null-terminate string
                g_command_ready_flag = true;             // 명령어 수신 플래그 설정
                // g_command_buffer_idx = 0; // UART_Commands_Process에서 리셋
            } else { // 빈 줄 입력 (엔터만)
                g_command_buffer_idx = 0; // 인덱스 리셋
                printf(">"); // 새 프롬프트
            }
            // 개행 문자를 받았으므로, 다음 입력을 위해 다시 수신 시작 (명령어 처리 후에도 가능)
            // 여기서는 g_command_ready_flag가 true가 되면 UART_Commands_Process에서 처리 후 다시 수신 시작하도록 함
        } else if (g_rx_buffer_char[0] >= 32 && g_rx_buffer_char[0] <= 126) { // Printable ASCII characters
            if (g_command_buffer_idx < UART_CMD_RX_BUFFER_SIZE - 1) {
                g_command_buffer[g_command_buffer_idx++] = g_rx_buffer_char[0];
                // 에코 백 (사용자 입력 확인용)
                HAL_UART_Transmit(g_huart_cmd, g_rx_buffer_char, 1, 10);
            } else { // 버퍼 오버플로우
                g_command_buffer[UART_CMD_RX_BUFFER_SIZE - 1] = '\0';
                printf("\r\nError: Command buffer overflow. Command cleared.\r\n>");
                g_command_buffer_idx = 0; // 버퍼 클리어
            }
        }
        // (g_command_ready_flag가 false일 때만) 다음 바이트 수신 계속
        if (!g_command_ready_flag) {
             if (HAL_UART_Receive_IT(g_huart_cmd, g_rx_buffer_char, 1) != HAL_OK) {
                 printf("\r\nError: Failed to restart UART RX IT (accumulating)\r\n>");
            }
        }
    }
}

void UART_CMD_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == g_huart_cmd->Instance) {
        uint32_t error_code = HAL_UART_GetError(g_huart_cmd);
        printf("\r\nUART RX Error! Code: 0x%lX\r\n>", error_code);
        // 에러 플래그 클리어 및 수신 재시작 시도
        __HAL_UART_CLEAR_OREFLAG(g_huart_cmd);
        __HAL_UART_CLEAR_NEFLAG(g_huart_cmd);
        __HAL_UART_CLEAR_FEFLAG(g_huart_cmd);
        g_command_buffer_idx = 0;
        g_command_ready_flag = false;
        if (HAL_UART_Receive_IT(g_huart_cmd, g_rx_buffer_char, 1) != HAL_OK) {
            printf("Error: Failed to restart UART RX IT after error callback\r\n>");
        }
    }
}

void UART_Commands_Process(void) {
    if (g_command_ready_flag) {
        // 수신된 명령어 처리
        printf("\r\nCMD RX: %s\r\n", (char*)g_command_buffer);

        float val;
        // sscanf는 float 파싱 시 locale 문제로 동작 안 할 수 있음. atof 권장.
        // 또는 간단하게 직접 파싱
        if (g_command_buffer[0] == 'r') {
            val = atof((char*)g_command_buffer + 1);
            target_angles[AXIS_ROLL] = val;
            printf("Target Roll set to: %.2f\r\n", target_angles[AXIS_ROLL]);
        } else if (g_command_buffer[0] == 'p') {
            val = atof((char*)g_command_buffer + 1);
            target_angles[AXIS_PITCH] = val;
            printf("Target Pitch set to: %.2f\r\n", target_angles[AXIS_PITCH]);
        } else if (g_command_buffer[0] == 'y') {
            val = atof((char*)g_command_buffer + 1);
            target_angles[AXIS_YAW] = val;
            printf("Target Yaw set to: %.2f\r\n", target_angles[AXIS_YAW]);
        } else if (strncmp((char*)g_command_buffer, "reset", 5) == 0) {
            target_angles[AXIS_ROLL] = 0.0f;
            target_angles[AXIS_PITCH] = 0.0f;
            target_angles[AXIS_YAW] = 0.0f;

            float pid_reinit_dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;
            PID_Init(&pid_roll, pid_roll.kp, pid_roll.ki, pid_roll.kd, pid_reinit_dt,
                     pid_roll.output_min, pid_roll.output_max, pid_roll.integral_min, pid_roll.integral_max);
            PID_Init(&pid_pitch, pid_pitch.kp, pid_pitch.ki, pid_pitch.kd, pid_reinit_dt,
                     pid_pitch.output_min, pid_pitch.output_max, pid_pitch.integral_min, pid_pitch.integral_max);
            PID_Init(&pid_yaw, pid_yaw.kp, pid_yaw.ki, pid_yaw.kd, pid_reinit_dt,
                     pid_yaw.output_min, pid_yaw.output_max, pid_yaw.integral_min, pid_yaw.integral_max);
            printf("Targets and PIDs have been reset.\r\n");
        } else {
            printf("Unknown command: %s\r\n", (char*)g_command_buffer);
        }

        g_command_buffer_idx = 0; // 명령어 처리 후 버퍼 인덱스 리셋
        memset(g_command_buffer, 0, UART_CMD_RX_BUFFER_SIZE); // 버퍼 클리어
        g_command_ready_flag = false; // 플래그 리셋

        printf(">"); // 다음 명령어 입력 프롬프트

        // 명령어 처리 후, 새로운 명령어 수신을 위해 다시 인터럽트 활성화
        if (HAL_UART_Receive_IT(g_huart_cmd, g_rx_buffer_char, 1) != HAL_OK) {
             printf("Error: Failed to restart UART RX IT after command processing\r\n>");
        }
    }
}
