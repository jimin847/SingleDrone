#include "uart_commands.h"
#include "main.h"     // Error_Handler, AXIS_ROLL 등 main.h 정의 사용
#include "pid.h"      // PID_Init, PIDController 등 PID 리셋 시 필요
#include "imu.h"      // AXIS_ROLL, AXIS_PITCH, AXIS_YAW 정의를 위해 추가!
#include <stdio.h>    // sscanf, printf 사용
#include <string.h>   // strncmp, strlen, strcspn 사용 (개행 문자 제거용)
#include <stdlib.h>   // atof (문자열 to float 변환)

// --- Private (Static) Global Variables for this module ---
static UART_HandleTypeDef *g_huart_cmd_handle_uart_commands; // 변수명 충돌 방지 위해 _uart_commands 추가
static uint8_t g_rx_char_buffer_uart_commands[1];
static uint8_t g_command_buffer_uart_commands[UART_CMD_RX_BUFFER_SIZE];
static uint16_t g_command_buffer_idx_uart_commands = 0;
static volatile bool g_command_ready_flag_uart_commands = false;
static volatile bool g_is_typing_command_uart_commands = false; // 사용자 입력 중 플래그

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
    g_huart_cmd_handle_uart_commands = huart_handle_to_use; // 수정된 변수명 사용
    g_command_buffer_idx_uart_commands = 0;                 // 수정된 변수명 사용
    g_command_ready_flag_uart_commands = false;             // 수정된 변수명 사용
    g_is_typing_command_uart_commands = false;              // 수정된 변수명 사용
    memset(g_command_buffer_uart_commands, 0, UART_CMD_RX_BUFFER_SIZE); // 수정된 변수명 사용

    printf("\r\nUART Command Interface Initialized.\r\n");
    printf("Available commands (terminate with Enter):\r\n");
    printf("  r<value>  : Set target Roll angle (e.g., r10.5)\r\n");
    printf("  p<value>  : Set target Pitch angle (e.g., p-5.0)\r\n");
    printf("  y<value>  : Set target Yaw angle (e.g., y0.0)\r\n");
    printf("  reset     : Reset target angles and PIDs\r\n");
    printf(">");
    fflush(stdout);

    if (HAL_UART_Receive_IT(g_huart_cmd_handle_uart_commands, g_rx_char_buffer_uart_commands, 1) != HAL_OK) { // 수정된 변수명 사용
        printf("Error: Failed to start UART RX IT in UART_Commands_Init\r\n");
    }
}

// 이 함수는 stm32f1xx_it.c 의 HAL_UART_RxCpltCallback 에서 호출됨
void UART_CMD_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == g_huart_cmd_handle_uart_commands->Instance) { // 수정된 변수명 사용
        g_is_typing_command_uart_commands = true; // 수정된 변수명 사용
        uint8_t received_char = g_rx_char_buffer_uart_commands[0]; // 수정된 변수명 사용

        HAL_UART_Transmit(g_huart_cmd_handle_uart_commands, &received_char, 1, 100); // Echo back

        if (received_char == '\r' || received_char == '\n') {
            printf("\r\n");
            if (g_command_buffer_idx_uart_commands > 0) { // 수정된 변수명 사용
                g_command_buffer_uart_commands[g_command_buffer_idx_uart_commands] = '\0'; // 수정된 변수명 사용
                g_command_ready_flag_uart_commands = true; // 수정된 변수명 사용
            } else {
                printf(">");
                fflush(stdout);
                g_command_buffer_idx_uart_commands = 0; // 수정된 변수명 사용
                g_is_typing_command_uart_commands = false; // 수정된 변수명 사용
            }
        } else if (received_char == '\b' || received_char == 127) {
            if (g_command_buffer_idx_uart_commands > 0) { // 수정된 변수명 사용
                g_command_buffer_idx_uart_commands--; // 수정된 변수명 사용
                char backspace_seq[] = "\b \b";
                HAL_UART_Transmit(g_huart_cmd_handle_uart_commands, (uint8_t*)backspace_seq, sizeof(backspace_seq)-1, 100); // 수정된 변수명 사용
            }
        } else if (received_char >= 32 && received_char <= 126) {
            if (g_command_buffer_idx_uart_commands < UART_CMD_RX_BUFFER_SIZE - 1) { // 수정된 변수명 사용
                g_command_buffer_uart_commands[g_command_buffer_idx_uart_commands++] = received_char; // 수정된 변수명 사용
            } else {
                printf("\r\nError: Command buffer overflow. Clearing command.\r\n>");
                fflush(stdout);
                g_command_buffer_idx_uart_commands = 0; // 수정된 변수명 사용
                memset(g_command_buffer_uart_commands, 0, UART_CMD_RX_BUFFER_SIZE); // 수정된 변수명 사용
                g_is_typing_command_uart_commands = false; // 수정된 변수명 사용
            }
        }

        if (!g_command_ready_flag_uart_commands) { // 수정된 변수명 사용
             if (HAL_UART_Receive_IT(g_huart_cmd_handle_uart_commands, g_rx_char_buffer_uart_commands, 1) != HAL_OK) { // 수정된 변수명 사용
                 // Error
            }
        }
    }
}

void UART_CMD_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == g_huart_cmd_handle_uart_commands->Instance) { // 수정된 변수명 사용
        // ... (에러 처리 로직, 변수명 일관성 있게 수정) ...
        g_command_buffer_idx_uart_commands = 0; // 수정된 변수명 사용
        g_command_ready_flag_uart_commands = false; // 수정된 변수명 사용
        memset(g_command_buffer_uart_commands, 0, UART_CMD_RX_BUFFER_SIZE); // 수정된 변수명 사용
        if (HAL_UART_Receive_IT(g_huart_cmd_handle_uart_commands, g_rx_char_buffer_uart_commands, 1) != HAL_OK) { // 수정된 변수명 사용
            // Error
        }
    }
}

void UART_Commands_Process(void) {
    if (g_command_ready_flag_uart_commands) { // 수정된 변수명 사용
        printf("Processing CMD: [%s]\r\n", (char*)g_command_buffer_uart_commands); // 수정된 변수명 사용

        float val;
        // bool command_processed = false; // 이 변수는 현재 사용되지 않으므로 주석 처리 또는 제거

        if (g_command_buffer_uart_commands[0] == 'r') { // 수정된 변수명 사용
            val = atof((char*)g_command_buffer_uart_commands + 1); // 수정된 변수명 사용
            target_angles[AXIS_ROLL] = val;
            printf("Target Roll set to: %.2f\r\n", target_angles[AXIS_ROLL]);
        } else if (g_command_buffer_uart_commands[0] == 'p') { // 수정된 변수명 사용
            // ... (이하 유사하게 변수명 수정) ...
            val = atof((char*)g_command_buffer_uart_commands + 1);
            target_angles[AXIS_PITCH] = val;
            printf("Target Pitch set to: %.2f\r\n", target_angles[AXIS_PITCH]);
        } else if (g_command_buffer_uart_commands[0] == 'y') {
            val = atof((char*)g_command_buffer_uart_commands + 1);
            target_angles[AXIS_YAW] = val;
            printf("Target Yaw set to: %.2f\r\n", target_angles[AXIS_YAW]);
        } else if (strncmp((char*)g_command_buffer_uart_commands, "reset", 5) == 0) {
        	target_angles[AXIS_ROLL] = 0.0f;
        	    target_angles[AXIS_PITCH] = 0.0f;
        	    target_angles[AXIS_YAW] = 0.0f;

        	    // PID 상태 리셋 (pid.c/h 에 PID_Reset 함수를 만들거나 직접 접근)
        	    // 예시: PID_Init을 다시 호출하거나, 멤버 직접 초기화
        	    float current_dt_for_reset = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;
        	    float pid_output_min = -100.0f; // main.c의 초기화 값과 동일하게
        	    float pid_output_max = 100.0f;
        	    float pid_integral_min = -500.0f;
        	    float pid_integral_max = 500.0f;

        	    PID_Init(&pid_roll, pid_roll.kp, pid_roll.ki, pid_roll.kd, current_dt_for_reset, pid_output_min, pid_output_max, pid_integral_min, pid_integral_max); // 현재 게인 유지하며 리셋
        	    PID_Init(&pid_pitch, pid_pitch.kp, pid_pitch.ki, pid_pitch.kd, current_dt_for_reset, pid_output_min, pid_output_max, pid_integral_min, pid_integral_max);
        	    PID_Init(&pid_yaw, pid_yaw.kp, pid_yaw.ki, pid_yaw.kd, current_dt_for_reset, pid_output_min, pid_output_max, pid_integral_min, pid_integral_max);
        	    // 또는 더 간단하게
        	    // pid_roll.integral = 0.0f; pid_roll.prev_error = 0.0f;
        	    // pid_pitch.integral = 0.0f; pid_pitch.prev_error = 0.0f;
        	    // pid_yaw.integral = 0.0f; pid_yaw.prev_error = 0.0f;
            printf("Targets and PIDs have been reset.\r\n");
        } else {
            printf("Unknown command: [%s]\r\n", (char*)g_command_buffer_uart_commands); // 수정된 변수명 사용
        }

        g_command_buffer_idx_uart_commands = 0; // 수정된 변수명 사용
        memset(g_command_buffer_uart_commands, 0, UART_CMD_RX_BUFFER_SIZE); // 수정된 변수명 사용
        g_command_ready_flag_uart_commands = false; // 수정된 변수명 사용
        g_is_typing_command_uart_commands = false; // 명령어 처리 완료 후 타이핑 상태 해제

        printf(">");
        fflush(stdout);

        if (HAL_UART_Receive_IT(g_huart_cmd_handle_uart_commands, g_rx_char_buffer_uart_commands, 1) != HAL_OK) { // 수정된 변수명 사용
             printf("Error: Failed to restart UART RX IT after command processing\r\n>");
             fflush(stdout);
        }
    }
}

// uart_commands.c
bool UART_IsUserTypingCommand(void) { // 이 함수는 main.c에서 로그 출력 제어용
    return g_is_typing_command_uart_commands && !g_command_ready_flag_uart_commands; // 수정된 변수명 사용
}
