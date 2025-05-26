#ifndef INC_UART_COMMANDS_H_
#define INC_UART_COMMANDS_H_

#include "stm32f1xx_hal.h" // HAL_UART_HandleTypeDef 사용 위함
#include <stdbool.h>       // bool 타입 사용 위함
#include <stdint.h>        // uint8_t, uint16_t 등 사용 위함

// UART 수신 버퍼 크기 정의
#define UART_CMD_RX_BUFFER_SIZE 32 // 예시 크기, 명령어 길이에 맞게 조절

// UART 명령어 처리 모듈 초기화 함수
void UART_Commands_Init(UART_HandleTypeDef *huart_handle_to_use);

// 메인 루프에서 주기적으로 호출될 명령어 처리 함수
void UART_Commands_Process(void);

// stm32f1xx_it.c의 HAL_UART_RxCpltCallback에서 호출될 함수
void UART_CMD_RxCpltCallback(UART_HandleTypeDef *huart);

// stm32f1xx_it.c의 HAL_UART_ErrorCallback에서 호출될 함수 (선택 사항)
void UART_CMD_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_COMMANDS_H_ */
