/*
 * pid.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */

#ifndef __PID_H__
#define __PID_H__

#define MAX_INTEGRAL 1000.0f
#define MIN_INTEGRAL -1000.0f

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;           // 샘플링 시간 (초 단위)
    float integral;
    float prev_error;
    float integral_min; // 적분항 최소 제한
    float integral_max; // 적분항 최대 제한
    float output_min;   // PID 출력 최소 제한
    float output_max;   // PID 출력 최대 제한
} PIDController;

// PID_Init 함수 시그니처 변경: dt, out_min, out_max, int_min, int_max 추가
void PID_Init(PIDController *pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max, float int_min, float int_max);

// PID_Compute 함수 시그니처는 동일하게 유지하거나, dt를 인자로 받을 수도 있음 (여기서는 구조체 멤버 사용)
float PID_Compute(PIDController *pid, float setpoint, float measured);

#endif /* __PID_H__ */
