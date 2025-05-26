/*
 * pid.c
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */


#include "pid.h"

// PID_Init 함수 수정: dt, out_min, out_max, int_min, int_max 인자 추가
void PID_Init(PIDController* pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max, float int_min, float int_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt; // dt 저장
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_min = int_min;
    pid->integral_max = int_max;
}

float PID_Compute(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    // I항 계산 시 dt 반영
    pid->integral += error * pid->dt;

    // 적분 포화 방지 (Anti-windup) - PID 구조체에 정의된 제한값 사용
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    // D항 계산 시 dt 반영
    float derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    // PID 제어 출력 계산
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // PID 출력 제한
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}


