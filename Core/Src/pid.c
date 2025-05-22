/*
 * pid.c
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */


#include "pid.h"

void PID_Init(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

float PID_Compute(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    pid->integral += error;

    // 적분 포화 방지
    if (pid->integral > MAX_INTEGRAL) pid->integral = MAX_INTEGRAL;
    if (pid->integral < MIN_INTEGRAL) pid->integral = MIN_INTEGRAL;

    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}


