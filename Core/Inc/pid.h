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
    float integral;
    float prev_error;
} PIDController;

void PID_Init(PIDController *pid, float kp, float ki, float kd);
float PID_Compute(PIDController *pid, float setpoint, float measured);

#endif /* __PID_H__ */
