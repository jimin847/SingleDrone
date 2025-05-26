/*
 * drone.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */

#ifndef INC_DRONE_H_
#define INC_DRONE_H_

typedef struct {
	float angle[3];     // Roll, Pitch, Yaw
	float angVel[3];    // 각 속도(rad/s)
    float prev_angle[3]; // 이전 각도 (각속도 계산용)
} DroneState;

void Drone_Init(void);

// Drone_Update 함수의 시그니처 변경: dt를 인자로 받음
void Drone_Update(float roll_rate_input, float pitch_rate_input, float yaw_rate_input, float dt);
DroneState* Drone_GetState(void);

// --- 다음 단계에서 추가될 함수 프로토타입 예시 ---
// #include "main.h" // PIDController 사용을 위해 (또는 pid.h)
// typedef struct {
//    float motor_speeds[2];
//    float servo_angles[4];
//} ActuatorOutputs_t;
//
// void Drone_Run_Control(PIDController* pid_roll, PIDController* pid_pitch, PIDController* pid_yaw,
//                        float target_roll, float target_pitch, float target_yaw_rate,
//                        float current_roll, float current_pitch, float current_yaw_rate,
//                        float throttle_input, float dt, ActuatorOutputs_t* actuators);

#endif /* INC_DRONE_H_ */
