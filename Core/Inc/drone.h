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
void Drone_Update(float roll_input, float pitch_input, float yaw_input);
DroneState* Drone_GetState(void);

#endif /* INC_DRONE_H_ */
