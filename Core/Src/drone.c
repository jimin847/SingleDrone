/*
 * drone.c
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */


#include "drone.h"

//실제 장비 오기전 시뮬하기위한 라이브러리
#include <stdlib.h>  // rand()
#include <time.h>    // time()

#define DELTA_TIME 0.01f  // 10ms 주기 (100Hz)

// 내부 상태 변수
static DroneState drone_state = {
    .angle = {0.0f, 0.0f, 0.0f},
    .angVel = {0.0f, 0.0f, 0.0f},
    .prev_angle = {0.0f, 0.0f, 0.0f}
};

void Drone_Init(void) {
	 for (int i = 0; i < 3; i++) {
	        drone_state.angle[i] = 0.0f;
	        drone_state.angVel[i] = 0.0f;
	        drone_state.prev_angle[i] = 0.0f;
	    }
}


// 드론 상태 업데이트 (입력된 회전값 적용 + 각속도 계산)
void Drone_Update(float roll_input, float pitch_input, float yaw_input) {
	float inputs[3] = {roll_input, pitch_input, yaw_input};

	for (int i = 0; i < 3; i++) {
		// 각도 갱신: angle += 입력 * dt
		drone_state.angle[i] += inputs[i] * DELTA_TIME;

		// 각속도 계산: (현재 각 - 이전 각) / dt
		drone_state.angVel[i] = (drone_state.angle[i] - drone_state.prev_angle[i]) / DELTA_TIME;

		 // 이전 각도 저장
		drone_state.prev_angle[i] = drone_state.angle[i];
	}
}

DroneState* Drone_GetState(void) {
    return &drone_state;
}


