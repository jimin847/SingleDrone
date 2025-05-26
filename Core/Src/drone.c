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


// 드론 상태 업데이트 (입력된 각속도와 dt를 사용하여 각도 업데이트 및 각속도 계산)
// 입력값은 각속도(rad/s 또는 deg/s)로 가정
void Drone_Update(float roll_rate_input, float pitch_rate_input, float yaw_rate_input, float dt) {
	float inputs[3] = {roll_rate_input, pitch_rate_input, yaw_rate_input};

    // dt가 0 이하이면 업데이트하지 않음 (0으로 나누기 방지)
    if (dt <= 0.0f) {
        return;
    }

	for (int i = 0; i < 3; i++) {
		// 이전 각도 저장 (각속도 계산용)
		// drone_state.prev_angle[i] = drone_state.angle[i]; // 이 줄은 아래로 이동

		// 새 각도 업데이트 (입력은 각속도로 가정, angle = angle + angular_velocity * dt)
		drone_state.angle[i] += inputs[i] * dt;

		// 각속도 계산 (현재는 입력을 그대로 사용하거나, 필터링된 자이로 값을 사용)
        // Drone_Update의 입력이 "목표 각속도" 또는 "모터에 의한 실제 각속도"라면,
        // drone_state.angVel은 이 입력값을 따르거나, 더 복잡한 동역학 모델을 따를 수 있음.
        // 현재 구현은 입력받은 각속도를 그대로 상태의 각속도로 사용한다고 볼 수 있음.
        // 또는, 각도 변화로부터 계산한다면 다음과 같이 할 수 있으나, 이는 입력과 중복될 수 있음.
        // drone_state.angVel[i] = (drone_state.angle[i] - drone_state.prev_angle[i]) / dt;
        // 여기서는 입력된 각속도를 그대로 사용한다고 가정하고, prev_angle은 다음 스텝의 각도 변화량 계산을 위해 남겨둠.
        drone_state.angVel[i] = inputs[i]; // 또는 더 복잡한 동역학 모델

        // 이전 각도 저장 (다음 반복에서 각도 변화 계산용 - 현재는 angVel을 직접 사용하므로, prev_angle은 사실상 필요 없을 수 있음)
        // 만약 angVel을 각도 미분으로 계속 계산한다면 필요.
        // drone_state.prev_angle[i] = drone_state.angle[i]; // 이 로직은 Drone_Update의 목적에 따라 달라짐
	}
}

DroneState* Drone_GetState(void) {
    return &drone_state;
}

// --- 다음 단계에서 추가될 함수 예시 (믹싱 로직 등) ---
// void Drone_Run_Control(PIDController* pid_roll, /* ... */) {
//     // PID 계산
//     // 믹싱 로직
//     // 액추에이터 값 계산
// }
//
// void Drone_Apply_Actuators(const ActuatorOutputs_t* actuators) {
//     // 실제 PWM 제어 또는 시뮬레이션 출력
// }

