/*
 * imu.c
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */


#include "imu.h"
#include "drone.h"

#include <stdlib.h>   // rand()
#include <time.h>     // time() (필요시)
#include <math.h>     // sin, cos 등 가능

#define SENSOR_NOISE_LEVEL  0.5f  // 시뮬레이션용 노이즈 크기


// 간단한 노이즈 함수 (임시)
static float add_noise(float value) {
	float noise = ((float)rand() / RAND_MAX - 0.5f) * 2 * SENSOR_NOISE_LEVEL;
	    return value + noise;
}


void IMU_Init(void) {
	// 시뮬레이션이라면 랜덤값 초기화
	    srand(0);
}

float IMU_GetRawAngle(Axis axis) {
    const DroneState* state = Drone_GetState();
    return add_noise(state->angle[axis]);
}

float IMU_GetRawGyro(Axis axis) {
    const DroneState* state = Drone_GetState();
    return add_noise(state->angVel[axis]);
}
