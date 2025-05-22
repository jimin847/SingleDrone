/*
 * imu.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "drone.h"

typedef enum {
	AXIS_ROLL = 0,
	AXIS_PITCH,
	AXIS_YAW
} Axis;

void IMU_Init(void);

// 드론 상태 기반 센서 읽기 함수
float IMU_GetRawAngle(Axis axis);
float IMU_GetRawGyro(Axis axis);

#endif /* INC_IMU_H_ */
