/*
 * filter.h
 *
 *  Created on: May 2, 2025
 *      Author: Admin
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_


#include "imu.h"
#include <stdint.h>

void Filter_Init(void);
// Filter_Update 함수 시그니처 변경: dt를 인자로 받음
void Filter_Update(Axis axis, float measured_angle, float measured_gyro, float dt);

float Filter_GetAngle(Axis axis);

#endif /* INC_FILTER_H_ */
