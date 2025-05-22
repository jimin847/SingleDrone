/*
 * filter.c
 *
 *  Created on: May 2, 2025
 *      Author: Admin
 */


#include "filter.h"

// 칼만 필터 구조체
typedef struct {
    float angle;   // 추정된 각도
    float bias;    // 바이어스
    float P[2][2]; // 오차 공분산
} KalmanFilter;

static KalmanFilter kalman_filters[3];

// 튜닝 가능한 상수
static const float Q_angle = 0.001f;   // 각도 예측 노이즈
static const float Q_bias  = 0.003f;   // 바이어스 예측 노이즈
static const float R_measure = 0.03f;  // 측정 노이즈


void Filter_Init(void)
{
    for (int i = 0; i < 3; ++i) {
        kalman_filters[i].angle = 0.0f;
        kalman_filters[i].bias = 0.0f;
        kalman_filters[i].P[0][0] = 1.0f;
        kalman_filters[i].P[0][1] = 0.0f;
        kalman_filters[i].P[1][0] = 0.0f;
        kalman_filters[i].P[1][1] = 1.0f;
    }
}

// dt: 시간 주기는 고정되어 있다고 가정 (예: 0.01초 = 100Hz)
#define DT 0.01f

void Filter_Update(Axis axis, float measured_angle, float measured_gyro)
{
    KalmanFilter *kf = &kalman_filters[axis];

    // 1. 예측 단계
    float rate = measured_gyro - kf->bias;
    kf->angle += DT * rate;

    // 오차 공분산 예측
    kf->P[0][0] += DT * (DT*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_angle);
    kf->P[0][1] -= DT * kf->P[1][1];
    kf->P[1][0] -= DT * kf->P[1][1];
    kf->P[1][1] += Q_bias * DT;

    // 2. 업데이트 단계
    float y = measured_angle - kf->angle;
    float S = kf->P[0][0] + R_measure;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    // 오차 공분산 갱신
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;
}

float Filter_GetAngle(Axis axis)
{
    return kalman_filters[axis].angle;
}
