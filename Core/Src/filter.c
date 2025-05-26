/*
 * filter.c
 *
 *  Created on: May 2, 2025
 *      Author: Admin
 */


#include "filter.h"
#include "imu.h" // Axis 열거형을 사용하기 위해 (filter.h에 이미 include 되어 있음)
#include <math.h>

// 칼만 필터 구조체
typedef struct {
    float angle;   // 추정된 각도
    float bias;    // 바이어스
    float P[2][2]; // 오차 공분산
} KalmanFilter;

static KalmanFilter kalman_filters[3]; // 각 축(Roll, Pitch, Yaw)에 대한 칼만 필터

// 튜닝 가능한 상수 (프로세스 노이즈 공분산 Q, 측정 노이즈 공분산 R)
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




// Filter_Update 함수 수정: dt를 인자로 받음
void Filter_Update(Axis axis, float measured_angle, float measured_gyro, float dt)
{
    // dt가 0 이하이면 업데이트하지 않음
    if (dt <= 0.0f) {
        return;
    }

    KalmanFilter *kf = &kalman_filters[axis];

    // 1. 예측 단계
    float rate = measured_gyro - kf->bias;
    kf->angle += dt * rate; // dt 사용

    // 오차 공분산 예측
    // (수식 검토 필요 - 일반적인 칼만 필터 문헌과 비교하여 Q 행렬 적용 방식 확인)
    // 아래는 기존 코드를 유지하되 dt를 사용하도록 변경
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] = kf->P[0][1]; // 대칭성 유지
    kf->P[1][1] += Q_bias * dt; // Q_bias에 dt를 곱하는 것이 맞는지, 아니면 Q_bias 자체가 dt^2와 관련된 항인지 확인 필요
                               // (일반적으로 Q 행렬의 대각 성분에 Q_angle*dt^2, Q_bias*dt^2 와 유사한 형태로 들어감)
                               // 또는 더 단순하게는 P[0][0] += Q_angle; P[1][1] += Q_bias; 형태로 dt를 Q_angle, Q_bias 자체에 포함시켜 튜닝

    // 2. 업데이트 단계
    float y = measured_angle - kf->angle; // 측정 잔차

    float S = kf->P[0][0] + R_measure; // 측정 잔차 공분산

    // S가 0에 매우 가까우면 나누기 오류 방지 (선택적)
    if (fabsf(S) < 1e-9f) { // 또는 적절한 작은 값
        // 오류 처리 또는 업데이트 건너뛰기
        return;
    }

    float K0 = kf->P[0][0] / S; // 칼만 이득
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

    // 공분산 행렬 대칭성 유지 (선택적이지만 권장)
    kf->P[1][0] = kf->P[0][1];
    // 공분산 행렬이 양의 정부호(positive definite)를 유지하는지 확인하는 로직 추가 가능 (고급)
}


float Filter_GetAngle(Axis axis)
{
    return kalman_filters[axis].angle;
}
