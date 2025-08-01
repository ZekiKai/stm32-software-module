/*
 * kalman_filter.h
 *
 *  Created on: Feb 20, 2025
 *      Author: zk836
 */

#ifndef INC_KALMANFILTER_1D_H_
#define INC_KALMANFILTER_1D_H_

typedef struct {

    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
    float P;      // 估计误差协方差
    float K;      // 卡尔曼增益

    float X_hat;  // 估计值 （先验估计->后验估计）

} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R);

float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif /* INC_KALMANFILTER_1D_H_ */
