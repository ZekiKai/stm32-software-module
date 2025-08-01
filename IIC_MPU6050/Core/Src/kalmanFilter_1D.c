/*
 * kalman_filter.c
 *
 *  Created on: Feb 20, 2025
 *      Author: zk836
 */

#include <kalmanFilter_1D.h>

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R) {
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0f;
    kf->K = 0.0f;
    kf->X_hat = 0.0f;
}

float KalmanFilter_Update(KalmanFilter *kf, float measurement) {

    kf->P = kf->P + kf->Q;                 // (e‘=x-x_hat’)协方差时间更新(k-1 -> k’) --  P_k‘ = A*P_k-1*A^ + Q

    kf->K = kf->P / (kf->P + kf->R);       // 卡尔曼增益更新 K_k = (P_k’*H^) / ((H*P_k‘*H^) + R)
    kf->X_hat += kf->K * (measurement - kf->X_hat); //  数据融合 X_hat = X_hat’ + K_k*(X_measure - X_hat‘)
    kf->P *= (1 - kf->K);                  // （e = x-x_hat）协方差后验更新(k‘ -> k) -- P_k+1 = (I - K_k+1*H^)*P_k’

    return kf->X_hat;
}
