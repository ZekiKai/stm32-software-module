/*
 * kalman_filter.h
 *
 *  Created on: Feb 20, 2025
 *      Author: zk836
 */

#ifndef INC_KALMANFILTER_1D_H_
#define INC_KALMANFILTER_1D_H_

typedef struct {

/* Kalman Filter for Linear Gaussian System
 *
 * 	  motion model: x = Ax + Bu + Q;
 * 	  measurement model: z = Hx + R;
 *
 * 	  state mean: x and state covariance: sigma;
 *
 * 	  prediction for state:
 * 		  x_pred = Ax + Bu;
 * 		  sigma_pred = A*sigma*A' + Q;
 *
 * 	  correction for state:
 * 		  z_hat = H*x_pred;
 * 		  K = sigma_pred*H'/(H*sigma_pred*H' + R);
 * 		  x = x_pred + K*(z - z_hat);
 * 		  sigma = (I - K*H)*sigma_pred;
 *
 */

	float A, B, Q;
	float H, R;
	float K;

    float x, sigma;
    float x_pred, sigma_pred;

} KalmanFilter;

void kf_init(KalmanFilter *kf, float A, float B, float Q, float H, float R, float x_int, float sigma_init);

void kf_prediction(KalmanFilter *kf, float u);

void kf_correction(KalmanFilter *kf, float z);

#endif /* INC_KALMANFILTER_1D_H_ */
