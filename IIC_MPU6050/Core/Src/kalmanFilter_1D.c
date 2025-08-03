/*
 * kalman_filter.c
 *
 *  Created on: Feb 20, 2025
 *      Author: zk836
 */

#include <kalmanFilter_1D.h>

void kf_init(KalmanFilter *kf, float A, float B, float Q, float H, float R, float x_init, float sigma_init) {

    kf->x = x_init;
    kf->sigma = sigma_init;

	kf->A = A;
	kf->B = B;
    kf->Q = Q;

    kf->H = H;
    kf->R = R;
    kf->K = 0.0f;

    kf->x_pred = kf->sigma_pred = 0.0f;

}

void kf_prediction(KalmanFilter *kf, float u){

	float x = kf->x;
	float sigma = kf->sigma;

	float A = kf->A;
	float B = kf->B;
	float Q = kf->Q;

	kf->x_pred = (A * x) + (B * u);
	kf->sigma_pred = (A * sigma * A) + Q;

}

void kf_correction(KalmanFilter *kf, float z){

	float x_pred = kf->x_pred;
	float sigma_pred = kf->sigma_pred;

	float H = kf->H;
	float R = kf->R;

	float z_hat = H * x_pred;
	kf->K = sigma_pred*H / (H*sigma_pred*H + R);

	kf->x = x_pred + kf->K * (z - z_hat);
	kf->sigma = (1 - (kf->K * H)) * sigma_pred;

}
