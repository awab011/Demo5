/*
 * KalmanFilter.c
 *
 *  Created on: Sep 4, 2024
 *      Author: saif
 */

#include "KalmanFilter.h"

void KalmanFilter_Init(float *input, float *output, float Pe, float A, float Q, float R, float H, KalmanFilter_t *kf){

	kf->p_predicted = 0.0;
	kf->x_predicted = 0.0;
	kf->p_estemated = 1.0;
	kf->x_estemated = output;
	kf->Kalman_A 	= A;
	kf->Kalman_Q 	= Q; 	// Process noise
	kf->Kalman_Z 	= input;
	kf->Kalman_H 	= H;
	kf->Kalman_R 	= R; 	// Measurement noise
	kf->KalmanGain 	= 0.0;

}

void KalmanFilter_5ms(KalmanFilter_t *kf){

	kf->x_predicted = *(kf->x_estemated) * kf->Kalman_A;
	kf->p_predicted = (kf->Kalman_A * kf->Kalman_A) * kf->p_estemated + kf->Kalman_Q;
	kf->KalmanGain = kf->p_predicted * kf->Kalman_H * 1/(kf->Kalman_H * kf->p_predicted + kf->Kalman_R);
	*(kf->x_estemated) = kf->x_predicted + ( *(kf->Kalman_Z) - kf->Kalman_H * kf->x_predicted) * kf->KalmanGain;
	kf->p_estemated = kf->p_predicted - kf->KalmanGain * kf->Kalman_H * kf->p_predicted;

}
