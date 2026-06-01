/*
 * KalmanFilter.h
 *
 *  Created on: Sep 4, 2024
 *      Author: saif
 */

#include "../../BIOS/BIOS.h"

#ifndef SRC_KALMANFILTER_KALMANFILTER_H_
#define SRC_KALMANFILTER_KALMANFILTER_H_

typedef struct {

	float p_predicted;
	float x_predicted;
	float p_estemated;
	float *x_estemated;
	float Kalman_A;
	float Kalman_Q;
	float *Kalman_Z;
	float Kalman_H;
	float Kalman_R;
	float KalmanGain;

} KalmanFilter_t;

void KalmanFilter_Init(float *input, float *output, float Pe, float A, float Q, float R, float H, KalmanFilter_t *kf);
void KalmanFilter_5ms(KalmanFilter_t *kf);

#endif /* SRC_KALMANFILTER_KALMANFILTER_H_ */
