/*
 * KF.c
 *
 *  Created on: 24 Sep 2023
 *      Author: KZ
 */
#include "../KF/KF.h"

#include <math.h>
/*
 * KalmanFilterInit(&input,&output,0.05,2.00,100,&(kf->lsr));
 * 1)Input can be ADC value (0 to 4095) or Distance (in meter).
 * 2)If input is ADC, output is also ADC value, which is filtered.
 * 3)Run the controller in 5ms environment.
 */
void KalmanFilterInit(float* kalman_input,float* kalman_output,
						float mea_e, float est_e, float noise, KALMANFILTER_t *kf){

	kf->kalman_input = kalman_input;
	kf->kalman_output= kalman_output;

	kf->err_measure = mea_e;
	kf->err_estimate = est_e;
	kf->q = noise;
}

/* --------------------------------------------------------------------------------------------------------------- */

void KalmanFilter (KALMANFILTER_t *kf){
	kf->kalman_gain = (kf->last_err_estimate + kf->q)/(kf->last_err_estimate + kf->q + kf->err_measure);
	kf->current_estimate = kf->current_estimate + kf->kalman_gain * (*(kf->kalman_input) - kf->current_estimate);
	kf->err_estimate = kf->kalman_gain * (kf->err_measure);
	kf->last_err_estimate = kf->err_estimate;

	*(kf->kalman_output) = kf->current_estimate;
}

