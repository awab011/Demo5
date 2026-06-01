/*
 * KF.h
 *
 *  Created on: 24 Sep 2023
 *      Author: KZ
 */

#ifndef SRC_KF_KF_H_
#define SRC_KF_KF_H_

#include "../../BIOS/BIOS.h"

typedef struct{
  float err_measure;
  float err_estimate;
  float last_err_estimate;
  float q;
  float current_estimate;
  float last_estimate;
  float kalman_gain;

  float* kalman_input;
  float* kalman_output;
}KALMANFILTER_t;

void KalmanFilterInit(float*,float*,float, float, float, KALMANFILTER_t*);
void KalmanFilter (KALMANFILTER_t *kf);

#endif /* SRC_KF_KF_H_ */
