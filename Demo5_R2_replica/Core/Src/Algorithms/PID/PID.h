/*
 * PID.h
 *
 *  Created on: Nov 28, 2025
 *      Author: mua
 */

#ifndef SRC_PID_PID_H_
#define SRC_PID_PID_H_




/*********************************************/
/*          Define                           */
/*********************************************/

/*********************************************/




/*********************************************/
/*          Enumarator                      _ */
/*********************************************/
enum {TS = 0, SAT, KE, KU, KP, KI, KD, KN, B_0, B_1, C_1, D_0, D_1};
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/
typedef struct{
	struct{
		unsigned s_flag	: 1;	/* Saturation 		*/
//		unsigned i_flag : 1;	/* Source Init Flag */
//		unsigned pd_flag: 1;
	};
	float *error;
	float *out_put;
	float K[13];
	float i_delay[2];
	float d_delay[2];
	float s_delay;
}PID_t;
/*********************************************/




/*********************************************/
/*           Function Prototype              */
/*********************************************/
void PIDSourceInit (float *in, float *out, PID_t *pid);
void PIDGainInit (float ts, float sat, float ke, float ku, float kp, float ki,
						float kd, float kn, PID_t *pid);
void PIDGainSet (unsigned char a, float value, PID_t *pid);
void PIDCoeffCalc (PID_t *pid);
void PIDDelayInit (PID_t *pid);
char PIDsSaturared (PID_t *pid);
void PID (PID_t *pid);
/*********************************************/


#endif /* SRC_PID_PID_H_ */
