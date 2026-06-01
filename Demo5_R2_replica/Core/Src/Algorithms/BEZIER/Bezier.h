/*
 * Bezier.h
 *
 *  Created on: 12 Oct 2023
 *      Author: KZ
 */

#ifndef SRC_BEZIER_BEZIER_H_
#define SRC_BEZIER_BEZIER_H_

#include "../../adapter.h"
#include "../PID/PID.h"
#include <math.h>
//#include "../common.h"
//#include "../LASER/laser.h"

#define bz_threshold  		fabs(bz->error_dist) <= bz->tol_dist && fabs(bz->error_z) <= bz->tol_z
#define bz_path_threshold	fabs(bz->error_dist) <= bz->tol_dist_path && fabs(bz->error_z) <= bz->tol_z

typedef struct{

	float *qeix;
	float *qeiy;
	volatile float *yaw;
	float *lsrx;
	float *lsry;
	float real_lsrx;
	float real_lsry;
	float prev_yaw;
	float real_z;
	float prev_real_z;
	float real_z_rad;
	float setz;
	float yaw_offset;
	float yaw_turn;
	float pos_x;
	float prev_x;
	float real_x;
	float prev_real_x;
	float pos_y;
	float prev_y;
	float real_y;
	float prev_real_y;
	float del_pos_x;
	float del_pos_y;
//
	float heading;
	float end_v;

//	float vx;
//	float vy;

	float pos_x_buff[15];
	float pos_y_buff[15];
	float ref_point_x[15];
	float ref_point_y[15];
	float curvature[15];

	float target_x[2];
	float target_y[2];
	float target_z[2];
	float vel_limit;
	float ku_dist;	//speed limit
	int order;
	float kp,ki,kd;

	float error_x;
	float error_y;
	float error_dist;
	float error_z;

	float out_dist;
	float outz;

	float rux;
	float ruy;

	float v1,v2,v3,v4;

	float tol_z,tol_dist_path, tol_dist;

	PID_t dist;
	PID_t z;

	int point_count;	//start point (1st)
	int path_count;	//1st to 20th points (not include start points, but include end point)
	int no_of_path;
	int no_of_point;
	int   point_reached;
	float path_cnt;	//t
	float path_x[20];	//initial_x[n] = path_x[0]
	float path_y[20];
	float path_z[20];
	float path_z_cnt[20];	//qei only
	float pp_rad_ptx;
	float pp_rad_pty;
	float yaw_kp;

	uint8_t no_target_reached;	//count no of pathplan completed

	uint8_t x_data[4];
	uint8_t y_data[4];
	uint8_t data_buff[8];

	uint32_t bz_tick;
	uint32_t stop_delay; //for tri omni position correcting purpose
	uint8_t stop_flag;

	union{
		uint16_t Halfword;
		struct{
			unsigned path_start		:	1;
			unsigned qei_start  	:	1;
			unsigned lsr_init		:	1;
			unsigned qei_init		:	1;
			unsigned lsr_start		:	1;
			unsigned lsry_swap		:	1;
			unsigned lsrx_swap		:	1;
			unsigned rotate			:	1;
			unsigned lsr_sensor		:	1;
			unsigned qei_sensor		:	1;
			unsigned bit10			:	1;
			unsigned bit11			:	1;
			unsigned bit12			:	1;
			unsigned bit13			:	1;
			unsigned bit14			:	1;
			unsigned bit15			:	1;
		};
	};
}Bezier_t;

Bezier_t bz;

//void BZ_Laser_Init(Laser_t *lsrx, Laser_t *lsry, unsigned x_swap, unsigned y_swap, volatile float *imu, Bezier_t* bz);
void BZ_QEI_Init(float *qeix, float *qeiy,volatile float *imu, Bezier_t* bz);
void BZ_PIDPathSet(float kp, float ki, float kd, Bezier_t* bz);
void BZ_PIDZSet(float kp, float ki, float kd, float ku, Bezier_t* bz);
void BZ_QEI_start(float point[8], float ref_point[][3], Bezier_t* bz);
void BZ_LSR_start(float point[7], float ref_point[][3], Bezier_t* bz);
void BZ_MAT_Send(float x_pos, float y_pos, UART_HandleTypeDef* huartx, Bezier_t* bz);
void Bezier_PathPlan(Bezier_t* bz);
void BZ_Error_Update(Bezier_t* bz);
void BZ_PID_Config(Bezier_t* bz);
void BZ_Heading_Control(Bezier_t* bz);
void BZ_Generate_Path(Bezier_t* bz);
float BZ_Path(int n, float t, float pos[], float w[]);
#endif /* SRC_BEZIER_BEZIER_H_ */
