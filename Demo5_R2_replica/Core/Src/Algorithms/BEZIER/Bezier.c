/*
 * Bezier.c
 *
 *  Created on: 12 Oct 2023
 *      Author: KZ
 */

#include "Bezier.h"

//Path Planning using Bezier Algorithm, users should know how to use old PathPlan library in order to use this.

//void BZ_Laser_Init(Laser_t *lsrx, Laser_t *lsry, unsigned x_swap, unsigned y_swap,
//		volatile float *imu, Bezier_t* bz){
//
//	bz->lsrx = &(lsrx->dist);
//	bz->lsry = &(lsry->dist);
//	bz->lsrx_swap = x_swap;
//	bz->lsry_swap = y_swap;
//	bz->yaw = imu;
//	bz->yaw_offset= *(bz->yaw);
//	bz->no_of_path = 10;	//discrete paths formed between start and destination
//	bz->no_of_point = 1;	//start and destination (use 2 for now, also we actually not use more than 2)
//	bz->no_target_reached = 0;
//	bz->tol_dist_path = 0.01;
//	bz->path_start = 1;
//	bz->lsr_sensor = 1;
//	bz->qei_init = 1;
//	bz->lsr_init = 1;
//	bz->lsr_start = 0;
//	bz->error_dist = 0.0;
//
//	PIDSourceInit(&(bz->error_dist), &(bz->out_dist), &(bz->dist));	//in meter
//	PIDGainInit(0.005, 1.0, 1.0, 1.0, 1.0, 0.0, 0.4, 30.0, &(bz->dist));
//	PIDDelayInit(&(bz->dist));
//
//	PIDSourceInit(&(bz->error_z), &(bz->outz), &(bz->z));	//in degree
//	PIDGainInit(0.005, 1.0, (1.0 / 30.0), 0.5, 5.0, 0.2, 0.2, 30.0, &(bz->z));
//	PIDDelayInit(&(bz->z));
//}

void BZ_QEI_Init(float *qeix, float *qeiy,volatile float *imu, Bezier_t* bz){

	bz->qeix = qeix;
	bz->qeiy = qeiy;
	bz->yaw = imu;
	bz->yaw_offset=*(bz->yaw);
	bz->no_of_path = 10;
	bz->no_target_reached = 0;
	bz->no_of_point = 1;
	bz->path_start = 1;
	bz->qei_sensor = 1;
	bz->qei_init = 1;
	bz->lsr_init = 1;
	bz->qei_start = 0;
	bz->tol_dist_path = 0.05;
	bz->error_dist = 0.0;

	PIDSourceInit(&(bz->error_dist), &(bz->out_dist), &(bz->dist));
	PIDGainInit(0.005, 1.0, 1.0, 1.0, 1.0, 0.0, 0.4, 1.0, &(bz->dist));
	PIDDelayInit(&(bz->dist));

	PIDSourceInit(&(bz->error_z), &(bz->outz), &(bz->z));
	PIDGainInit(0.005, 1.0, (1.0 / 30.0), 0.5, 5.0, 0.2, 0.2, 1.0, &(bz->z));
	PIDDelayInit(&(bz->z));
}

void BZ_PIDPathSet(float kp, float ki, float kd, Bezier_t* bz){

	bz->kp=kp;
	bz->ki=ki;
	bz->kd=kd;

	PIDGainSet(KP,kp,&(bz->dist));
	PIDGainSet(KI,ki,&(bz->dist));
	PIDGainSet(KD,kd,&(bz->dist));
}

void BZ_PIDZSet(float kp, float ki, float kd, float ku, Bezier_t* bz){

	PIDGainSet(KP,kp,&(bz->z));
	PIDGainSet(KI,ki,&(bz->z));
	PIDGainSet(KD,kd,&(bz->z));
	PIDGainSet(KU,ku,&(bz->z));
}

void BZ_QEI_start(float point[8], float ref_point[][3], Bezier_t* bz){
	bz->target_x[0] = bz->real_x;
	bz->target_y[0] = bz->real_y;
	bz->target_z[0] = bz->real_z_rad;
	bz->vel_limit = point[0];
	bz->target_x[1] = point[1];
	bz->target_y[1] = point[2];
	bz->target_z[1] = point[3];
	bz->order = (int)point[4];
	bz->tol_dist = point[5];
	bz->tol_z = point[6];
	bz->end_v = point[7];

	for(int i = 0; i < bz->order - 1; i++){
		bz->ref_point_x[i] = ref_point[i][0];
		bz->ref_point_y[i] = ref_point[i][1];
		bz->curvature[i] = ref_point[i][2];
	}

	bz->point_count = 0;
	bz->path_count = 0;
	bz->point_reached = 0;
	bz->qei_init = 0;
	bz->qei_start=1;
}

void BZ_LSR_start(float point[7], float ref_point[][3], Bezier_t* bz){
	bz->target_x[0] = bz->real_x;
	bz->target_y[0] = bz->real_y;
	bz->target_z[0] = bz->real_z_rad;
	bz->vel_limit = point[0];
	bz->target_x[1] = point[1];
	bz->target_y[1] = point[2];
	bz->target_z[1] = point[3];
	bz->order = point[4];
	bz->tol_dist = point[5];
	bz->tol_z = point[6];

	for(int i = 0; i < bz->no_of_point - 1; i++){
		bz->ref_point_x[i] = ref_point[i][0];
		bz->ref_point_y[i] = ref_point[i][1];
		bz->curvature[i] = ref_point[i][2];
	}
	bz->lsr_start = 1;
	bz->point_count = 0;
	bz->lsr_init = 0;
	bz->stop_flag = 0;
	bz->stop_delay = 0;
	bz->point_reached = 0;
}


void BZ_MAT_Send(float x_pos, float y_pos, UART_HandleTypeDef* huartx, Bezier_t* bz){
	uint8_t i;
	if(HAL_GetTick() - bz->bz_tick > 100){
		memcpy(&bz->x_data,&x_pos,4);
		memcpy(&bz->y_data,&y_pos,4);

		for(i = 0; i < 4; i++){
			bz->data_buff[i] = bz->x_data[i];
		}
		for(i = 4; i < 8; i++){
			bz->data_buff[i] = bz->y_data[i - 4];
		}

		HAL_UART_Transmit(huartx, bz->data_buff, 8, 10);

		bz->bz_tick = HAL_GetTick();
	}
}

//1 or 5ms interrupt
void Bezier_PathPlan(Bezier_t* bz){

	if (bz->path_start) {
		//IMU (algorithm at imu library)
		bz->real_z = *(bz->yaw) - bz->yaw_offset;
		bz->real_z_rad = (bz->real_z / 180.0) * M_PI;

		//QEI data
		if(bz->qei_sensor){
			bz->pos_x = *(bz->qeix);
			bz->pos_y = *(bz->qeiy);
			bz->real_x = bz->pos_x * cosf(bz->real_z_rad) + bz->pos_y * sinf(bz->real_z_rad);
			bz->real_y = -(bz->pos_x) * sinf(bz->real_z_rad) + bz->pos_y * cosf(bz->real_z_rad);
		}
		//LASER Data
		if(bz->lsr_sensor){
			bz->real_lsrx = *(bz->lsrx) * cosf(bz->real_z_rad);
			bz->real_lsry = *(bz->lsry) * cosf(bz->real_z_rad);
		}
	}

	if(!bz->lsr_init || !bz->qei_init)	BZ_Generate_Path(bz);

	//position data
	if(bz->path_start && bz->lsr_start){
		if(!bz->lsr_init){
			BZ_PID_Config(bz);
			bz->lsr_init = 1;	//path and PID generation done
		}

		if (bz->path_count >= bz->no_of_path - 1 && bz_threshold){	//point reached
			bz->point_reached = 1;
		}
		else if(bz->path_count < bz->no_of_path - 1 && bz_path_threshold){	//path reached
			bz->path_count++;
			BZ_Error_Update(bz);
			BZ_PID_Config(bz);
		}
		else
			bz->point_reached = 0;
		BZ_Heading_Control(bz);
	}

	//QEI algorithm
	if(bz->path_start && bz->qei_start){
		BZ_Error_Update(bz);

		if(!bz->qei_init){
			BZ_PID_Config(bz);
			bz->qei_init = 1;
		}
		if (bz->path_count >= bz->no_of_path - 1 && bz_threshold){	//point reached
			bz->point_reached = 1;
		}
		else if(bz->path_count < bz->no_of_path - 2 && bz_path_threshold){	//path reached
			bz->path_count++;
			BZ_Error_Update(bz);
			BZ_PID_Config(bz);
		}
		else if(bz->path_count == bz->no_of_path - 2 && bz_path_threshold){
			bz->path_count++;
			bz->qei_init = 0;
			bz->vel_limit = bz->end_v;
			bz->kp = 1.2;
			bz->ki = 0.0;
			bz->kd = 0.001;
			BZ_Error_Update(bz);
			BZ_PID_Config(bz);
			bz->qei_init = 1;
		}
		else
			bz->point_reached = 0;
	}

	//point reached statement here (qei and laser)
	if(bz->point_reached){
		bz->lsr_start = 0;
		bz->qei_start = 0;
		bz->point_reached = 0;
		bz->no_target_reached++;
	}
	BZ_Error_Update(bz);

	if(bz->error_x||bz->error_y)
		BZ_Heading_Control(bz);
	else
		bz->heading = 0.0;

//	if(bz->error_z < bz->tol_z){
//		bz->error_z = 0.0;
//		PIDDelayInit(&(bz->z));
//	}

	if(bz->path_start){
		PID(&(bz->z));
		PID(&(bz->dist));

			//robot velocity xy component (robot orientation)
		bz->rux = -(bz->vel_limit + bz->out_dist) * sinf(bz->real_z_rad - bz->heading);
		bz->ruy =  (bz->vel_limit + bz->out_dist) * cosf(bz->real_z_rad - bz->heading);
	}
}

//************************************************************************************************************************

void BZ_Error_Update(Bezier_t* bz){
	if(bz->lsr_sensor){
		bz->error_x = bz->real_lsrx - bz->path_x[bz->path_count];
		bz->error_y = bz->real_lsry - bz->path_y[bz->path_count];
		bz->error_z = bz->target_z[bz->point_count] - bz->real_z;	//bz->point_count = 1
		bz->error_dist = sqrt(SQ(bz->error_x) + SQ(bz->error_y));	//should be short distance

		if (bz->lsrx_swap)	//will effect pp->heading
			bz->error_x = -bz->error_x;
		if (bz->lsry_swap)
			bz->error_y = -bz->error_y;
	}
	else if(bz->qei_sensor){
		bz->error_x = bz->path_x[bz->path_count] - bz->real_x;
		bz->error_y = bz->path_y[bz->path_count] - bz->real_y;
		bz->error_z = bz->target_z[1] - bz->real_z;
		bz->error_dist = sqrt(SQ(bz->error_x) + SQ(bz->error_y));
	}
}

void BZ_PID_Config(Bezier_t* bz){
	if (fabs(bz->error_dist)) {
		PIDGainSet(KE, 1.0 / fabs(bz->error_dist), &(bz->dist));
	} else {
		PIDGainSet(KE, 1.0, &(bz->dist));
	}

	if(!bz->lsr_init || !bz->qei_init){
		bz->ku_dist = bz->vel_limit;

		PIDGainSet(KU, bz->ku_dist, &(bz->dist));
		PIDGainSet(KP, bz->kp, &(bz->dist));
		PIDGainSet(KI, bz->ki, &(bz->dist));
		PIDGainSet(KD, bz->kd, &(bz->dist));
	}

//	if(bz->point_reached){
//		PIDGainSet(KE, 1.0, &(bz->dist));
//		PIDGainSet(KP, bz->kp, &(bz->dist));
//		PIDGainSet(KI, bz->ki, &(bz->dist));
//		PIDGainSet(KD, bz->kd, &(bz->dist));
//	}

	PIDDelayInit(&(bz->dist));
	PIDDelayInit(&(bz->z));
}

void BZ_Generate_Path(Bezier_t* bz){
	bz->error_dist = 0.0;	//reset
	bz->pos_x_buff[0] = bz->target_x[0];
	bz->pos_y_buff[0] = bz->target_y[0];

	bz->pos_x_buff[1] = bz->ref_point_x[0];
	bz->pos_y_buff[1] = bz->ref_point_y[0];

	bz->pos_x_buff[bz->order] = bz->target_x[1];
	bz->pos_y_buff[bz->order] = bz->target_y[1];

	for(int i = 0; i < bz->no_of_path;i++){
		bz->path_cnt += 1.0/(float)bz->no_of_path;
		bz->path_x[i] = BZ_Path(bz->order, bz->path_cnt, bz->pos_x_buff, bz->curvature);
		bz->path_y[i] = BZ_Path(bz->order, bz->path_cnt, bz->pos_y_buff, bz->curvature);
	}
	bz->path_cnt = 0.0;
}

void BZ_Heading_Control(Bezier_t* bz){
	if(!bz->error_x && bz->error_y){
		if(bz->error_y > 0.0)
			bz->heading = 0.0;
		else if(bz->error_y < 0.0)
			bz->heading = M_PI;
	}
	else if(bz->error_x && !bz->error_y){
		if(bz->error_x > 0.0)
			bz->heading = M_PI/2.0;
		else if(bz->error_x < 0.0)
			bz->heading = -(M_PI/2.0);
	}
	else
		bz->heading = atan2f(bz->error_x, bz->error_y);	//range = [2*M_PI,-2*M_PI], origin = robot front
}

float factorial(int num){
	int ans = 1;

	for(int i=2;i<=num;i++){
		ans *= i;
	}
	return (float)ans;
}

float BZ_Path(int n, float t, float pos[], float w[]){
	if(n > 0){
		float A = factorial(n);
		float B, C, omega[5] = {0,0,0,0,0}, temp[2] = {0,0};

		omega[0] = 1.0;
		omega[1] = w[0];
		omega[n] = 1.0;

		for(int i=0;i<=n;i++){	//n = order, t = counter [0,1]
			B = factorial(n - i);
			C = factorial(i);

			temp[0] += (A/(B*C))*powf(1.0 - t,(float)(n - i))*powf(t,(float)i)*pos[i]*omega[i];
			temp[1] += (A/(B*C))*powf(1.0 - t,(float)(n - i))*powf(t,(float)i)*omega[i];
		}
		return (temp[0]/temp[1]);
	}
	else
		return 0;
}
