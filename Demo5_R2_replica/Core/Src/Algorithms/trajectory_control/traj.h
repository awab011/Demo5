/*
 * traj.h
 *
 *  Created on: May 14, 2024
 *      Author: ahmed amir
 *      trapezoidal trajectory control
 */

#ifndef SRC_TRAJECTORY_CONTROL_TRAJ_H_
#define SRC_TRAJECTORY_CONTROL_TRAJ_H_
//#include "../../BIOS/COM/FDCAN.h"
#include "../../BIOS/BIOS.h"
#include "../PID/PID.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

typedef struct{
//    float angle;
    float Y;
    float Yd;
    float Ydd;
    float set_pos;
    float Df;
    float Di;
    float t;
    float Vi;
    float Ar;
    float Vr;
    float Dr;
    float Ta;
    float Tv;
    float Td;
    float Tf;
    float Daccel;
    uint8_t state;
    uint8_t time_stamp;


}trajectory_t;

trajectory_t joint1,joint2;

typedef struct{
	float x_error;
	float y_error;
	float Vpx;
	float Vpy;
	trajectory_t xtraj;
	trajectory_t ytraj;
	float Vx,Vy;
	uint8_t state;
}robottraj_t;

PID_t pid_x,pid_y;

enum{
	IdLE,
	CLOSED_LOOP,
	DONE
};


void robot_trajint(float sat,float Ke,float Ku,float Kp,float Ki,float Kd,float Kn ,robottraj_t *R);
void robot_trajstart(float point[2][5],float xpos,float ypos,robottraj_t *R);
void robot_trajeval(float xpos,float ypos,robottraj_t *R);
void trajint(float Df,float Di,float Vf,float Vi,float accel,float deccel,trajectory_t *traj);
void trajeval(trajectory_t *traj);
//void navitrajeval(float pos,trajectory_t *traj);


#endif /* SRC_TRAJECTORY_CONTROL_TRAJ_H_ */
