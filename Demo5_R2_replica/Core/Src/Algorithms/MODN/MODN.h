#ifndef MODN_H
#define MODN_H

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "../../adapter.h"
/*********************************************/

/*********************************************/
/*          Define                           */
/*********************************************/
#define SQ(x)	((x)*(x))

/*********************************************/
/*          Enumarator                       */
/*********************************************/
typedef enum{
	MODN_FWD_OMNI = 0,
	MODN_MECANUM,
	MODN_TRI_OMNI,
	MODN_FWD_SWERVE,
	MODN_TRI_SWERVE,
} RobotBaseType_t;


/*********************************************/
/*          Variable                         */
/*********************************************/
typedef struct {
	float *x_dir;
	float *y_dir;
	float *w_dir;

	float *vel1;	float *vel2;	float *vel3;	float *vel4;
	float *dir1;	float *dir2;	float *dir3;	float *dir4;

	RobotBaseType_t base;
	float width;
	float length;
	float m, n;

	// for swerve
	float omega;
	float vtx;
	float vty;
	float a;	float b;	float c;	float d;
	float asign;	float bsign;	float csign;	float dsign;

}MODN_t;

extern MODN_t modn;
extern MODN_t modn_ext;
/*********************************************/
/*           Function Prototype              */
/*********************************************/
void MODNRobotBaseVelInit(RobotBaseType_t base, float width, float length, MODN_t *modn);
void MODNRobotConInit(float *x_dir, float *y_dir, float *w_dir, MODN_t *modn);
void MODNWheelVelInit(float *vel1, float *vel2, float *vel3, float *vel4, MODN_t *modn);
void MODNWheelDirInit(float *dir1, float *dir2, float *dir3, float *dir4, MODN_t *modn);
void MODNUpdate(MODN_t *modn);


#endif
