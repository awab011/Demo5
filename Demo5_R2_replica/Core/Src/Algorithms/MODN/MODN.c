/*********************************************/
/*          Include Header                   */
/*********************************************/


/* Standard C header */
#include <math.h>

/* ROBOCON Library */

/* Internal Header */
#include "MODN.h"

/*********************************************/

/*********************************************/
/*          Variable                         */
/*********************************************/


/*********************************************/

/********************************************/

MODN_t modn;
MODN_t modn_ext;


/*---------------------------------------------*/
/*           Subroutine Function             */
/*********************************************/
void MODNRobotBaseVelInit(RobotBaseType_t base, float width, float length, MODN_t *modn)
{
	modn->base = base;
	modn->width = width;
	modn->length = length;
}


void MODNRobotConInit(float *x_dir, float *y_dir, float *w_dir, MODN_t *modn)
{
	modn->x_dir	= x_dir;
	modn->y_dir	= y_dir;
	modn->w_dir	= w_dir;
}

void MODNWheelVelInit(float *vel1, float *vel2, float *vel3, float *vel4, MODN_t *modn)
{
	modn->vel1	= vel1;
	modn->vel2	= vel2;
	modn->vel3	= vel3;
	modn->vel4	= vel4;
}

void MODNWheelDirInit(float *dir1, float *dir2, float *dir3, float *dir4, MODN_t *modn)
{
	modn->dir1	= dir1;
	modn->dir2	= dir2;
	modn->dir3	= dir3;
	modn->dir4	= dir4;
}

void MODNUpdate(MODN_t *modn)
{
	switch (modn->base){
	case MODN_FWD_OMNI:
		*(modn->vel1) = *(modn->x_dir)*(0.70711)  +  *(modn->y_dir)*(0.70711)  + *(modn->w_dir);
		*(modn->vel2) = *(modn->x_dir)*(-0.70711) +  *(modn->y_dir)*(0.70711)  - *(modn->w_dir);
		*(modn->vel3) = *(modn->x_dir)*(-0.70711)  +  *(modn->y_dir)*(0.70711)  + *(modn->w_dir);
		*(modn->vel4) = *(modn->x_dir)*(0.70711) +  *(modn->y_dir)*(0.70711)  - *(modn->w_dir);
		break;

	case MODN_MECANUM:
		*(modn->vel1) = *(modn->y_dir)*(1.0) + *(modn->x_dir)*(1.0)  - *(modn->w_dir);
		*(modn->vel2) = *(modn->y_dir)*(1.0) + *(modn->x_dir)*(-1.0) - *(modn->w_dir);
		*(modn->vel3) = *(modn->y_dir)*(1.0) + *(modn->x_dir)*(-1.0)  + *(modn->w_dir);
		*(modn->vel4) = *(modn->y_dir)*(1.0) + *(modn->x_dir)*(1.0) + *(modn->w_dir);

		break;

	case MODN_TRI_OMNI:
		*(modn->vel1) =   *(modn->x_dir)*(1.0) + *(modn->w_dir);
		*(modn->vel3) = *(modn->y_dir)*(0.866) +  *(modn->x_dir)*(-0.5) + *(modn->w_dir);
		*(modn->vel2) = *(modn->y_dir)*(0.866) +  *(modn->x_dir)*(0.5)  + *(modn->w_dir)*(-1.0);

		break;

	case MODN_FWD_SWERVE:
		modn->omega = *(modn->w_dir);
		modn->vtx = *(modn->x_dir);
		modn->vty = *(modn->y_dir);

		modn->a = modn->vtx + modn->omega * (modn->length/2);
		modn->b = modn->vtx - modn->omega * (modn->length/2);
		modn->c = modn->vty + modn->omega * (modn->width/2);
		modn->d = modn->vty - modn->omega * (modn->width/2);

		*(modn->vel1) = sqrt(SQ(modn->a)+SQ(modn->c));
		*(modn->vel2) = sqrt(SQ(modn->a)+SQ(modn->d));
		*(modn->vel3) = sqrt(SQ(modn->b)+SQ(modn->c));
		*(modn->vel4) = sqrt(SQ(modn->b)+SQ(modn->d));

		*(modn->dir1) = atan2f((modn->a),(modn->d))*(180/M_PI);
		*(modn->dir2) = atan2f((modn->a),(modn->c))*(180/M_PI);
		*(modn->dir3) = atan2f((modn->b),(modn->d))*(180/M_PI);
		*(modn->dir4) = atan2f((modn->b),(modn->c))*(180/M_PI);

		break;

	case MODN_TRI_SWERVE:

		/*
		 * https://www.desmos.com/calculator/gxezcefuqj
		 *
		 * 		     /\
		 * 			 ||
		 * 			 ||
		 * 	   |---width----|
		 * 	  A ____________ B__
		 * 		\          /	|
		 *  	 \        /		|
		 *		  \      /	  length
		 * 		   \    /		|
		 * 		    \  /		|
		 * 		     \/		  __|
		 * 			 C
		 *
		 */

		modn->omega = *(modn->w_dir);
		modn->vtx   = *(modn->x_dir);
		modn->vty   = *(modn->y_dir);

		modn->m = modn->width / (2 * COS_30_DEG);
		modn->n = modn->length / (1 + SIN_30_DEG);

		modn->a = modn->vtx + modn->omega * modn->n * SIN_30_DEG; // ax, bx //++--
		modn->b = modn->vty - modn->omega * modn->m * COS_30_DEG; // ay
		modn->c = modn->vty + modn->omega * modn->m * COS_30_DEG; // by
		modn->d = modn->vtx - modn->omega * modn->n; // SIN 90 // cx

		*(modn->vel1) = sqrt(SQ(modn->a)+SQ(modn->b));
		*(modn->vel2) = sqrt(SQ(modn->a)+SQ(modn->c));
		*(modn->vel3) = sqrt(SQ(modn->d)+SQ(modn->vty));

		*(modn->dir1) = atan2f((modn->a),(modn->b))*(180/M_PI);
		*(modn->dir2) = atan2f((modn->a),(modn->c))*(180/M_PI);
		*(modn->dir3) = atan2f((modn->d),(modn->vty))*(180/M_PI);

		break;
	}
}
