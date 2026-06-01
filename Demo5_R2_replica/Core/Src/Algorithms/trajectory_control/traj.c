/*
 * traj.c
 *
 *  Created on: May 14, 2024
 *      Author: ahmed amir
 */
#include "traj.h"



void robot_trajint(float sat,float Ke,float Ku,float Kp,float Ki,float Kd,float Kn ,robottraj_t *R){


	PIDSourceInit(&R->x_error,&R->Vpx,&pid_x);
	PIDSourceInit(&R->y_error,&R->Vpy,&pid_y);
//	PIDGainInit(0.005, 1.0, 0.06, 3.0, 20.5,0.0,0.8,0.0,&pid_x);
//	PIDGainInit(0.005, 1.0, 0.06, 3.0, 20.5,0.0,0.8,0.0,&pid_y);
	PIDGainInit(0.005,sat,Ke,Ku,Kp,Ki,Kd,Kn,&pid_x);
	PIDGainInit(0.005,sat,Ke,Ku,Kp,Ki,Kd,Kn,&pid_y);

}

void robot_trajstart(float point[2][5],float xpos,float ypos,robottraj_t *R){
   if(R->state!=CLOSED_LOOP){
	if (point[0][0])
	trajint(point[0][1],xpos,point[0][2],0,point[0][3],point[0][4], &R->xtraj);

	if (point[1][0])
	trajint(point[1][1],ypos,point[1][2],0,point[1][3],point[1][4], &R->ytraj);
	R->state=CLOSED_LOOP;
   }
}



void robot_trajeval(float xpos,float ypos,robottraj_t *R){
	if(R->state==CLOSED_LOOP){
	trajeval(&R->xtraj);
	trajeval(&R->ytraj);
	if((R->xtraj.state==DONE)&&(R->ytraj.state==DONE))
		R->state=DONE;
	R->x_error=R->xtraj.Y-xpos;
	R->y_error=R->ytraj.Y-ypos;
	PID(&pid_x);
	PID(&pid_y);
	R->Vx=(R->xtraj.state==DONE)? 0:R->xtraj.Yd+R->Vpx;
	R->Vy=(R->ytraj.state==DONE)? 0:R->ytraj.Yd+R->Vpy;
	}

}

void trajint(float Df,float Di,float Vf,float Vi,float accel,float deccel,trajectory_t *traj){
//	float Df=sqrt(Xf*Xf+Yf*Yf);
//	traj->angle=atanf(Yf/Xf);
	if(traj->state==IdLE){
	float dX = Df - Di;
	float stop_dist = (Vi * Vi) / (2.0f * deccel);
	float dXstop = copysign(stop_dist, Vi);
	float s = (dX - dXstop)>0 ? 1:-1;
	traj->Ar= s * accel;
	traj->Dr = -s * deccel;
	traj->Vr= s * Vf;

	if ((s * Vi) > (s * traj->Vr)) {
		traj->Ar  = -s * accel;
	}

	traj->Ta = (traj->Vr- Vi) / traj->Ar;
	traj->Td = -traj->Vr/ traj->Dr;

	float dXmin = 0.5f*traj->Ta *(traj->Vr + Vi) + 0.5f*traj->Td*traj->Vr;
	if (s*dX < s*dXmin) {
		// Short move (triangle profile)
		traj->Vr = s *sqrt(fmax((traj->Dr*(Vi*Vi) + 2*traj->Ar*traj->Dr*dX) / (traj->Dr- traj->Ar), 0.0f));
		traj->Ta = fmax(0.0f, (traj->Vr - Vi) /traj->Ar);
		traj->Td= fmax(0.0f, -traj->Vr/ traj->Dr);
		traj->Tv= 0.0f;
	} else {
		// Long move (trapezoidal profile)
		traj->Tv = (dX - dXmin) / traj->Vr;
	}
	traj->Tf= traj->Ta+ traj->Td+ traj->Tv;
	traj->Di=Di;
	traj->Df = Df;
	traj->Vi = Vi;
	traj->Daccel= Di + Vi*traj->Ta + 0.5f*traj->Ar*traj->Ta*traj->Ta;
	traj->state=CLOSED_LOOP;
	}
}




void trajeval(trajectory_t *traj){
	if(traj->state==CLOSED_LOOP){
		traj->t+=0.005;
	  float	t=traj->t;


	 if (t < 0.0f) {  // Initial Condition
//	        traj->Y   = traj->Di;
 	        traj->Yd  = traj->Vi;
	        traj->Ydd = 0.0f;
	    } else if (t < traj->Ta) {  // Accelerating
	        traj->Y   = traj->Di + traj->Vi*t + 0.5f*traj->Ar*t*t;
	        traj->Yd  = traj->Vi + traj->Ar*t;
	        traj->Ydd = traj->Ar;
	    } else if (t < traj->Ta + traj->Tv) {  // Coasting
	        traj->Y   = traj->Daccel + traj->Vr*(t - traj->Ta);
	        traj->Yd  = traj->Vr;
	        traj->Ydd = 0.0f;
	    } else if (t < traj->Tf) {  // Deceleration
	        float td     = t - traj->Tf;
	        traj->Y   = traj->Df + 0.5f*traj->Dr*td*td;
	        traj->Yd  = traj->Dr*td;
	        traj->Ydd = traj->Dr;
	    } else if (t >= traj->Tf) {  // Final Condition
	        traj->Y   = traj->Df;
	        traj->Yd  = 0.0f;
	        traj->Ydd = 0.0f;
	        traj->set_pos=traj->Df;
	        traj->state =DONE;
	        traj->t=0;
	    } else {
	        // TODO: report error here
	    }
	}


}









//void navitrajeval(float pos,trajectory_t *traj){
//	if(traj->state==CLOSED_LOOP){
//		float D=pos-traj->Di;
//		float yaccel=traj->Di + traj->Vi*traj->Ta + 0.5f*traj->Ar*traj->Ta*traj->Ta;
//		float yv=traj->Daccel+traj->Vr*traj->Tv;
//
//
//
//		if (pos < yaccel) {  // Accelerating
//			traj->Yd  = sqrt(2*D*traj->Ar);
//		}
//
//		else if(pos<yv){  //coasting
//			traj->Yd=traj->Vr;
//		}
//
//		else if (pos<traj->Df){ //Decelerating
//
//			traj->Yd=sqrt(2*(pos-traj->Df)*traj->Dr);
//		}
//		else if (pos >= traj->Df) {  // Final Condition
//			traj->Yd  = 0.0f;
//		}
//	}
//}
