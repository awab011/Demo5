/*
 *  inverse_kinematics.h
 *
 *  Created on: 16/9/2025
 *      Author: Jin Ye Leong
 */

#include "../../adapter.h"



#define IK_MAX_JOINTS 6
#define IK_MAX_TASKDIM 6   // support up to 6 task dims (x,y,z + yaw,pitch,roll)


#define MAX_WAYPOINTS 10


typedef struct {
	float q[IK_MAX_JOINTS]; // joint angles (radians)
	uint8_t n; // number of joints
} ik_joints_t;


typedef struct {
	float pos[IK_MAX_TASKDIM]; // convention: [x,y,z, yaw, pitch, roll] (meters)
} ik_pose_t;


typedef struct {
    float J2Jlength[IK_MAX_JOINTS]; // Joint-to-joint length

    float p2ptime; //point-to-point time

    float q_orient_unit[IK_MAX_JOINTS]; //joint orientation & gear ratio & unit conversion

    float q_min[IK_MAX_JOINTS]; // lower joint limits (rad)
	float q_max[IK_MAX_JOINTS]; // upper joint limits (rad)
} myrobot_params_t;




typedef bool (*fk_cb_t)(const ik_joints_t* joints, ik_pose_t* pose, void* userdata);



typedef struct {
	ik_joints_t arm_joints;
	ik_pose_t arm_endpoint;
	myrobot_params_t arm_params;

	bool ikdls_done;
	bool outoflimit;

	int task_dim;                          // m: how many task components are active (3..6)
	bool task_is_angle[IK_MAX_TASKDIM];    // mark which task components are angular (wrap)

	int waypoint_count;
	int current_waypoint;
	ik_pose_t waypoints[MAX_WAYPOINTS];
} RoboticArm_IK;


//ik_joints_t robot_joints;
//ik_pose_t robot_endpoint;
//myrobot_params_t robot_params;

RoboticArm_IK robotarm1;


/***prototype function***/
uint8_t ik2_analytic(float l1, float l2, float x, float y, float out_q[2][2]);

void mat_mult_vec(const float* A, const float* x, float* y, int m);
void mat_add_diag(float* A, int m, float lam2);
bool mat_inverse(const float* A_in, float* Ainv_out, int m);
void numeric_jacobian(const ik_joints_t* joints, fk_cb_t fk, void* userdata, int task_dim, float J[IK_MAX_TASKDIM][IK_MAX_JOINTS]);
void ik_dls(RoboticArm_IK* arm, fk_cb_t fk, void* userdata, uint16_t max_iters, float tol, float lambda);
float wrap_to_pi(float angle);
bool joints_within_limits(RoboticArm_IK* arm);

void IK_RobotArm_Init(RoboticArm_IK* arm, uint8_t task_dim, uint8_t n_joints, ...);
void IK_RobotArm_InitialGuess(RoboticArm_IK* arm, int n_joints, ...);
void joints_safetyangle(RoboticArm_IK* arm, int jointindex, float anglemin, float anglemax);


/*** waypoint manager ***/
void IK_AddWaypoint(RoboticArm_IK* arm, int task_dim, ...);
void IK_NextWaypoint(RoboticArm_IK* arm);


bool fk_myrobot(const ik_joints_t* joints, ik_pose_t* pose, void* userdata);
/* SRC_INVERSE_KINEMATICS_INVERSE_KINEMATICS_H_ */
