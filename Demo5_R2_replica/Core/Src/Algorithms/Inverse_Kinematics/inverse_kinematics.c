/*
 *  inverse_kinematics.c
 *
 *  Created on: 16/9/2025
 *      Author: Jin Ye Leong
 */


#include "inverse_kinematics.h"



#define IK_EPS 1e-6f


/*
 * Function Name		: ik2_analytic
 * Function Description : Analytic IK for 2Dof planar robotics arm.
 * Function Remarks		: Will give two solutions, elbow-up and elbow-down.
 * Function Arguments	: l1			,	length of joint1 to joint2 (meter)
 * 						  l2			,	length of joint2 to end-effector (meter)
 * 						  x				,	target end-effector x-position (meter)
 * 						  y				,	target end-effector y-position (meter)
 * 						  out_q			,	joint angles (J1 from world horizontal plane, J2 from horizontal plane extended from J1)
 * 						  					(solution[0] is elbow-down, solution[1] is elbow-up)
 * Function Return		: num of solutions
 * Function Example		: ik2_analytic(l1, l2, x, y, solutions);
 *
 * IMPROVEMENTS: 	1. Tweak solutions tolerance
 * 					2. Add a struct for the length configuration, make it structured
 * 					3. Clamping
 */
uint8_t ik2_analytic(float l1, float l2, float x, float y, float out_q[2][2]) {
	float r2 = x*x + y*y;
	float c2 = (r2 - l1*l1 - l2*l2) / (2.0f * l1 * l2);
	if (c2 > 1.0f + 1e-6f || c2 < -1.0f - 1e-6f) return 0; // unreachable
	if (c2 > 1.0f) c2 = 1.0f;
	if (c2 < -1.0f) c2 = -1.0f;
	float s2_pos = sqrtf(fmaxf(0.0f, 1.0f - c2*c2));
	float theta2_1 = atan2f(s2_pos, c2); // Elbow-Down
	float theta2_2 = atan2f(-s2_pos, c2); // Elbow-Up


	float k1 = l1 + l2 * c2;
	float k2_1 = l2 * s2_pos;
	float k2_2 = -k2_1;


	float theta1_1 = atan2f(y, x) - atan2f(k2_1, k1);
	float theta1_2 = atan2f(y, x) - atan2f(k2_2, k1);


	out_q[0][0] = theta1_1; out_q[0][1] = theta2_1;
	out_q[1][0] = theta1_2; out_q[1][1] = theta2_2;


	if (fabsf(s2_pos) < 1e-9f) return 1; // one solution (elbow straight)
	return 2;
}



/*
 * Function Name		: mat_mult_vec
 * Function Description : Multiply m×m matrix by m-vector: y = A*x
 */
void mat_mult_vec(const float* A, const float* x, float* y, int m) {
    for (int i = 0; i < m; ++i) {
        float s = 0.0f;
        for (int j = 0; j < m; ++j) s += A[i*m + j] * x[j];
        y[i] = s;
    }
}


/*
 * Function Name		: mat_add_diag
 * Function Description : Add λ² to the diagonal of an m×m matrix (λ²I)
 */
void mat_add_diag(float* A, int m, float lam2) {
    for (int i = 0; i < m; ++i) A[i*m + i] += lam2;
}


/*
 * Function Name		: mat_inverse
 * Function Description : Matrix inversion (Gauss-Jordan) for m×m
 */
bool mat_inverse(const float* A_in, float* Ainv_out, int m) {
    // local augmented array (keep size bounded by IK_MAX_TASKDIM)
    float aug[IK_MAX_TASKDIM][2 * IK_MAX_TASKDIM];
    // copy A into left, identity to right
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) aug[i][j] = A_in[i*m + j];
        for (int j = 0; j < m; ++j) aug[i][j + m] = (i == j) ? 1.0f : 0.0f;
    }

    // Gauss-Jordan
    for (int col = 0; col < m; ++col) {
        // Find pivot (simple pivoting: if near zero fail; could be improved with row swap)
        float pivot = aug[col][col];
        if (fabsf(pivot) < 1e-9f) {
            // try to swap with a lower row with non-zero pivot (improves robustness)
            int swap_row = -1;
            for (int r = col+1; r < m; ++r) if (fabsf(aug[r][col]) > 1e-9f) { swap_row = r; break; }
            if (swap_row < 0) return false; // singular
            // swap rows col and swap_row
            for (int c = 0; c < 2*m; ++c) {
                float tmp = aug[col][c]; aug[col][c] = aug[swap_row][c]; aug[swap_row][c] = tmp;
            }
            pivot = aug[col][col];
        }
        float inv_pivot = 1.0f / pivot;
        // normalize pivot row
        for (int j = 0; j < 2*m; ++j) aug[col][j] *= inv_pivot;
        // eliminate other rows
        for (int r = 0; r < m; ++r) {
            if (r == col) continue;
            float factor = aug[r][col];
            if (factor == 0.0f) continue;
            for (int j = 0; j < 2*m; ++j) aug[r][j] -= factor * aug[col][j];
        }
    }

    // extract right side as inverse
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) Ainv_out[i*m + j] = aug[i][j + m];
    }
    return true;
}


/*
 * Function Name		: numeric_jacobian
 * Function Description : J is m x n: J[row=r][col=j] = ∂(task_r) / ∂q_j
 */
void numeric_jacobian(const ik_joints_t* joints, fk_cb_t fk, void* userdata, int task_dim, float J[IK_MAX_TASKDIM][IK_MAX_JOINTS]) {
    ik_pose_t p0;
    fk(joints, &p0, userdata);
    const float dq = 1e-6f;
    for (uint8_t j = 0; j < joints->n; ++j) {
        ik_joints_t tmp = *joints;
        tmp.q[j] += dq;
        ik_pose_t p;
        fk(&tmp, &p, userdata);
        for (int r = 0; r < task_dim; ++r) {
            J[r][j] = (p.pos[r] - p0.pos[r]) / dq;
        }
    }
}



/*
 * Function Name		: ik_dls
 * Function Description : Jacobian + Damped Least Square algorithm used for 3Dof - 6Dof robotics arm.
 * Function Remarks		: Jacobian see how minor adjustment in the joint affects the error, DLS to avoid singularity.
 * Function Arguments	: arm			,	Robotic arm object.
 * 						  fk			,	Forward Kinematics function called to check if the current calculated joint angles reached the target, using trigonometry.
 * 						  max_iters		,	Limit iterations.
 * 						  tol			,	Tolerance for the error, more than that continue calculate.
 * 						  lambda		,	Jacobian lambda.
 *
 * Function Return		: ikdls_done, outoflimit
 * Function Example		: ik_dls(&robotarm1, fk_myrobot, &robotarm1.arm_params, max_iters, tol, lambda);
 *
 * IMPROVEMENTS: 	1. If the joint is out of limit, can set closer guess (use initial guess func)
 * 					2. Not tested yet for 4Dof - 6Dof, need to set task_is_angle[4-6] = true
 * 					3. Less heavy calculation (Hard)
 * 				Done4. Target not reached although it is reachable, used steps to solved already (Solved most if the time)
 */
void ik_dls(RoboticArm_IK* arm, fk_cb_t fk, void* userdata, uint16_t max_iters, float tol, float lambda){
    if (arm->arm_joints.n < 1) { arm->ikdls_done = false; return; }
    int n = arm->arm_joints.n;
    int m = arm->task_dim;
    if (m < 1 || m > IK_MAX_TASKDIM) { arm->ikdls_done = false; return; }

    // current end-effector pose
    ik_pose_t cur_pose;
    if (!fk(&arm->arm_joints, &cur_pose, userdata)) { arm->ikdls_done = false; return; }

    // prepare per-dimension deltas and sub-goals
    const int steps = 20; // tune as needed
    float delta[IK_MAX_TASKDIM] = {0.0f};

    for (int d = 0; d < m; ++d) {
        float target = arm->arm_endpoint.pos[d];
        float start  = cur_pose.pos[d];
        if (arm->task_is_angle[d]) {
            // shortest angular difference (wrap)
            float diff = target - start;
            // wrap to [-pi,pi]
            diff = wrap_to_pi(diff);
            delta[d] = diff / steps;
        } else {
            delta[d] = (target - start) / steps;
        }
    }

    // march through sub-goals
    for (int s = 1; s <= steps; ++s) {
        float sub_goal[IK_MAX_TASKDIM];
        for (int d = 0; d < m; ++d) {
            sub_goal[d] = cur_pose.pos[d] + delta[d] * s;
            if (arm->task_is_angle[d]) {
                // keep angles in [-pi, pi]
            	sub_goal[d] = wrap_to_pi(sub_goal[d]);
            }
        }

        bool subgoal_reached = false;
        // iterative solver for this sub-goal
        for (uint16_t iter = 0; iter < max_iters; ++iter) {
            ik_pose_t cur;
            if (!fk(&arm->arm_joints, &cur, userdata)) { arm->ikdls_done = false; return; }

            // compute error vector (m)
            float err[IK_MAX_TASKDIM] = {0.0f};
            for (int d = 0; d < m; ++d) {
                float e = sub_goal[d] - cur.pos[d];
                if (arm->task_is_angle[d]) {
                    // wrap shortest path
                    e = wrap_to_pi(e);
                }
                err[d] = e;
            }

            // norm
            float err_norm = 0.0f;
            for (int d = 0; d < m; ++d) err_norm += err[d]*err[d];
            err_norm = sqrtf(err_norm);
            if (err_norm <= tol) {
                for (int i = 0; i < n; ++i) arm->arm_joints.q[i] = wrap_to_pi(arm->arm_joints.q[i]);
                if (joints_within_limits(arm)) {
                    subgoal_reached = true;
                    break;
                } else {
                    arm->outoflimit = true;
                    // do not accept this solution — try continue iterations (or reinit)
                    // here we continue; you can add reinitialization strategy if needed
                    continue;
                }
            }

            // compute numeric Jacobian J (m x n)
            float J[IK_MAX_TASKDIM][IK_MAX_JOINTS];
            numeric_jacobian(&arm->arm_joints, fk, userdata, m, J);

            // compute A = J * J^T (m x m)
            float A[IK_MAX_TASKDIM * IK_MAX_TASKDIM];
            for (int r = 0; r < m; ++r){
            	for (int c = 0; c < m; ++c) {
					float sum = 0.0f;
					for (int k = 0; k < n; ++k) sum += J[r][k] * J[c][k];
					A[r*m + c] = sum;
            	}
            }

            // damping
            float lam2 = lambda * lambda;
            mat_add_diag(A, m, lam2);

            // invert A
            float Ainv[IK_MAX_TASKDIM * IK_MAX_TASKDIM];
            if (!mat_inverse(A, Ainv, m)) {
                // singular-ish — cannot continue
                arm->ikdls_done = false;
                return;
            }

            // tmp = Ainv * err
            float tmp[IK_MAX_TASKDIM];
            mat_mult_vec(Ainv, err, tmp, m);

            // dq = J^T * tmp  (n-vector)
            float dq[IK_MAX_JOINTS];
            for (int j = 0; j < n; ++j) {
                float ssum = 0.0f;
                for (int r = 0; r < m; ++r) ssum += J[r][j] * tmp[r];
                dq[j] = ssum;
            }

            // apply update (optionally add step scaling)
            float step = 1.0f;
            for (int j = 0; j < n; ++j) arm->arm_joints.q[j] += step * dq[j];
        } // iter

        if (!subgoal_reached) {
            arm->ikdls_done = false;
            return; // fail early if a subgoal can't be reached
        }

//        osDelay(200);
    } // steps

    // all sub-goals reached
    arm->ikdls_done = true;
    arm->outoflimit = false;
}



/*
 * Function Name		: fk_myrobot
 * Function Description : Forward kinematics for the 3-DOF setup (base yaw + 2D planar)
 * Function Remarks		: 3Dof - 6Dof robotics arm can refer to this template, by just adding respective calculation for the joint
 */
bool fk_myrobot(const ik_joints_t* joints, ik_pose_t* pose, void* userdata) {
	if (joints->n < 3) return false;
	myrobot_params_t* p = (myrobot_params_t*) userdata;


	float q0 = joints->q[0]; // yaw
	float q1 = joints->q[1]; // shoulder
	float q2 = joints->q[2]; // elbow


	// planar reach in the local X-Z plane (before yaw):
	float r_x = p->J2Jlength[1] * cosf(q1) + p->J2Jlength[2] * cosf(q1 + q2); // forward distance
	float r_z = p->J2Jlength[1] * sinf(q1) + p->J2Jlength[2] * sinf(q1 + q2); // height


	// rotate the planar X by base yaw q0 into world X,Y
	pose->pos[0] = r_x * cosf(q0); // X
	pose->pos[1] = r_x * sinf(q0); // Y
	pose->pos[2] = r_z + p->J2Jlength[0]; // Z (add l1 as base height offset)
	return true;
}


/*
 * Function Name		: wrap_to_pi
 * Function Description : Normalize an angle to [-pi, +pi].
 */
float wrap_to_pi(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}


/*
 * Function Name		: joints_within_limits
 * Function Description : Checks if joint is within limit.
 */
bool joints_within_limits(RoboticArm_IK* arm) {
    for (int i = 0; i < arm->arm_joints.n; i++) {
        if (arm->arm_joints.q[i] < arm->arm_params.q_min[i] || arm->arm_joints.q[i] > arm->arm_params.q_max[i]) {
            return false;
        }
    }
    return true;
}


/*
 * Function Name		: IK_RobotArm_Init
 * Function Description : Initialize the IK solver.
 * Function Remarks		:
 * Function Arguments	: arm			,	Robotic arm object.
 * 						  task_dim		,	Task dimention (m) must be less than n_joints (n) to work properly, decides how many movements are involved.
 * 						  n_joints		,	Number of joints.
 * 						  ...			,	Joint-to-joint lengths from J0 to end effector
 *
 * Function Return		:
 * Function Example		: 	IK_RobotArm_Init(&robotarm1, 3, 3, 0.05, 0.325, 0.28);
 */
void IK_RobotArm_Init(RoboticArm_IK* arm, uint8_t task_dim, uint8_t n_joints, ...) {
    memset(arm, 0, sizeof(RoboticArm_IK));
    arm->arm_joints.n = n_joints;
    arm->task_dim = task_dim;

    va_list args;
	va_start(args, n_joints);

	for (int i = 0; i < n_joints; i++)
		arm->arm_params.J2Jlength[i] = (float)va_arg(args, double);
	va_end(args);

	arm->current_waypoint = 0;
    arm->waypoint_count = 0;
    arm->ikdls_done = false;
}


/*
 * Function Name		: IK_RobotArm_InitialGuess
 * Function Description : Initial guess of the joint angle so the IK solver can converge well on the first attempt.
 * Function Remarks		: Not necessary if it works fine. Even now with the step method, this is not necessary.
 * Function Arguments	: arm			,	Robotic arm object.
 * 						  n_joints		,	Number of joints.
 * 						  ...			,	Initial guess of the angle of each joint (radian).
 *
 * Function Return		:
 * Function Example		: IK_RobotArm_InitialGuess(&robotarm1, 3, -1.0, 1.0, -1.0);
 */
void IK_RobotArm_InitialGuess(RoboticArm_IK* arm, int n_joints, ...) {
    va_list args;
	va_start(args, n_joints);

	for (int i = 0; i < n_joints; i++)
	    arm->arm_joints.q[i] = (float)va_arg(args, double);
	va_end(args);
}


/*
 * Function Name		: joints_safetyangle
 * Function Description : Set the safety angle limit for each joint.
 * Function Remarks		:
 * Function Arguments	: arm			,	Robotic arm object.
 * 						  jointindex	,	0 for J0.
 * 						  anglemin		,	minimum angle (radian).
 * 						  anglemax		,	maximum angle (radian).
 *
 * Function Return		:
 * Function Example		: joints_safetyangle(&robotarm1, 0, -1.57, 1.57); // -90 - 90
 */
void joints_safetyangle(RoboticArm_IK* arm, int jointindex, float anglemin, float anglemax) {
    arm->arm_params.q_min[jointindex] = anglemin;
    arm->arm_params.q_max[jointindex] = anglemax;
}


/*
 * Function Name		: IK_AddWaypoint
 * Function Description : Add waypoint for point to point movements.
 * Function Remarks		:
 * Function Arguments	: arm			,	Robotic arm object.
 * 						  task_dim		,	m.
 * 						  ...			,	The end effector position and orientation.
 *
 * Function Return		:
 * Function Example		: IK_AddWaypoint(&robotarm1, 3, 0.35f, -0.25f, 0.30f);
 */
void IK_AddWaypoint(RoboticArm_IK* arm, int task_dim, ...) {
    if (arm->waypoint_count < MAX_WAYPOINTS) {

    	va_list args;
		va_start(args, task_dim);

		for (int i = 0; i < task_dim; i++)
			arm->waypoints[arm->waypoint_count].pos[i] = (float)va_arg(args, double);
		va_end(args);

        arm->waypoint_count++;
    }
}


/*
 * Function Name		: IK_NextWaypoint
 * Function Description : Call this function to move to next waypoint.
 * Function Remarks		:
 * Function Arguments	: arm			,	Robotic arm object.
 *
 * Function Example		: IK_NextWaypoint(&robotarm1);
 */
void IK_NextWaypoint(RoboticArm_IK* arm) {
    if (arm->waypoint_count > 0) {
        arm->current_waypoint = (arm->current_waypoint) % arm->waypoint_count; //Loop
        arm->arm_endpoint = arm->waypoints[arm->current_waypoint];
        arm->current_waypoint++;
        arm->ikdls_done = false;  // reset solver state
        arm->outoflimit = false;
    }
}
