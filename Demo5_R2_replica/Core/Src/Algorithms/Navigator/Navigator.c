/*
 * Navigator.c
 *
 *  Created on: Nov 25, 2025
 *      Author: Ibrahim Meselhy
 */
#include "Navigator.h"

float Normalize_Angle(float angle) {
	while (angle > 180) angle -= 2.0f * 180.0;
	while (angle < -180) angle += 2.0f * 180;
	return angle;
}

/*
 * Function: Shortest_Angular_Distance
 * Description: Calculate shortest angular distance from current to target
 */
float Shortest_Angular_Distance(float from, float to) {
	float diff = to - from;
	return Normalize_Angle(diff);
}

/*
 * Function: Navigator_Init
 * Description: Initialize navigation system with PathProfile + PID
 */
void Navigator_Init(Navigator_t *nav,float* current_x,float* current_y,float* current_yaw) {
	// Initialize PathProfiles
	Path_Profile_Init(&nav->prof_x);
	Path_Profile_Init(&nav->prof_y);
	Path_Profile_Init(&nav->prof_yaw);
	nav->prof_x.start_t = 1.0;
	nav->prof_x.end_t = 1.21;
	nav->prof_x.k_val = 1.47;

	nav->prof_y.start_t = 1.0;
	nav->prof_y.end_t = 1.21;
	nav->prof_y.k_val = 1.47;
	// Initialize PID sources (connect error input and velocity output)
	PIDSourceInit(&nav->error_x, &nav->x_vel, &nav->pid_x);
	PIDSourceInit(&nav->error_y, &nav->y_vel, &nav->pid_y);
	PIDSourceInit(&nav->error_yaw, &nav->w_vel, &nav->pid_yaw);

	// Initialize PID gains for X and Y
	// Arguments: ts, sat, ke, ku, kp, ki, kd, kn, pid
	PIDGainInit(0.005f,  // ts: 5ms sampling time
			1.0f,    // sat: normalized saturation
			1.0,    // ke: error gain
			2.5f,  // ku: max velocity (m/s)
			1.5f,    // kp: proportional gain
			0.005f,    // ki: integral gain
			0.079f,    // kd: derivative gain
			50.0f,  // kn: derivative filter
			&nav->pid_x);

	PIDGainInit(0.005f,  // ts: 5ms sampling time
			1.0f,    // sat: normalized saturation
			1.0,    // ke: error gain
			2.5f,  // ku: max velocity (m/s)
			1.5f,    // kp: proportional gain
			0.005f,    // ki: integral gain
			0.079f,    // kd: derivative gain
			50.0f,  // kn: derivative filter
			&nav->pid_y);

	// Initialize PID gains for Yaw (angular velocity)
	PIDGainInit(0.005f, // ts: 5ms sampling time
			1.0,   // Saturation Value
			1.0/180.0, // Error Gain
			1.0,	// Output scaling gain
			4.0, // Proportional gain
			0.0, // Integral gain
			1.5, // Derivative gain
			30.0, // Filter for derivative
			&nav->pid_yaw);

	// Initialize PID delays
	PIDDelayInit(&nav->pid_x);
	PIDDelayInit(&nav->pid_y);
	PIDDelayInit(&nav->pid_yaw);

	// Set initial state
	nav->mode = NAV_MODE_PATHPROFILE;  // Default to PathProfile mode
	nav->is_moving = 0;
	nav->pos_locked =0;
	nav->tolerance_pos = 0.005f;      // 5mm position tolerance
	nav->tolerance_yaw = 1.0f;     // ~1.0 degrees yaw tolerance
	nav->face_direction = 0;        // Don't auto-face direction by default
	nav->current_x = current_x ;
	nav->current_y = current_y;
	nav->current_yaw = current_yaw;
	nav->max_velocity = 1.0f;      // Maximum safe speed
	nav->accel_dist = 0.4f;         // Acceleration distance
	nav->decel_dist = 0.2;         // Deceleration distance
	nav->max_angular_vel = 2.5f;     // rad/s - maximum rotation speed
}

/*
 * Function: Navigator_MoveTo
 * Description: Start smooth motion to target position
 * If target_yaw is NAN, robot will maintain current heading (no rotation)
 *
 * IMPORTANT: Use NAN (not 0.0) if you don't want yaw control!
 *   Correct:   Navigator_MoveTo(&nav, 1.0, 1.0, NAN);     // No yaw change
 *   Incorrect: Navigator_MoveTo(&nav, 1.0, 1.0, 0.0);     // Will try to rotate to 0°
 */
void Navigator_MoveTo(Navigator_t *nav, float target_x, float target_y, float target_yaw) {
	// Calculate distances
	float dx = target_x - *nav->current_x;
	float dy = target_y - *nav->current_y;
	float total_dist = sqrtf(dx * dx + dy * dy);

	// Check if already at target position
	if (total_dist < nav->tolerance_pos) {
		// Only handle yaw rotation
		if (!isnan(target_yaw)) {
			float yaw_dist = Shortest_Angular_Distance(*nav->current_yaw, target_yaw);
			if (fabsf(yaw_dist) > nav->tolerance_yaw) {
				// Pure rotation
				Path_Profile_Set(&nav->prof_yaw,
						*nav->current_yaw,
						0.4f,           // Small accel for rotation
						yaw_dist,
						0.4f,           // Small decel
						2.5f);          // Angular velocity (rad/s)
				nav->target_yaw = target_yaw;
				nav->is_moving = 1;
				nav->completed = 0;
			}
		}
		return;
	}

	// Motion parameters - TUNE THESE & MATCH UNITS!
	nav->completed = 0;
	nav->pos_locked = 0;
	// Handle short distances (not enough room for full accel/decel)
	float accel_dist = nav->accel_dist;
	float decel_dist = nav->decel_dist;
	if (total_dist < nav->accel_dist + nav->decel_dist) {
		float scale = total_dist / (nav->accel_dist + nav->decel_dist) * 0.8f;
		accel_dist *= scale;
		decel_dist *= scale;
	}

	// Calculate proportional distances for each axis
	float x_ratio = fabsf(dx) / total_dist;
	float y_ratio = fabsf(dy) / total_dist;

	// Set PathProfile for X-axis
	Path_Profile_Set(&nav->prof_x,
			*nav->current_x,           // offset: current position
			accel_dist * x_ratio,     // d1: accel distance
			dx,                       // td: total distance (signed)
			decel_dist * x_ratio,     // d3: decel distance
			nav->max_velocity);            // v: velocity

	// Set PathProfile for Y-axis
	Path_Profile_Set(&nav->prof_y,
			*nav->current_y,
			accel_dist * y_ratio,
			dy,
			decel_dist * y_ratio,
			nav->max_velocity);

	// Handle yaw (heading) control
	uint8_t control_yaw = 0;  // Flag to track if we're controlling yaw

	if (nav->face_direction) {
		// Auto-calculate target yaw to face direction of travel
		target_yaw = atan2f(dy, dx);
		control_yaw = 1;
	}
	else if (!isnan(target_yaw)) {
		// User provided explicit yaw target
		control_yaw = 1;
	}

	if (control_yaw) {
		// Calculate shortest angular path
		float yaw_dist = Shortest_Angular_Distance(*nav->current_yaw, target_yaw);

		// Only set yaw profile if rotation is significant
		if (fabsf(yaw_dist) > nav->tolerance_yaw) {
			// Set PathProfile for yaw
			Path_Profile_Set(&nav->prof_yaw,
					*nav->current_yaw,
					1.0f,              // Small accel distance (radians)
					yaw_dist,          // Angular distance (radians)
					1.0f,              // Small decel distance
					nav->max_angular_vel);  // Angular velocity

			if (nav->prof_yaw.error) {
				// Error in yaw profile, abort
				nav->is_moving = 0;
				return;
			}
		} else {
			// Already at target heading, no rotation needed
			Path_Profile_Reset(&nav->prof_yaw);
			target_yaw = NAN;  // Mark as no yaw control needed
		}
	} else {
		// No yaw control requested
		Path_Profile_Reset(&nav->prof_yaw);
		target_yaw = NAN;  // Ensure its marked as no control
	}

	// Save target (NAN if no yaw control)
	nav->target_yaw = target_yaw;

	// Check for errors
	if (nav->prof_x.error || nav->prof_y.error) {
		// Handle error (e.g., invalid parameters)
		nav->is_moving = 0;
		return;
	}

	// Reset PID controllers
	PIDDelayInit(&nav->pid_x);
	PIDDelayInit(&nav->pid_y);
	PIDDelayInit(&nav->pid_yaw);

	// Save targets
	nav->target_x = target_x;
	nav->target_y = target_y;
	// nav->target_yaw already set above (might be NAN)
	nav->mode = NAV_MODE_PATHPROFILE;  // Default to PathProfile mode
	nav->is_moving = 1;
}
/*
 * Function: Navigator_Update
 * Description: Call this in your main loop (or in a 5ms loop, or any loop)
 * Updates PathProfile trajectories and PID tracking for X, Y, and Yaw
 */
void Navigator_Update(Navigator_t *nav) {
	if (!nav->is_moving) {
		nav->x_vel = 0.0f;
		nav->y_vel = 0.0f;
		nav->w_vel = 0.0f;
		return;
	}
	if (nav->mode == NAV_MODE_PURE_PID) {
		// ========================================
		// PURE PID MODE (Position Lock)
		// ========================================
		// Direct PID control from current to target
		// No PathProfile, just maintain position

		// Calculate errors directly to target
		nav->error_x = nav->target_x - *nav->current_x;
		nav->error_y = nav->target_y - *nav->current_y;
		//		nav->error_yaw = Shortest_Angular_Distance(*nav->current_yaw, nav->target_yaw);
		nav->error_yaw = 0.0;

		// Run PID controllers
		PID(&nav->pid_x);    // Outputs to nav->velocity_x
		PID(&nav->pid_y);    // Outputs to nav->velocity_y
		//		PID(&nav->pid_yaw);  // Outputs to nav->w_vel
		if(fabsf(nav->error_x) < 0.005)
		{
			nav->x_vel = 0;
		}
		if(fabsf(nav->error_y) < 0.005)
		{
			nav->y_vel = 0;
		}
		nav->w_vel = 0.0;
		// Check if within tolerance (optional: can stay in this mode indefinitely)
		float dist_to_target = sqrtf(nav->error_x * nav->error_x + nav->error_y * nav->error_y);
#define NAV_MIN_VEL  0.08f   // minimum speed that actually moves motors


		if (dist_to_target > nav->tolerance_pos) {
			// Direction unit vector toward final target (not profile reference)
			float inv_dist = 1.0f / dist_to_target;
			float dir_x = (nav->target_x - *nav->current_x) * inv_dist;
			float dir_y = (nav->target_y - *nav->current_y) * inv_dist;

			// If the PID output in this axis is smaller than deadband, inject minimum
			if (fabsf(nav->x_vel) < NAV_MIN_VEL && fabsf(dir_x) > 0.01f)
				nav->x_vel = dir_x * NAV_MIN_VEL;
			if (fabsf(nav->y_vel) < NAV_MIN_VEL && fabsf(dir_y) > 0.01f)
				nav->y_vel = dir_y * NAV_MIN_VEL;
		}
		if (dist_to_target < nav->tolerance_pos &&
				fabsf(nav->error_yaw) < nav->tolerance_yaw && (fabs(nav->x_vel) <= 0.15f) && (fabs(nav->y_vel) <= 0.15f)) {
				nav->completed = 1;
				nav->is_moving = 0;
				nav->x_vel = 0.0f;
				nav->y_vel = 0.0f;
				nav->w_vel = 0.0f;
			//			nav->pos_locked = 1;
			// (Don't set is_moving = 0, keep actively holding)
		}
	}
	else{
		// Update PathProfile trajectories (reference paths)
		Path_Profile_Update(&nav->prof_x);
		Path_Profile_Update(&nav->prof_y);

		// Calculate tracking errors (reference - actual)
		nav->error_x = nav->prof_x.path - *nav->current_x;
		nav->error_y = nav->prof_y.path - *nav->current_y;

		// Calculate yaw error (handle angle wrapping)

		uint8_t yaw_done = 1;  // Assume yaw is done (default for no yaw control)

		if (!isnan(nav->target_yaw)) {
			Path_Profile_Update(&nav->prof_yaw);
			float yaw_error = Shortest_Angular_Distance(*nav->current_yaw, nav->prof_yaw.path);
			nav->error_yaw = yaw_error;
			PID(&nav->pid_yaw);  // Outputs to nav->w_vel
			// We ARE controlling yaw, so check if it's complete
			float yaw_error_final = Shortest_Angular_Distance(*nav->current_yaw, nav->target_yaw);
			yaw_done = (nav->prof_yaw.state == 0 &&
					fabsf(yaw_error_final) < nav->tolerance_yaw);
		}
		// Run PID controllers to generate velocity commands
		PID(&nav->pid_x);    // Outputs to nav->velocity_x
		PID(&nav->pid_y);    // Outputs to nav->velocity_y

#define NAV_MIN_VEL  0.1f   // minimum speed that actually moves motors

		float dist_remaining = sqrtf(
				(nav->target_x - *nav->current_x) * (nav->target_x - *nav->current_x) +
				(nav->target_y - *nav->current_y) * (nav->target_y - *nav->current_y)
		);

		if (dist_remaining > nav->tolerance_pos) {
			// Direction unit vector toward final target (not profile reference)
			float inv_dist = 1.0f / dist_remaining;
			float dir_x = (nav->target_x - *nav->current_x) * inv_dist;
			float dir_y = (nav->target_y - *nav->current_y) * inv_dist;

			// If the PID output in this axis is smaller than deadband, inject minimum
			if (fabsf(nav->x_vel) < NAV_MIN_VEL && fabsf(dir_x) > 0.01f)
				nav->x_vel = dir_x * NAV_MIN_VEL;
			if (fabsf(nav->y_vel) < NAV_MIN_VEL && fabsf(dir_y) > 0.01f)
				nav->y_vel = dir_y * NAV_MIN_VEL;
		}
		float dist_to_target = dist_remaining;


		// Motion complete when EITHER:
		// a) Robot physically arrived within tolerance (primary check), OR
		// b) Profile finished AND robot is within a relaxed tolerance
		// This decouples the open-loop profile from the closed-loop physical position,
		// preventing a stall when the profile ends but the robot hasn't quite crossed
		// the tight threshold, or vice versa.
		uint8_t profile_done = (nav->prof_x.state == 0 && nav->prof_y.state == 0);
		uint8_t position_done = /*(dist_to_target < nav->tolerance_pos) || */
				(profile_done && dist_to_target < nav->tolerance_pos);


		// If target_yaw is NAN, yaw_done stays 1 (don't wait for yaw)
		if (position_done && yaw_done) {
			nav->completed = 1;
			nav->is_moving = 0;
			nav->x_vel = 0.0f;
			nav->y_vel = 0.0f;
			nav->w_vel = 0.0f;
		}
	}
}
/*
 * Function: Navigator_MoveToWithoutYaw
 * Description: Move to position without changing heading
 */
void Navigator_MoveToWithoutYaw(Navigator_t *nav, float target_x, float target_y) {
	Navigator_MoveTo(nav, target_x, target_y, NAN);
}

/*
 * Function: Navigator_RotateToHeading
 * Description: Pure rotation to target heading without translation
 */
void Navigator_RotateToHeading(Navigator_t *nav, float target_yaw) {
	float yaw_dist = Shortest_Angular_Distance(*nav->current_yaw, target_yaw);

	if (fabsf(yaw_dist) < nav->tolerance_yaw) {
		return;  // Already at target heading
	}

	// Reset position profiles
	Path_Profile_Reset(&nav->prof_x);
	Path_Profile_Reset(&nav->prof_y);

	// Set yaw profile
	Path_Profile_Set(&nav->prof_yaw,
			*nav->current_yaw,
			0.2f,
			yaw_dist,
			0.2f,
			1.5f);  // Max angular velocity

	if (nav->prof_yaw.error) {
		return;
	}

	PIDDelayInit(&nav->pid_yaw);
	nav->target_yaw = target_yaw;
	nav->is_moving = 1;
	nav->completed = 0;
}

/*
 * Function: Navigator_SetFaceDirection
 * Description: Enable/disable auto-facing direction of travel
 */
void Navigator_SetFaceDirection(Navigator_t *nav, uint8_t enable) {
	nav->face_direction = enable;
}
/*
 * Function: Navigator_LockPosition
 * Description: Lock current position using Pure PID control
 *
 * This mode continuously applies PID control to hold position
 * Useful for:
 * - Fighting disturbances (wind, collisions, slopes)
 * - Maintaining position while waiting
 * - Active braking
 *
 * Call Navigator_Stop() to release the lock
 */
void Navigator_LockPosition(Navigator_t *nav) {
	// Set target to current position
	nav->target_x = *nav->current_x;
	nav->target_y = *nav->current_y;
	nav->target_yaw = *nav->current_yaw;

	// Reset PathProfiles (not used in Pure PID mode)
	Path_Profile_Reset(&nav->prof_x);
	Path_Profile_Reset(&nav->prof_y);
	Path_Profile_Reset(&nav->prof_yaw);

	// Reset PID controllers for fresh start
	PIDDelayInit(&nav->pid_x);
	PIDDelayInit(&nav->pid_y);
	PIDDelayInit(&nav->pid_yaw);

	// Switch to Pure PID mode
	nav->mode = NAV_MODE_PURE_PID;
	nav->completed = 0;
	nav->pos_locked =0;
	nav->is_moving = 1;  // Keep active (holding position)
}

/*
 * Function: Navigator_LockPositionAt
 * Description: Lock at a specific position using Pure PID
 *
 * Immediately jumps to trying to hold target position
 * No smooth trajectory - will be jerky if far from target!
 *
 * Use cases:
 * - Quick corrections to target position
 * - Holding a specific waypoint against disturbances
 */
void Navigator_LockPositionAt(Navigator_t *nav, float target_x, float target_y, float target_yaw) {
	nav->x_vel = 0;
	nav->y_vel = 0;
	nav->w_vel = 0;

	nav->target_x = target_x;
	nav->target_y = target_y;
	nav->target_yaw = target_yaw;

	// Reset PathProfiles
	Path_Profile_Reset(&nav->prof_x);
	Path_Profile_Reset(&nav->prof_y);
	Path_Profile_Reset(&nav->prof_yaw);

	// Reset PID controllers
	PIDDelayInit(&nav->pid_x);
	PIDDelayInit(&nav->pid_y);
	PIDDelayInit(&nav->pid_yaw);

	// Switch to Pure PID mode
	nav->mode = NAV_MODE_PURE_PID;
	nav->completed = 0;
	nav->pos_locked =0;
	nav->is_moving = 1;
}
/*
 * Function: Navigator_Stop
 * Description: Emergency stop
 */
void Navigator_Stop(Navigator_t *nav) {
	nav->is_moving = 0;
	nav->completed = 1;
	//	nav->pos_locked = 1;
	nav->x_vel = 0.0f;
	nav->y_vel = 0.0f;
	nav->w_vel = 0.0f;
	Path_Profile_Reset(&nav->prof_x);
	Path_Profile_Reset(&nav->prof_y);
	Path_Profile_Reset(&nav->prof_yaw);
	PIDDelayInit(&nav->pid_x);
	PIDDelayInit(&nav->pid_y);
	PIDDelayInit(&nav->pid_yaw);
}

/*
 * Function: Navigator_IsMoving
 * Description: Check if navigation is in progress
 */
uint8_t Navigator_IsMoving(Navigator_t *nav) {
	return nav->is_moving;
}

/*
 * Function: Navigator_GetVelocityX
 */
float Navigator_GetVelocityX(Navigator_t *nav) {
	return nav->x_vel;
}

/*
 * Function: Navigator_GetVelocityY
 */
float Navigator_GetVelocityY(Navigator_t *nav) {
	return nav->y_vel;
}

/*
 * Function: Navigator_GetAngularVelocity
 */
float Navigator_GetAngularVelocity(Navigator_t *nav) {
	return nav->w_vel;
}

/*
 * Function: Navigator_TunePID_Position
 * Description: Tune PID gains for X and Y position control
 */
void Navigator_TunePID_Position(Navigator_t *nav, float kp, float ki, float kd) {
	PIDGainSet(KP, kp, &nav->pid_x);
	PIDGainSet(KI, ki, &nav->pid_x);
	PIDGainSet(KD, kd, &nav->pid_x);

	PIDGainSet(KP, kp, &nav->pid_y);
	PIDGainSet(KI, ki, &nav->pid_y);
	PIDGainSet(KD, kd, &nav->pid_y);
}

/*
 * Function: Navigator_TunePID_Yaw
 * Description: Tune PID gains for yaw control
 */
void Navigator_TunePID_Yaw(Navigator_t *nav, float kp, float ki, float kd) {
	PIDGainSet(KP, kp, &nav->pid_yaw);
	PIDGainSet(KI, ki, &nav->pid_yaw);
	PIDGainSet(KD, kd, &nav->pid_yaw);
}
/*
 * Function: Nav_SetMotionParams
 * Description: Adjust motion parameters (velocity, accel/decel distances)
 */
void Nav_SetMotionParams(Navigator_t *nav, float velocity, float accel_dist, float decel_dist) {
	nav->max_velocity = velocity;
	nav->accel_dist = accel_dist;
	nav->decel_dist = decel_dist;
}
void Nav_SetPosTol(Navigator_t *nav, float tol)
{
	nav->tolerance_pos = tol;
}
