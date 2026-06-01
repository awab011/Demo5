/*
 * Extension_Leg.c
 *
 *  Created on: Nov 26, 2025
 *      Author: Ibrahim Meselhy
 */


/*
 * Rack & Pinion Ext Control using PathProfile
 *
 * PathProfile generates smooth position trajectories
 * Robomaster motor handles position control with internal PID
 *
 * Features:
 * - Move from any position to target (e.g., 2.0 → 3.0)
 * - Smooth acceleration/deceleration
 * - Can stop at any time
 * - Resume from current position
 */

#include "Extension_Leg.h"

ExtController_t front_ext;
ExtController_t rear_ext;
ExtController_t spear_arm;
ExtController_t spear_picker;
ExtController_t kfs_grip;
ExtController_t kfs_up;
ExtController_t kfs_yaw;
ExtController_t cubemars_ext;
/*
 * Function: Ext_Init
 * Description: Initialize Ext controller
 *
 * Call this ONCE during setup
 */
void Ext_Init(ExtController_t *Ext,float* current_pos) {
    // Initialize PathProfile
    Path_Profile_Init(&Ext->profile);

    // Set motion parameters - TUNE THESE
    Ext->max_velocity = 6.0f;       // Units per second
    Ext->accel_distance = 1.0f;     // Distance to accelerate
    Ext->decel_distance = 1.0f;     // Distance to decelerate
    Ext->tolerance = 0.005f;         // Position tolerance

    // Initial state
    Ext->current_position = current_pos;
    Ext->target_position = 0.0f;
    Ext->is_moving = 0;
}

/*
 * Function: Ext_MoveTo
 * Description: Command Ext to move to target position
 *
 * Can be called from ANY current position
 * Examples:
 *
 * Arguments:
 *   Ext: Pointer to Ext controller
 *   target: Target position (e.g., 3.0)
 */
void Ext_MoveTo(ExtController_t *Ext, float target) {
    // Calculate distance to travel
    float distance = target - *Ext->current_position;
    float abs_distance = fabsf(distance);

    // Check if already at target
    if (abs_distance < Ext->tolerance) {
        Ext->is_moving = 0;
        return;
    }

    // Handle short moves (not enough room for full accel/decel)
    float accel_dist = Ext->accel_distance;
    float decel_dist = Ext->decel_distance;

    if (abs_distance < accel_dist + decel_dist) {
        // Scale down acceleration/deceleration for short moves
        float scale = abs_distance / (accel_dist + decel_dist) * 0.8f;
        accel_dist *= scale;
        decel_dist *= scale;
    }

    // Set PathProfile parameters
    // offset: starting position (where we are now)
    // d1: acceleration distance
    // td: TOTAL distance to travel (signed: + forward, - backward)
    // d3: deceleration distance
    // v: velocity (absolute value)
    Path_Profile_Set(&Ext->profile,
                     *Ext->current_position,  // Start from current position
                     accel_dist,             // Accel distance
                     distance,               // Total distance (signed!)
                     decel_dist,             // Decel distance
                     Ext->max_velocity);     // Velocity

    // Check for errors (e.g., invalid parameters)
    if (Ext->profile.error) {
        Ext->is_moving = 0;
        return;
    }

    // Save target and start moving
    Ext->target_position = target;
    Ext->is_moving = 1;
}

/*
 * Function: Ext_Update
 * Description: Update Ext trajectory and get position command
 *
 * Call this in your MAIN LOOP or TIMER INTERRUPT
 *
 * Returns: Position command to send to Robomaster motor
 */
float Ext_Update(ExtController_t *Ext) {
    if (!Ext->is_moving) {
        // Not moving, return current position
        return *Ext->current_position;
    }

    // Update PathProfile trajectory
    Path_Profile_Update(&Ext->profile);

    // Get smooth position command from PathProfile
    float position_command = Ext->profile.path;

    // Check if motion is complete
    float error = fabsf(Ext->target_position - *Ext->current_position);
    /*Ext->profile.state == 0 && */
    if ( error < Ext->tolerance) {
        // Motion complete!
        Ext->is_moving = 0;
        position_command = Ext->target_position;  // Ensure exact target
    }

    return position_command;
}

/*
 * Function: Ext_Stop
 * Description: Stop Ext immediately at current position
 *
 * Use for emergency stop or user interrupt
 */
void Ext_Stop(ExtController_t *Ext) {
    Ext->is_moving = 0;
    Path_Profile_Reset(&Ext->profile);
    // Note: Motor will hold current position due to its internal PID
}

/*
 * Function: Ext_IsMoving
 * Description: Check if Ext is currently moving
 */
uint8_t Ext_IsMoving(ExtController_t *Ext) {
    return Ext->is_moving;
}
void Ext_SetTolerance(ExtController_t *Ext,float tolerance)
{
	Ext->tolerance = tolerance;
}
/*
 * Function: Ext_SetMotionParams
 * Description: Adjust motion parameters (velocity, accel/decel distances)
 */
void Ext_SetMotionParams(ExtController_t *Ext, float velocity, float accel_dist, float decel_dist) {
    Ext->max_velocity = velocity;
    Ext->accel_distance = accel_dist;
    Ext->decel_distance = decel_dist;
}
