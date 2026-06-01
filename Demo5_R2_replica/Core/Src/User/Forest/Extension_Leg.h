/*
 * Extension_Leg.h
 *
 *  Created on: Nov 26, 2025
 *      Author: Ibrahim Meselhy
 */

#ifndef SRC_EXTENSION_LEG_H_
#define SRC_EXTENSION_LEG_H_

#include "../../Algorithms/PathProfile/PathProfile.h"

// Ext controller structure
typedef struct {
    Path_Profile_t profile;      // PathProfile for trajectory generation

    float* current_position;      // Current Ext position (from encoder)
    float target_position;       // Target position

    uint8_t is_moving;          // Movement status flag
    float tolerance;            // Position tolerance (how close is "close enough")

    // Motion parameters (tune for your Ext)
    float max_velocity;         // Maximum safe velocity
    float accel_distance;       // Distance for acceleration
    float decel_distance;       // Distance for deceleration

} ExtController_t;

extern ExtController_t front_ext;
extern ExtController_t rear_ext;
extern ExtController_t spear_arm;
extern ExtController_t spear_picker;
extern ExtController_t kfs_grip;
extern ExtController_t kfs_up;
extern ExtController_t kfs_yaw;
extern ExtController_t cubemars_ext;

void Ext_Init(ExtController_t *Ext,float* current_pos);
void Ext_MoveTo(ExtController_t *Ext, float target);
float Ext_Update(ExtController_t *Ext);
void Ext_Stop(ExtController_t *Ext);
uint8_t Ext_IsMoving(ExtController_t *Ext);
void Ext_SetTolerance(ExtController_t *Ext,float tolerance);
void Ext_SetMotionParams(ExtController_t *Ext, float velocity, float accel_dist, float decel_dist);

#endif /* SRC_EXTENSION_LEG_H_ */
