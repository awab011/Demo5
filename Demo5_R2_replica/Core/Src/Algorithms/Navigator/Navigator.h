/*
 * Navigator.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Ibrahim Meselhy
 */

#ifndef SRC_NAVIGATOR_NAVIGATOR_H_
#define SRC_NAVIGATOR_NAVIGATOR_H_

#include "../PathProfile/PathProfile.h"
#include "../PID/PID.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define NAV_MODE_PATHPROFILE 0  // Smooth trajectory following
#define NAV_MODE_PURE_PID 1     // Direct PID position lock

// Structure definition
typedef struct {
    float* current_x;
    float* current_y;
    float* current_yaw;

    float target_x;
    float target_y;
    float target_yaw;

    Path_Profile_t prof_x;
    Path_Profile_t prof_y;
    Path_Profile_t prof_yaw;

    float accel_dist;
    float max_velocity;
    float decel_dist;
    float max_angular_vel;

    PID_t pid_x;
    PID_t pid_y;
    PID_t pid_yaw;

    float error_x;
    float error_y;
    float error_yaw;
    float x_vel;
    float y_vel;
    float w_vel;

    uint8_t mode;  // 0 = PathProfile mode, 1 = Pure PID mode (position lock)
    uint8_t is_moving;
    uint8_t completed;
    uint8_t pos_locked;
    float tolerance_pos;
    float tolerance_yaw;
    uint8_t face_direction;
} Navigator_t;

// Function declarations (prototypes only)
float Normalize_Angle(float angle);
float Shortest_Angular_Distance(float from, float to);
void Navigator_Init(Navigator_t *nav,float* current_x,float* current_y,float* current_yaw);
void Navigator_MoveTo(Navigator_t *nav, float target_x, float target_y, float target_yaw);
void Navigator_Update(Navigator_t *nav);
void Navigator_MoveToWithoutYaw(Navigator_t *nav, float target_x, float target_y);
void Navigator_RotateToHeading(Navigator_t *nav, float target_yaw);
void Navigator_SetFaceDirection(Navigator_t *nav, uint8_t enable);
void Navigator_LockPosition(Navigator_t *nav);
void Navigator_LockPositionAt(Navigator_t *nav, float target_x, float target_y, float target_yaw);
void Navigator_Stop(Navigator_t *nav);
uint8_t Navigator_IsMoving(Navigator_t *nav);
float Navigator_GetVelocityX(Navigator_t *nav);
float Navigator_GetVelocityY(Navigator_t *nav);
float Navigator_GetAngularVelocity(Navigator_t *nav);
void Navigator_TunePID_Position(Navigator_t *nav, float kp, float ki, float kd);
void Navigator_TunePID_Yaw(Navigator_t *nav, float kp, float ki, float kd);
void Nav_SetMotionParams(Navigator_t *nav, float velocity, float accel_dist, float decel_dist);
void Nav_SetPosTol(Navigator_t *nav, float tol);
#endif /* SRC_NAVIGATOR_NAVIGATOR_H_ */
