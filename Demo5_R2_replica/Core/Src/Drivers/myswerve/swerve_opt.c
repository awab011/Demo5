/*
 * swerve.c
 *
 *  Created on: Oct 17, 2025
 *      Author: Ibrahim Meselhy
 */


/*
 * swerve.c - OPTIMIZED VERSION
 *
 * Improvements:
 * - Faster alignment
 * - Two alignment methods, incase the PWMEnoder fails.
 * - Fixed bug in the old library where after long use (many spins) the zero position shifts a little so the wheels werent aligned.
 *
 */

#include "../myswerve/swerve_opt.h"
#include "cmsis_os.h"


swerve_t myswerve;
MonoSwerve_t swerveA , swerveB, swerveC, swerveD;
// ============================================================================
// INITIALIZATION
// ============================================================================

/*
 * Function Name		: SwerveInit
 * Function Description : Called to initialize swerve structure.
 * Function Remarks		: Core function      ,User-Function.
 * Function Arguments	: swerve_type	     , Four wheeled or three wheeled
 * 						  swerve_gear_ratio	 , gear ratio for the steering motor (external gear ratio)
 * 						  width		         , width of the robot base
 * 						  length			 , length of the robot base
 * 						  *vesc				 , pointer to vesc structure that controls swerve wheels
 * 						  *rbms				 , pointer to RBMS_t struct that controls steering the swerve
 * 						  *swerve_t			 , pointer to the swerve structure responsible for storing the data
 * Function Return		: None
 * Function Example		: SwerveInit(MODN_FWD_SWERVE,2.0f,0.75,0.75,&vesc,&rbms1,&myswerve);
 */
void SwerveInit(int swerve_type, float swerve_gear_ratio, float width,
		float length, VESC_t *vesc, RBMS_t *rbms, swerve_t *swerve)
{
	// Clear structure
	memset(swerve, 0, sizeof(swerve_t));

	swerve->gear_ratio = swerve_gear_ratio;
	swerve->rbms = rbms;
	swerve->vesc = vesc;
	swerve->type = swerve_type;

	// Initialize MODN based on type
	if (swerve_type == MODN_FWD_SWERVE) {
		MODNRobotBaseVelInit(MODN_FWD_SWERVE, width, length, &modn);
		swerve->num_wheels = 4;
	} else if (swerve_type == MODN_TRI_SWERVE) {
		MODNRobotBaseVelInit(MODN_TRI_SWERVE, width, length, &modn);
		swerve->num_wheels = 3;
	}

	for (int i = 0; i < swerve->num_wheels; i++) {
		RBMS_Config(rbms, i, C610, swerve_gear_ratio);
		RBMS_Set_Control_Mode(rbms, i, POSITION);
		rbms->motor[i].config.vel_limit = 200;
		rbms->motor[i].config.POS_P = 300;
	}

	// Initialize VESC if tri-swerve
	if (swerve_type == MODN_TRI_SWERVE) {
		VESCInit(VESC1, VESC2, VESC3, 0, vesc);
	}
}

// ============================================================================
// ALIGNMENT INITIALIZATION
// ============================================================================
/*
 * Function Name		: MonoSwerve_Init
 * Function Description : Called to initialize MonoSwerve_t structure.
 * Function Remarks		: Core function      ,User-Function.
 * Function Arguments	: *mono			     , pointer pointing to the alignment structure for each wheel
 * 						  *rbms				 , pointer to RBMS_t struct that controls steering the swerve
 * 						  motor_idx			 , the rbms ID
 * 						  *reg		         , register of the IR(Hall) sensor
 * 						  bit  			     , the bit number in the register responsible for the Hall sensor
 * 						  *encoder		     , pointer to vesc structure that controls swerve wheels
 * 						  gear ratio		 , gear ratio for the steering motor (external gear ratio)
 * Function Return		: None
 * Function Example		: MonoSwerve_Init(&swerveA,&rbms1,RBMS1,&GPIOE->IDR,12,&enc1,2.0f,206.72);
 */
void MonoSwerve_Init(MonoSwerve_t *mono, RBMS_t *rbms, int motor_idx,
                     const volatile uint32_t* reg, uint8_t bit,
                     PWMEnc_t *encoder, float gear_ratio, float target_angle)
{
    mono->rbms = rbms;
    mono->motor_index = motor_idx;
    mono->encoder = encoder;
    mono->gear_ratio = gear_ratio;
    mono->target_angle = target_angle;
    mono->reg = reg;
    mono->bit = bit;

    mono->aligned = 0;
    mono->error = 0;
    mono->zero_angle = 0;
    mono->zero_rev = 0;
    mono->zero_offset = 0;
    mono->final_position = 0;
    mono->align_duration_ms = 0;
    mono->method_used = -1;
}
/*
 * Function Name		: MonoSwerve_AlignPWM
 * Function Description : tries aligning the swerve module using the AS5047p (PWM) encoder .
 * Function Remarks		: Internal function  ,Internal-Function Do Not use it.
 * Function Arguments	: *mono			     , pointer pointing to the alignment structure for each wheel
 * Function Return		: int
 * Function Example		:	MonoSwerve_AlignPWM(&mono)
 */

int MonoSwerve_AlignPWM(MonoSwerve_t *mono)
{
    uint32_t start_time = HAL_GetTick();
    const uint32_t TIMEOUT_MS = 3000;

    // Step 1: Update encoder reading
    PWMEncoder_Angle_Update(mono->encoder);

    // Step 2: Calculate zero angle
    mono->zero_angle = mono->encoder->Angle - mono->target_angle;

    // Normalize to [-180, 180]
    while (mono->zero_angle > 180.0f)
        mono->zero_angle -= 360.0f;
    while (mono->zero_angle < -180.0f)
        mono->zero_angle += 360.0f;

    // Step 3: Convert to motor position
    mono->zero_rev = mono->zero_angle / 360.0f;
    mono->zero_offset = (mono->rbms->motor[mono->motor_index].pos -
                         mono->zero_rev) /(mono->gear_ratio);

    // Step 4: Move to calculated position
    RBMS_Set_Target_Position(mono->rbms, mono->motor_index, mono->zero_offset);

    // Wait for motor to reach position
    osDelay(300);  // Give motor time to move

    // Step 5: Verify with hall sensor
    int attempts = 0;
    while (attempts < 2) {  // Try twice: 0° and 180°

        if (MonoSwerve_CheckAlignment(mono)) {
            // Success!
            mono->method_used = 0;  // PWM method
            mono->align_duration_ms = HAL_GetTick() - start_time;
            mono->error = 0;
            return 1;
        }

        // Not aligned, try 180° rotation
        if (attempts == 0) {
            RBMS_Set_Target_Position(mono->rbms, mono->motor_index,
                                     mono->zero_offset + 0.5f);
            osDelay(400);
        }

        attempts++;

        // Check timeout
        if (HAL_GetTick() - start_time > TIMEOUT_MS) {
            mono->error = 1;
            return 0;  // Timeout
        }
    }

    // Failed to align with PWM method
    return 0;
}
/*
 * Function Name		: MonoSwerve_AlignRbmaster
 * Function Description : tries aligning the swerve module using the Robomaster motor + Hall sensor .
 * Function Remarks		: Internal function  ,Internal-Function Do Not use it.
 * Function Arguments	: *mono			     , pointer pointing to the alignment structure for each wheel
 * Function Return		: int
 * Function Example		: MonoSwerve_AlignRbMaster(&mono)
 */

int MonoSwerve_AlignRbMaster(MonoSwerve_t *mono)
{
    uint32_t start_time = HAL_GetTick();
    const uint32_t TIMEOUT_MS = 8000;  // 8 second max (full rotation)
    // Start slow rotation
    mono->rbms->motor[mono->motor_index].config.vel_limit = 300;
    mono->rbms->motor[mono->motor_index].config.POS_P = 150;
    RBMS_Set_Control_Mode(mono->rbms, mono->motor_index, VELOCITY);
    RBMS_Set_Target_Velocity(mono->rbms, mono->motor_index, -300.0f);

    // Wait for hall sensor
    while (1) {
        // Check hall sensor
        if (readbit(mono->reg,mono->bit) == 0) {
            // Found zero! Stop motor
            RBMS_Set_Target_Velocity(mono->rbms, mono->motor_index, 0.0f);
//            mono->zero_rbms_current_ang = mono->rbms->motor[mono->motor_index].pos;
//            mono->zero_rbms_frac = modf((double)mono->zero_rbms_ang,&mono->zero_rbms_whole);
//            mono->zero_rbms_current_frac = modf((double)mono->zero_rbms_current_ang,&mono->zero_rbms_whole);
//            RBMS_Set_Control_Mode(mono->rbms, mono->motor_index, POSITION);
//            osDelay(50);
//            RBMS_Set_Target_Position(mono->rbms, mono->motor_index,-(float)(( mono->zero_rbms_current_frac - mono->zero_rbms_frac)));
//            osDelay(50);
            osDelay(100);
            if(MonoSwerve_CheckAlignment(mono)){
            RBMS_Set_Control_Mode(mono->rbms, mono->motor_index, POSITION);
            mono->method_used = 1;  // RbMaster method
            mono->align_duration_ms = HAL_GetTick() - start_time;
            mono->error = 0;
            return 1;
            }
            else{
                RBMS_Set_Target_Velocity(mono->rbms, mono->motor_index, 50.0f);
            }
        }

        // Check timeout
        if (HAL_GetTick() - start_time > TIMEOUT_MS) {
            // Timeout! Stop motor
            RBMS_Set_Target_Velocity(mono->rbms, mono->motor_index, 0.0f);
            mono->error = 1;
            return 0;
        }
        mono->zero_rbms_ang = mono->rbms->motor[mono->motor_index].pos;

        // Yield to other tasks
//        osDelay(5);  // Check every 5ms
    }
}
/*
 * Function Name		: MonoSwerve_Align
 * Function Description : checks if the wheel is already aligned, if not then use the PWMEncoder if it is not a NULL pointer,
 * 						  if that doesnt work then try using the robomaster, if Not then give an error .
 * Function Remarks		: Core function  	 ,User-Function.
 * Function Arguments	: *mono			     , pointer pointing to the alignment structure for each wheel
 * Function Return		: int
 * Function Example		: MonoSwerve_Align(&swerveA)
 */
int MonoSwerve_Align(MonoSwerve_t *mono)
{
    uint32_t start_time = HAL_GetTick();

    // Step 1: Quick passive check (maybe already aligned!)
    if (MonoSwerve_CheckAlignment(mono)) {
        mono->method_used = -1;  // No active alignment needed
        mono->align_duration_ms = 0;
    	mono->rbms->motor[mono->motor_index].config.vel_limit = 200;
    	RBMS_Set_Control_Mode(mono->rbms,mono->motor_index,POSITION);
        return 1;
    }

    // Step 2: Try PWM encoder method (fast, requires encoder)
    if (mono->encoder != NULL) {
        if (MonoSwerve_AlignPWM(mono)) {
        	mono->rbms->motor[mono->motor_index].config.vel_limit = 200;
        	RBMS_Set_Control_Mode(mono->rbms,mono->motor_index,POSITION);
            return 1;  // Success!
        }
    }

    // Step 3: Fallback to RbMaster method (slow but reliable)
    if (MonoSwerve_AlignRbMaster(mono)) {
    	mono->rbms->motor[mono->motor_index].config.vel_limit = 200;
    	RBMS_Set_Control_Mode(mono->rbms,mono->motor_index,POSITION);
        return 1;  // Success!
    }

    // Step 4: Total failure
    mono->error = 1;
    mono->align_duration_ms = HAL_GetTick() - start_time;
    return 0;
}

// ============================================================================
// SWERVE RUN
// ============================================================================
/*
 * Function Name		: SwerveRun
 * Function Description : Runs the MODN commands and translates it into motor commands
 * Function Remarks		: Core function  		 ,User-Function.
 * Function Arguments	: *swerve			     , pointer pointing to the swerve structure
 * Function Return		: None
 * Function Example		: SwerveRun(&myswerve)
 */

void SwerveRun(swerve_t *swerve)
{
	if(swerve->aligned){
	// Calculate total velocity to check if moving
	 swerve->total_vel = 0.0f;

	for (int i = 0; i < swerve->num_wheels; i++) {
		swerve->total_vel += fabsf(swerve->vel[i]);
	}
	// ────────────────────────────────────────────────────────────
	// MOVING: Calculate wheel angles and velocities
	// ────────────────────────────────────────────────────────────
	if (swerve->total_vel > 0.01f) {  // Small threshold to avoid noise
		swerve->timer = 0;

		// Process all wheels using for loop
		for (int i = 0; i < swerve->num_wheels; i++) {
			// Handle angle wrapping
			float angle_diff = swerve->ang[i] - swerve->ang_prev[i];

			if (angle_diff < -180.0f) {
				swerve->addvalue[i] += 360.0f;
			} else if (angle_diff > 180.0f) {
				swerve->addvalue[i] -= 360.0f;
			}

			// Calculate unwrapped angle
			swerve->total_ang[i] = swerve->ang[i] + swerve->addvalue[i];

			// Calculate angle differences
			swerve->ang1[i] = swerve->total_ang[i] - swerve->total_ang_prev[i];
			swerve->ang2[i] = (swerve->ang1[i] >= 0) ?
					swerve->ang1[i] - 180.0f :
					swerve->ang1[i] + 180.0f;

			// Update previous values
			swerve->ang_prev[i] = swerve->ang[i];
			swerve->total_ang_prev[i] = swerve->total_ang[i];

			// Choose shortest path (optimization logic)
			if (swerve->state[i] == 0) {
				swerve->final_ang[i] = swerve->final_ang_prev[i] + swerve->ang1[i];
				swerve->sign[i] = 1.0f;

				if (fabsf(swerve->ang1[i]) > fabsf(swerve->ang2[i])) {
					swerve->state[i] = 1;
					swerve->final_ang[i] = swerve->final_ang_prev[i] + swerve->ang2[i];
					swerve->sign[i] = -1.0f;
				}
			} else {  // state == 1
				swerve->final_ang[i] = swerve->final_ang_prev[i] + swerve->ang1[i];
				swerve->sign[i] = -1.0f;

				if (fabsf(swerve->ang1[i]) > fabsf(swerve->ang2[i])) {
					swerve->state[i] = 0;
					swerve->final_ang[i] = swerve->final_ang_prev[i] + swerve->ang2[i];
					swerve->sign[i] = 1.0f;
				}
			}

			swerve->final_ang_prev[i] = swerve->final_ang[i];

			// Apply velocity with direction sign
			swerve->fvel[i] = swerve->vel[i] * swerve->sign[i];
		}

		// Send commands to motors
		VESCVelocity(swerve->fvel[0], swerve->fvel[1],
				swerve->fvel[2], swerve->fvel[3], swerve->vesc);

		// Set wheel angles (for loop!)
		for (int i = 0; i < swerve->num_wheels; i++) {
			RBMS_Set_Target_Position(swerve->rbms, i, (swerve->final_ang[i] / 360.0f));
		}
	}
	// ────────────────────────────────────────────────────────────
	// STOPPED: Reset to zero position
	// ────────────────────────────────────────────────────────────
	else {
		VESCVelocity(0, 0, 0, 0, swerve->vesc);


		if (swerve->timer > 600) {
			for (int i = 0; i < swerve->num_wheels; i++) {
				swerve->rbms->motor[i].reset_pos = 1;
				swerve->align_flag = 1;
				swerve->ang[i] = 0.0f;
				swerve->ang_prev[i] = 0.0f;
				swerve->addvalue[i] = 0.0f;
				swerve->total_ang[i] = 0.0f;
				swerve->total_ang_prev[i] = 0.0f;
				swerve->final_ang[i] = 0.0f;
				swerve->final_ang_prev[i] = 0.0f;
				swerve->state[i] = 0;
			}
		}
	}
	}
}
/*
 * Function Name		: MonoSwerve_CheckAlignment
 * Function Description : Reads the hall sensor pin using register access.
 * Function Remarks		: Helper function  		 ,Internal-Function, Do Not use it.
 * Function Arguments	: *mono 			     , pointer pointing to the swerve structure
 * Function Return		: int
 * Function Example		: MonoSwerve_CheckAlignment(&mono);
 */
int MonoSwerve_CheckAlignment(MonoSwerve_t *mono)
{

    uint8_t state = readbit(mono->reg,mono->bit);
    if (state == 0) {
        mono->aligned = 1;
        mono->final_position = mono->rbms->motor[mono->motor_index].pos;
        return 1;
    }

    mono->aligned = 0;
    return 0;
}
