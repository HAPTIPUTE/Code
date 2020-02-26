/*
 * Copyright (C) 2017 EPFL-LSRO (Laboratoire de Systemes Robotiques).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
#define RADIUS 0.075 // the radius of the paddle [m]

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].
volatile float32_t hapt_stiffness; // Motor torque [N.m].
volatile float32_t hapt_restAngle;
volatile float32_t hapt_dampingFactor;
float32_t motorShaftAngleInit;

void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{

    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;


    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("stiffness [N*m/rad]", (float32_t*)&hapt_stiffness, READWRITE);
    comm_monitorFloat("Rest Angle [rad]", (float32_t*)&hapt_restAngle, READWRITE);
    comm_monitorFloat("damping factor [unit]", (float32_t*)&hapt_restAngle, READWRITE);
    comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("angular_speed [deg]", (float32_t*)&hapt_angularSpeed, READONLY);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);

//    if(hall_GetVoltage() > 2.6)
//   	{
//    	while(hall_GetVoltage() > 2.6){
//    		torq_SetTorque(0.002);
//    	}
//    }else{
//    	while(hall_GetVoltage() < 2.6){
//    	    		torq_SetTorque(-0.002);
//    	    	}
//    }
//    hapt_restAngle =  3.14*(enc_GetPosition()/REDUCTION_RATIO)/180;
//    torq_SetTorque(0);
}

/**
  * @brief Updates the haptic controller state.
  */


float32_t numerical_Diff(float32_t old, float32_t new, float32_t dt)
{
	return (new-old)/(2*dt) ;
}

void hapt_Update()
{
    float32_t motorShaftAngle; // [deg].
    float32_t hapt_encoderAngleRad ;
    float32_t hapt_encoderAngleOld = 0 ;

    // Compute the dt (uncomment if you need it).
    float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / (1000000.0f); // [s].

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();
    
    // Get the Hall sensor voltage.
    hapt_hallVoltage = hall_GetVoltage();

    // Get the encoder position.
    hapt_encoderAngleOld = hapt_encoderPaddleAngle ;
    motorShaftAngle = enc_GetPosition() ;
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;
//    hapt_encoderAngleRad = 3.14*hapt_encoderPaddleAngle/180;

    hapt_angularSpeed=numerical_Diff( hapt_encoderAngleOld ,hapt_encoderPaddleAngle , dt);

    // Compute the motor torque, and apply it.
//    hapt_motorTorque = -hapt_stiffness*(hapt_encoderAngleRad-hapt_restAngle)/REDUCTION_RATIO ;
    //hapt_motorTorque = 0.0f;
    hapt_motorTorque
    torq_SetTorque(hapt_motorTorque);
}


