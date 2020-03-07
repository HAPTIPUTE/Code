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
#include "lib/basic_filter.h"


#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
#define RADIUS 0.075 // the radius of the paddle [m]

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].
volatile float32_t hapt_stiffness; // Motor torque [N.m].
volatile float32_t hapt_restAngle;
volatile float32_t hapt_dampingFactor;
volatile float32_t hapt_angularSpeed;
volatile float32_t hapt_estimVToAngle;
volatile float32_t hapt_estimVToAngSpeed;
bfilt_BasicFilter BasicFilter;
volatile float32_t hapt_angularAcc;
volatile float32_t hapt_estimVToAngAcc;


float32_t motorShaftAngleInit;

void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{

    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;
    hapt_restAngle = 0.0f ;


    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("stiffness [N*m/rad]", (float32_t*)&hapt_stiffness, READWRITE);
    comm_monitorFloat("Rest Angle [rad]", (float32_t*)&hapt_restAngle, READWRITE);
    comm_monitorFloat("damping factor [unit]", (float32_t*)&hapt_dampingFactor, READWRITE);
    comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("angular_speed [deg/s]", (float32_t*)&hapt_angularSpeed, READONLY);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);
    comm_monitorFloat("hall_estimate angle from V [deg]", (float32_t*)&hapt_estimVToAngle, READONLY);
    comm_monitorFloat("hall_estimate angular speed from V [deg/s]", (float32_t*)&hapt_estimVToAngSpeed, READONLY);
    comm_monitorFloat("angular_acceleration [deg/s^2]", (float32_t*)&hapt_angularAcc, READONLY);
    comm_monitorFloat("hall_estimate angular acceleration from V [deg/s^2]", (float32_t*)&hapt_estimVToAngAcc, READONLY);

    float32_t tau = 0.2;
    float32_t initialValue = 0;
    bfilt_Init(&BasicFilter, tau, initialValue);


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


float32_t VoltToDeg(float32_t Volt) {
	return -52.1345 + 38.671 * Volt - 50;
}

float32_t numerical_Diff(float32_t old, float32_t new, float32_t dt) {
	return (new-old)/(dt) ;
}


void hapt_Update()
{
	float32_t motorShaftAngle; // [deg].
    float32_t hapt_encoderAngleRad ;
    float32_t hapt_encoderAngleOld = 0 ;
    float32_t hapt_estimVToAngleOld = 0 ;
    float32_t hapt_estimVToAngleOldOld = 0;
    float32_t hapt_encoderAngleOldOld = 0;
    float32_t hapt_angularSpeedOld = 0;
    float32_t hapt_estimVToAngSpeedOld = 0;


    // Compute the dt (uncomment if you need it).
    float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / (1000000.0f); // [s].

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();
    
    // Get the Hall sensor voltage.
    hapt_hallVoltage = hall_GetVoltage();

    hapt_hallVoltage = bfilt_Step(&BasicFilter, hapt_hallVoltage);

    // Get the encoder position.
    hapt_encoderAngleOldOld = hapt_encoderAngleOld;
    hapt_encoderAngleOld = hapt_encoderPaddleAngle ;
    motorShaftAngle = enc_GetPosition() ;
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;
    hapt_encoderAngleRad = 3.14*hapt_encoderPaddleAngle/180;
    hapt_angularSpeedOld = hapt_angularSpeed;

    hapt_angularSpeed=numerical_Diff( hapt_encoderAngleOld ,hapt_encoderPaddleAngle , dt);
    hapt_angularAcc=numerical_Diff( hapt_angularSpeedOld ,hapt_angularSpeed , dt);

    //hapt_angularSpeed=numerical_Diff( hapt_encoderAngleOldOld ,hapt_encoderPaddleAngle ,2* dt);

    // Compute the motor torque, and apply it.
    //hapt_motorTorque = 0.0f;
    if(abs(hapt_restAngle - hapt_encoderPaddleAngle) <= 20 )
    {
    	hapt_motorTorque = -hapt_dampingFactor*hapt_angularSpeed-hapt_stiffness*(hapt_encoderAngleRad-hapt_restAngle)/REDUCTION_RATIO ; ;
    }else{
    	hapt_motorTorque = -hapt_stiffness*(hapt_encoderAngleRad-hapt_restAngle)/REDUCTION_RATIO ;
    }
    utils_SaturateF(&hapt_motorTorque, -0.05f, 0.05f) ;
//	hapt_motorTorque = -hapt_stiffness*(hapt_encoderAngleRad-hapt_restAngle)/REDUCTION_RATIO ;
    torq_SetTorque(hapt_motorTorque);
    hapt_estimVToAngleOldOld = hapt_estimVToAngleOld;

    hapt_estimVToAngleOld = hapt_estimVToAngle;

    hapt_estimVToAngle = VoltToDeg(hapt_hallVoltage);
    hapt_estimVToAngSpeedOld = hapt_estimVToAngSpeed;
    hapt_estimVToAngSpeed = numerical_Diff( hapt_estimVToAngleOld,hapt_estimVToAngle,dt);

    hapt_estimVToAngAcc = numerical_Diff( hapt_estimVToAngSpeedOld,hapt_estimVToAngSpeed,dt);

    //hapt_estimVToAngSpeed = bfilt_Step(&BasicFilter, hapt_estimVToAngSpeed);

}
