// PIDController.c
//
// Author: Justin Marple with Team BNS
// Contact: jmarple@umass.edu
// Date: 01/07/2015
//
// Migrated for: Michael Jones with 72832
// Contact: acetousk@gmail.com
// Date: 03/19/2019
//
// This source file includes source code that
// implements a PID controller for use in
// Vex Robotics Competition.
//
// Dependencies:
//    None!
//
// ------------------------------------------------------------------------
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// The author can be contacted by email at acetousk@gmail.com
//
// ------------------------------------------------------------------------

#pragma systemFile

#ifndef __BNS_PID_CONTROLLER_C
#define __BNS_PID_CONTROLLER_C

#ifndef __BNS_PID_CONTROLLER_H
#include "PIDController.h"
#endif

// Initializes the default values for the PID Controller
void PIDInit(struct PID *controller, float kP, float kI, float kD){
    controller->kP = kP;
    controller->kI = kI;
    controller->kD = kD;

    controller->error = controller->previousError = 0;
    controller->integral = controller->derivative = 0;

    controller->kILimit = 50;
}

// Computes the response for the PID controller
float PIDCompute(struct PID *controller, float error){
    controller->error = error;
    controller->integral += error;
    controller->derivative = error - controller->previousError;
    controller->previousError = error;

    if(controller->integral > controller->kILimit)
        controller->integral = controller->kILimit;

    if(controller->integral < -controller->kILimit)
        controller->integral = -controller->kILimit;

    return controller->kP * controller->error
                + controller->kI * controller->integral
                + controller->kD * controller->derivative;
}

// Sets the limit for the integral constant
void PIDSetIntegralLimit(struct PID *controller, float kILimit){
    controller->kILimit = kILimit;
}

// Restes the integral value
void PIDResetIntegral(struct PID *controller){
    controller->integral = 0;
}

void moveStraight(int reqDistance){
	SensorValue[rightEnc] = 0;

	// The PID controller that will be used
	// The basic usage of this PID controller is as following:
	//
	// PID pid1;
	// PIDInit(&pid1, PConstant, IConstant, DConstant);
	// float feedback = PIDCompute(&pid1, your_error);
	
	PID pid1;
	PIDInit(&pid1, 0.4, 0.03, 0.1); // Set P, I, and D constants

	int targetDistance = reqDistance * (360/12.56);
	int currentDistance;

	// Output instructions to view the PID response
	writeDebugStreamLine("*** Copy/paste all the results in the debug window to Excel and graph what the PID response looks like! ***");

	// Loop through many times so we can graph
	//  the PID loop
	for(int i = 0; i < 1000; i++){
		currentDistance = SensorValue[rightEnc];
		// This calculates how far off we are from the true value
		//  The PID will return a response that will hopefully minimize this error over time
		float pidResult = PIDCompute(&pid1, targetDistance - currentDistance);

		// Add pid to motor value
		motor[frontLeft]   = pidResult;
		motor[midLeft]     = pidResult;
		motor[backLeft]    = pidResult;
		motor[frontRight]  = pidResult;
		motor[midRight]    = pidResult;
		motor[backRight]   = pidResult;

		// There is a bug in RobotC where if you print too fast,
		//   you might get weird characters at random
		delay( reqDistance / 360 );
	}
}

void moveTurn(int reqDistance)
{
	SensorValue[rightEnc] = 0;

	// The PID controller that will be used
	// The basic usage of this PID controller is as following:
	//
	// PID pid1;
	// PIDInit(&pid1, PConstant, IConstant, DConstant);
	// float feedback = PIDCompute(&pid1, your_error);
	//
	// We start at 0 units and want to reach 100 units

	int currentDistance;

	PID pid1;
	PIDInit(&pid1, 0.4, 0.03, 0.1); // Set P, I, and D constants

	// Loop through many times so we can graph
	//  the PID loop
	for(int i = 0; i < 1000; i++){
		currentDistance = SensorValue[rightEnc];
		// This calculates how far off we are from the true value
		//  The PID will return a response that will hopefully minimize this error over time
		float pidResult = PIDCompute(&pid1, reqDistance - currentDistance);

		// Add pid to motor value
		motor[frontLeft]   = pidResult;
		motor[midLeft]     = pidResult;
		motor[backLeft]    = pidResult;
		motor[frontRight]  = pidResult * -1;
		motor[midRight]    = pidResult * -1;
		motor[backRight]   = pidResult * -1;

		// There is a bug in RobotC where if you print too fast,
		//   you might get weird characters at random
		delay(abs(reqDistance)/360 );
	}
}

#endif
