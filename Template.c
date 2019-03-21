#include "72832Lib.h"

void pidCtrlStrt(int reqDist, int unitType = 0){

	//unit type for pid sensor selection. default is clicks
	if( unitType == 1 ){   //this unit type is inches

		int pidDist = reqDist * /*degrees / Wheel Circumference*/ (360/(4*3.141529)) ;

	}else if(unitType == 2){//this unit type is feet 

		int pidDist = reqDist * /*(degrees / Wheel Circumference) * 12*/ (4320/(4*3.141529));

	}else if(unitType == 3){ //this value is tiles (2 feet)

		int pidDist = reqDist * /*(degrees / Wheel Circumference) * 24*/ (8640/(4*3.141529);

	}else if(unitType == 0){ //this unit type is, by default, clicks

		int pidDist = reqDist; 

	}

	// Loop through many times so we can graph
	//  the PID loop
	for(int i = 0; i < 2000; i++){
		int sensorCurrentValue = SensorValue[
			exampleEncoderName //change this value to your main encoder name
		];

		// This calculates how far off we are from the true value
		//  The PID will return a response that will hopefully minimize this error over time
		float pidResult = PIDCompute( &pidExampleStruct, pidDist - sensorCurrentValue );

		driveFunc(pidResult);  //see void driveFunc

		delay1Ms( abs( reqDist * ( 2000 / 360 ) ) ); //if your reqDist = 360 then it will delay to one second. This can be adjusted but only for advanced users.

	}
}

void pidCtrlStrt(int reqDist, int turnBool = 1, int unitType = 0){

	//unit type for pid sensor selection. default is clicks
	if(unitType == -1){//this unit type is degrees which is more advanced but highly suggested

		int pidDist = reqDist * /* 360/ */ ((1440 * ((4*3.141529))/360);

	}else if( unitType == 1 ){   //this unit type is inches

		int pidDist = reqDist * /*degrees / Wheel Circumference*/ (360/(4*3.141529)) ;

	}else if(unitType == 2){//this unit type is feet 

		int pidDist = reqDist * /*(degrees / Wheel Circumference) * 12*/ (4320/(4*3.141529));

	}else if(unitType == 3){ //this value is tiles (2 feet)

		int pidDist = reqDist * /*(degrees / Wheel Circumference) * 24*/ (8640/(4*3.141529);

	}else if(unitType == 0){ //this unit type is, by default, clicks

		int pidDist = reqDist; 

	}

	// Loop through many times so we can graph
	//  the PID loop
	for(int i = 0; i < 2000; i++){
		int sensorCurrentValue = SensorValue[
			exampleEncoderName //change this value to your main encoder name
		];

		// This calculates how far off we are from the true value
		//  The PID will return a response that will hopefully minimize this error over time
		float pidResult = PIDCompute( &pidExampleStruct, pidDist - sensorCurrentValue );

		driveTurn(pidResult * turnInt);  //see void driveFunc

		delay1Ms( abs( reqDist * ( 2000 / 360 ) ) ); //if your reqDist = 360 then it will delay to one second. This can be adjusted but only for advanced users.

	}
}


task main(){

	// TODO: Your Code
	// See the Examples folder for example programs

}
