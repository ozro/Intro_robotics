#pragma config(Sensor, S2,     backLight,      sensorLightActive)
#pragma config(Sensor, S3,     frontLight,     sensorLightActive)
#pragma config(Motor,  motorA,          rightMotor,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          leftMotor,     tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/**********************************************
* Lab 4 : Starter code
* Written by Kaushik Viswanathan,
* Modified by Allan Wang (Jan 2017)

* Feel free to modify any part of these codes.
**********************************************/
//Global PID Variables
float last_error;

//Set parameters
//const int LOW_THRESHOLD = 36;
//const int HIGH_THRESHOLD = 66;
//Brightness value at the "gray zone" between line and background
//const int THRESHOLD = (LOW_THRESHOLD + HIGH_THRESHOLD)/2;
const float MOTOR_POWER = 1;	 //Base motor power to control the speed
const float UPDATE_INTERVAL = 1; //Delay updates by x miliseconds

const float kP = 4;
const float kD = .3;

/*****************************************
* Main function - Needs changing
*****************************************/

/*task main()
{
	motor[motorA] = MOTOR_POWER;
}*/

task main()
{
	//Wait for sensor initialization
	wait1Msec(50);

	clearTimer(T1);

	// Start balancing
	clearTimer(T1);
	while(1){

		int dt = time1[T1];
		clearTimer(T1);
		if(dt == 0) {
			continue;
		}

		//PID
		float error = SensorValue(frontLight) - SensorValue(backLight)+1; //Positive error = tilting forward. Negative error = tilting backward
		float d_error = (error-last_error)/dt;
		last_error = error;

		float signal = kP * error + kD * d_error;
		//signal = signal*5;
		nxtDisplayCenteredTextLine(2, "Error: %f", error);
		nxtDisplayCenteredTextLine(3, "Signal: %f", signal);

		// Update motors
		if(signal > 0){ //Tilting forward, need to move backward
			motor(leftMotor) = MOTOR_POWER * signal;
			motor(rightMotor) = motor(leftMotor); //Left Motor
		}
		else if (signal < 0) { //Tilting backward, need to move forward
			motor(leftMotor) = MOTOR_POWER * signal;
			motor(rightMotor) = motor(leftMotor); //Left Motor
		}
		else {
			motor(leftMotor) = 0;
			motor(rightMotor) = 0;
		}
		nxtDisplayCenteredTextLine(4, "Left motor: %f", motor(leftMotor));
		nxtDisplayCenteredTextLine(5, "Right motor: %f", motor(rightMotor));
		wait1Msec(UPDATE_INTERVAL);
	}
}
