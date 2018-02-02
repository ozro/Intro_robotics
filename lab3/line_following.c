/**********************************************
* Lab 3 : Starter code
* Written by Kaushik Viswanathan,
* Modified by Allan Wang (Jan 2017)

* Feel free to modify any part of these codes.
**********************************************/
//Global PID Variables
float last_error;

//Set parameters
const int LOW_THRESHOLD = 36;
const int HIGH_THRESHOLD = 66;
//Brightness value at the "gray zone" between line and background
const int THRESHOLD = (LOW_THRESHOLD + HIGH_THRESHOLD)/2;
const int MOTOR_POWER = 60;	 //Base motor power to control the speed
const int UPDATE_INTERVAL = 1; //Delay updates by x miliseconds

const float kP = 1.4;
const float kD = 20;

/*****************************************
* Main function - Needs changing
*****************************************/
#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)

task main()
{
	//Wait for sensor initialization
	wait1Msec(50);

	clearTimer(T1);

	// Find the line by turning left, then if line is not found within angle, turn right
	while(SensorValue[S3] >= THRESHOLD ){
		if (time1[T1] < 500) {
		motor[motorA] = MOTOR_POWER;
		motor[motorB] = -1 * MOTOR_POWER;
		}
		else {
			motor[motorA] = -1 * MOTOR_POWER;
			motor[motorB] = MOTOR_POWER;
		}


		nxtDisplayCenteredTextLine(2, "Searching: %d", SensorValue[S3]);
		wait1Msec(UPDATE_INTERVAL);
	}
	// Start line following
	clearTimer(T1);
	while(1){
		int dt = time1[T1];
		clearTimer(T1);
		if(dt == 0) {
			continue;
		}

		//PID
		float actual = SensorValue[S3];
		float error =  THRESHOLD - actual; // Negative if too dark, positive if too bright
		float d_error = (error-last_error)/dt;
		last_error = error;

		float signal = kP * error + kD * d_error;
		signal = signal/15;
		nxtDisplayCenteredTextLine(2, "Brightness: %d", SensorValue[S3]);
		nxtDisplayCenteredTextLine(3, "Signal: %f", signal);

		// Update motors
		if(signal > 0){ //Turn right
			//motor[motorA] = MOTOR_POWER  * (-2*signal*signal+1); 	//Right Motor
			motor[motorB] = MOTOR_POWER  * (-2*signal+1);
			motor[motorA] = MOTOR_POWER; //Left Motor
		}
		else { //Turn left
			motor[motorB] = MOTOR_POWER; 	//Right Motor
			motor[motorA] = MOTOR_POWER * (2*signal+1); //Left Motor
		}
		wait1Msec(UPDATE_INTERVAL);
	}
}
