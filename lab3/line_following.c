/**********************************************
* Lab 3 : Starter code
* Written by Kaushik Viswanathan,
* Modified by Allan Wang (Jan 2017)

* Feel free to modify any part of these codes.
**********************************************/
//Set parameters
const int THRESHOLD = 15;
const int MOTOR_POWER = 10;
const int UPDATE_INTERVAL = 5;
const float CURVE_RATE = 0.5;

/*****************************************
* Main function - Needs changing
*****************************************/
#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)

task main()
{
	int motorPowerL;
	int motorPowerR;

	//Wait for sensor initialization
	wait1Msec(50);

	// Find the line by turning right
	while(SensorValue[S3] >= THRESHOLD){
		motorPowerL = MOTOR_POWER;
		motorPowerR = -1 * MOTOR_POWER;

		nxtDisplayCenteredTextLine(2, "Searching: %d", SensorValue[S3]);
		wait1Msec(UPDATE_INTERVAL);
	}
	while(1){
		// Curve left away from the line
		if(SensorValue[S3] < THRESHOLD){
			motorPowerL = MOTOR_POWER * CURVE_RATE;
			motorPowerR = MOTOR_POWER;

			nxtDisplayCenteredTextLine(2, "Turn Left");
		}
		else{
			motorPowerL = MOTOR_POWER;
			motorPowerR = MOTOR_POWER * CURVE_RATE;

			nxtDisplayCenteredTextLine(2, "Turn Right");
		}

		nxtDisplayCenteredTextLine(3, "Light: %d", SensorValue[S3]);

		// Update motors
		motor[motorA] = motorPowerL;
		motor[motorB] = motorPowerR;

		wait1Msec(UPDATE_INTERVAL);
	}
}
