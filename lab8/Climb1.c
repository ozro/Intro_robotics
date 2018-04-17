#pragma config(Sensor, S2,     lightSensor,    sensorLightActive)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop, encoder)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop, encoder)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

const int rotationCount = 360;
bool sensed = false;
int finalCount = 0;
task main()
{
	nMotorEncoder[motorC] = 0;

	motor[motorC] = 55;
	while(nMotorEncoder[motorC] <= 2.5*rotationCount)
	{
		//motor[motorC] = 50;
		nxtDisplayCenteredTextLine(4, "Sensor: %f", SensorValue[S2]);
		nxtDisplayCenteredTextLine(5, "Counts: %f", nMotorEncoder[motorC]);
		writeDebugStreamLine("%d", SensorValue[S2])

		nxtDisplayCenteredTextLine(2, "Rung: %d", floor(nMotorEncoder[motorC]/180));
		int rung = floor(nMotorEncoder[MotorC]/180);
		if(!sensed && SensorValue[S2] >= 30 && floor(nMotorEncoder[motorC]/180)%2 == 0)
		{
			playSound(soundBeepBeep);
			nxtDisplayCenteredTextLine(3, "Even: %f", SensorValue[S2]);
			sensed = True

			//finalCount = nMotorEncoder[motorC] + 0.4*rotationCount;
			finalCount = rung * 180 + 45;
		}
		else if(!sensed && SensorValue[S2] >= 40)
		{
			playSound(soundBeepBeep);
			nxtDisplayCenteredTextLine(3, "Odd: %f", SensorValue[S2]);
			sensed = True
			//finalCount = nMotorEncoder[motorC] + 0.20*rotationCount;
			finalCount = rung * 180 + 40;
		}
		while(sensed)
		{
				nxtDisplayCenteredTextLine(4, "Sensor: %f", SensorValue[S2]);
				nxtDisplayCenteredTextLine(5, "Counts: %f", nMotorEncoder[motorC]);
				writeDebugStreamLine("%d", SensorValue[S2]);
				if(nMotorEncoder[motorC] <= finalCount){
					motor[motorC] = 50;
				}
				else{
					motor[motorC] = 0;
				}
		}
	}
	motor[motorC] = 0;
	return;
}
