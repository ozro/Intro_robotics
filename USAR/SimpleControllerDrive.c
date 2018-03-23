#include "JoystickDriver.c"
#pragma config(Sensor, S1,TIR,sensorI2CCustom)
#define ARDUINO_ADDRESS	0x14
#define ARDUINO_PORT S1

ubyte I2Cmessage[22];
char I2Creply[20];

void i2c_read_registers_text(ubyte register_2_read, int message_size, int return_size){
  memset(I2Creply, 0, sizeof(I2Creply));
	message_size = message_size+3;

  I2Cmessage[0] = message_size;    // Messsage Size
  I2Cmessage[1] = ARDUINO_ADDRESS;
  I2Cmessage[2] = register_2_read; // Register
  sendI2CMsg(S1, &I2Cmessage[0], return_size);
  wait1Msec(20);

  readI2CReply(ARDUINO_PORT, &I2Creply[0], return_size);
/*
  int i = 0;
  while(true){
  	writeDebugStream("%c", I2Creply[i]);
  	i++;
  	if(I2Creply[i] == 0) break;
  }
  writeDebugStreamLine(" ");
  */
}
task main()
{
	while(true)
	{
		getJoystickSettings(joystick);

		//Motor Control
		if(abs(joystick.joy1_y1)>15)
		{
			motor[motorB] = -1.0*joystick.joy1_y1;
		}
		else motor[motorB] = 0;
		if(abs(joystick.joy1_y2)>15)
		{
			motor[motorA] = -1.0*joystick.joy1_y2;
		}
		else motor[motorA] = 0;


		//set LED light
		if(joy1Btn(1) == 1 && joy1Btn(3) == 0 && joy1Btn(2) == 0 && joy1Btn(4) == 0)
		{
			i2c_read_registers_text(0x10, 0, 10);
		}
		if(joy1Btn(1) == 0 && joy1Btn(3) == 1 && joy1Btn(2) == 0 && joy1Btn(4) == 0)
		{
			i2c_read_registers_text(0x01, 0, 10);
		}
		if(joy1Btn(1) == 0 && joy1Btn(3) == 0 && joy1Btn(2) == 1 && joy1Btn(4) == 0)
		{
			i2c_read_registers_text(0x00, 0, 10);
		}
		if(joy1Btn(1) == 0 && joy1Btn(3) == 1 && joy1Btn(2) == 0 && joy1Btn(4) == 1)
		{
			i2c_read_registers_text(0x11, 0, 10);
		}
	}

}
