#include "Arduino.h"

/*
  ##########################
  # Arduino IMU3000 driver #
  ##########################
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/


#ifndef IMU3000_h
#define IMU3000_h

#define BITMASK(a) 1<<a
//I2C address
#define IMU3000_ADDR 0x68  

//Values
#define RANGE_250			B00000000
#define RANGE_500			B00001000
#define RANGE_1000			B00010000
#define RANGE_2000			B00011000

#define DLPF_256			0x0
#define DLPF_188			0x1
#define DLPF_98				0x2
#define DLPF_42				0x3
#define DLPF_20				0x4
#define DLPF_10				0x5
#define DLPF_5				0x6

//Registers
#define IMU3000_REG_WHO_AM_I		0x00	//Contains IMU3000 6 bit I2C address B1-B6

//SMPLRT_DIV = Sample rate divider
#define IMU3000_REG_SMPLRT_DIV		0x15	//Fsample=Finternal/(SMPLRT_DIV+1)

//DLPF = Range(FS_SEL) and digital low pass filter(DLPF_CFG) config
/*
DLPF_CFG ->	0 = 256Hz	(8kHz internal)
			1 = 188Hz	(1kHz internal)
			2 = 98Hz	(1kHz internal)
			3 = 42Hz	(1kHz internal)
			4 = 20Hz	(1kHz internal)
			5 = 10Hz	(1kHz internal)
			6 = 5Hz		(1kHz internal)


FS_SEL ->	0 = +-250 deg/sec
			1 = +-500 deg/sec
			2 = +-1000 deg/sec
			3 = +-2000 deg/sec
*/
#define IMU3000_REG_DLPF			0x16	//B0-1-2 = DLPF_CFG  ## B3-4 = FS_SEL

#define IMU3000_REG_TEMP_OUT_H		0x1B	//Temperature High
#define IMU3000_REG_TEMP_OUT_L		0x1C	//Temperature Low
#define IMU3000_REG_GYRO_XOUT_H		0x1D	//Gyro data
#define IMU3000_REG_GYRO_XOUT_L		0x1E
#define IMU3000_REG_GYRO_YOUT_H		0x1F
#define IMU3000_REG_GYRO_YOUT_L		0x20
#define IMU3000_REG_GYRO_ZOUT_H		0x21
#define IMU3000_REG_GYRO_ZOUT_L		0x22

//Power management register
//B7 = H_RESET -> 1 = reset all to power on
//B6 = SLEEP
#define IMU3000_REG_PWR_MAN			0x3E

//############
//### Bits ###
//############
#define IMU3000_H_RESET			0x7
#define IMU3000_SLEEP			0x6

#define IMU3000_DLPF_CFG_MASK		0x7

#define IMU3000_FS_SEL_MASK			0x18

class IMU3000
{
public:
	int gx;
	int gy;
	int gz;
	int temp;
	
	IMU3000();
	void init();
	void read();
	
	void reset();

	void setSleepState(bool state);
	bool getSleepState();

	void setSampleRate(byte divider, byte lowpass);

	void setRange(byte range);
	int	getRange();

	void writeTo(byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);
	void setRegisterBit(byte regAdress, int bitPos, bool state);
	bool getRegisterBit(byte regAdress, int bitPos);  

};

#endif