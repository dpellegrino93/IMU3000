#include <Wire.h>
#include "IMU3000.h"

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


IMU3000::IMU3000()
{
	gx = gy = gz = 0;
	temp = 0;
}

void IMU3000::writeTo(byte address, byte val)
{
	Wire.beginTransmission(IMU3000_ADDR);
	Wire.write(address);             
	Wire.write(val);                 
	Wire.endTransmission();
}

void IMU3000::readFrom(byte address, int num, byte buff[])
{
	Wire.beginTransmission(IMU3000_ADDR);
	Wire.write(address);
	Wire.endTransmission();
	
	Wire.beginTransmission(IMU3000_ADDR);
	Wire.requestFrom(IMU3000_ADDR, num);
	
	int i = 0;
	while(Wire.available())
	{ 
		buff[i] = Wire.read();
		i++;
	}
	Wire.endTransmission();
}


void IMU3000::setRegisterBit(byte regAdress, int bitPos, bool state) 
{
	byte _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);
	} 
	else {
		_b &= ~(1 << bitPos);
	}
	writeTo(regAdress, _b);  
}

bool IMU3000::getRegisterBit(byte regAdress, int bitPos) 
{
	byte _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

void IMU3000::init()
{
	Wire.begin();
}

void IMU3000::read()
{
	byte _buff[8];
	readFrom(IMU3000_REG_TEMP_OUT_H	, 8, _buff);
	temp = _buff[0]<<8 | _buff[1];
	gx = _buff[2]<<8 | _buff[3];
	gy = _buff[4]<<8 | _buff[5];
	gz = _buff[6]<<8 | _buff[7];
}
	
void IMU3000::reset()
{
	setRegisterBit(IMU3000_REG_PWR_MAN, IMU3000_H_RESET,true);
}

void IMU3000::setSleepState(bool state)
{
	setRegisterBit(IMU3000_REG_PWR_MAN, IMU3000_SLEEP, state);
}

bool IMU3000::getSleepState()
{
	byte buf;
	readFrom(IMU3000_REG_PWR_MAN,1,&buf);
	return buf && BITMASK(IMU3000_SLEEP);
}

void IMU3000::setSampleRate(byte divider, byte lowpass)
{
	//writing divider
	writeTo(IMU3000_REG_SMPLRT_DIV,divider);

	//writing lowpass rate freq
	byte lpf;
	readFrom(IMU3000_REG_DLPF, 1, &lpf);
	
	lpf &= ~IMU3000_DLPF_CFG_MASK;
	lpf |= lowpass;

	writeTo(IMU3000_REG_DLPF, lpf);  

}


void IMU3000::setRange(byte range)
{
	byte rng;
	readFrom(IMU3000_REG_DLPF, 1, &rng);
	
	rng &= ~IMU3000_FS_SEL_MASK;
	rng |= range;

	writeTo(IMU3000_REG_DLPF, rng);  
}

int	IMU3000::getRange()
{
	byte rng;
	readFrom(IMU3000_REG_DLPF, 1, &rng);
	switch(rng)
	{
		case RANGE_250:
			return 250;
			break;
		case RANGE_500:
			return 500;
			break;
		case RANGE_1000:
			return 1000;
			break;
		case RANGE_2000:
			return 2000;
			break;
	}
	return 0;
}
