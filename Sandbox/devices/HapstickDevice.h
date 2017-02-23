/**
* HapstickDevice.h
* Purpose: Define library functions to communicate between Chai3D and Hapstick Device
*
* @author Colette Knopp
* @version 1.0 2/22/2017
*/
#ifndef HAPSTICKDEVICE_H
#define HAPSTICKDEVICE_H

#define MAX_SIZE_OF_DATA 7 //"-30,-30 is 7 characters

#include "SerialPort.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

class HapstickDevice {
private:
	double prevX;
	double prevY;
	SerialPort *hapstick;

public:
	HapstickDevice();
	~HapstickDevice();

	int getDeviceCount(int *count);
	int open(unsigned int index);
	int close();
	int hapstickGetPosition(double angleX, double angleY);
};

#endif // SERIALPORT_H