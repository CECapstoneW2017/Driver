/**
* HapstickDevice.h
* Purpose: Define library functions to communicate between Chai3D and Hapstick Device
*
* @author Colette Knopp
* @version 1.0 2/22/2017
*/
#ifndef HAPSTICKDEVICE_H
#define HAPSTICKDEVICE_H

#define SIZE_OF_DATA 13 //"512,512,512\r\n is 13 characters
#define MAX_BUFFER_LENGTH 1000	//was 255********************
#define SIGNAL_TO_DEGREES 0.263 //30 degrees / 114 ticks
#define POT_ZERO 512
#define MAX_SIGNAL_CHANGE 228
#define MIN_SIGNAL_CHANGE 5
#define INCOMING_MSG_TIME .01

#include "SerialPort.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

class HapstickLib {
private:
	double prevX, prevY, prevZ;
	SerialPort *hapstick;

public:
	HapstickLib();
	~HapstickLib();

	int getDeviceCount(int *count);
	int open(unsigned int index);
	int close();
	int hapstickGetPosition(double *angleX, double *angleY, double *angleZ);
	int hapstickSetForce(int *forceX, int *forceY, int *forceZ);
};

#endif // SERIALPORT_H