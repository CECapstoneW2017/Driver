/***
 * @file HapstickLib.cpp
 * @brief HapstickLib class, interface between Chai3d and Hapstick device
 * @author Colette Knopp
 *
 */

#include "HapstickLib.h"
#include <iostream>
#include <stdio.h>


	HapstickLib::HapstickLib()
	{
		//variables to keep track of previous position coordinates 
		this->prevX = 0;
		this->prevY = 0;
		this->prevZ = 0;
		this->hapstick;

		//leftover vars from falcon lib
		/*m_forceVec[0] = 0.0;
		m_forceVec[1] = 0.0;
		m_forceVec[2] = 0.0;
		m_position[0] = 0.0;
		m_position[1] = 0.0;
		m_position[2] = 0.0;
		*/
	}

	HapstickLib::~HapstickLib()
	{
		close();
	}

	/**
	* Return the number of connected Hapstick devices
	*
	* @param & count: number of devices
	* @returns True if no errors occured
	*/
	int HapstickLib::getDeviceCount(int *count)
	{
		//TODO: Programmatically determine how many Hapstick devices are connected
		*count = 1;
		return 1;
	}

	/**
	* Open connection with Hapstick device
	*
	* @param index: index of the device to open a connection with
	* @returns True if connection established successfully
	*/
	int HapstickLib::open(unsigned int index)
	{
		int bHapstickConnected = 0;
		char* portName = "\\\\.\\COM6"; //TODO: determine this programatically

		//Create instance of Serial Port
		hapstick = new SerialPort(portName);

		//Check if arduino is connected
		if (hapstick->isConnected()){
			bHapstickConnected = 1;
		}

		return bHapstickConnected;
	}

	/**
	* Closes a connection with Hapstick device
	*
	* @param none
	* @returns void
	*/
	int HapstickLib::close()
	{
		//Do something (delete instance of SerialPort or call destructor?)
		hapstick->~SerialPort();
		return 1;
	}

	/**
	* Gets position angles from the arduino
	*
	* @param angleX: pointer to X angle pos, angleY: pointer to Y angle pos
	* @returns error: 1 if no error, 0 if arduino is not connected
	*/
	int HapstickLib::hapstickGetPosition(double *angleX, double *angleY, double *angleZ) //add device id later
	{
		int retVal = 0; //set to true if position is attained
		int data_length = 0;
		char input[MAX_BUFFER_LENGTH + 1];
		double signalX, signalY, signalZ;
		double zeroedX, zeroedY, zeroedZ;
		*angleX = 0;
		*angleY = 0;
		*angleZ = 0;

		if (hapstick->isConnected()){
			//read position from serial buffer
			data_length = hapstick->readSerialPort(input, MAX_BUFFER_LENGTH);

			// buffer is full or has 1 data set
			if (data_length >= SIZE_OF_DATA) {
				//input[size] = 0; //end with null termination

				int lastNewLine = 0;
				int dataSetStart = 0;
				int index = data_length;

				//there is more than 1 data set in the buffer (possibly ending in truncated data)
				while ((index > 0) && (dataSetStart == 0)){
					if (input[index] == '\n') { //this is either the end of the desired data set or the end of the preceding data set
						if (lastNewLine == 0){ //hasn't found last new line character yet
							lastNewLine = index;
						}
						else{ //has already found end of desired data set, this \n is the end of preceding data set
							dataSetStart = index + 1; //add one to location of \n to get to the start of the desired data set 
						}
					}
					index--;
				}
				
				// check for readable data
				if ((lastNewLine - dataSetStart) == SIZE_OF_DATA-1){ //make sure it is a complete data set


					//Parse serial input into 2 char strings
					char* pos_arr = &input[dataSetStart];
					sscanf(pos_arr, "%lf,%lf,%lf", &signalX, &signalY, &signalZ);

					//Subtract potentiometer offset
					zeroedX = signalX - POT_ZERO;
					zeroedY = signalY - POT_ZERO;
					zeroedZ = signalZ - POT_ZERO;

					//filter out signal fluctuations
					double diffX = abs(zeroedX - (prevX / SIGNAL_TO_DEGREES));
					double diffY = abs(zeroedY - (prevY / SIGNAL_TO_DEGREES));
					double diffZ = abs(zeroedZ - (prevZ / SIGNAL_TO_DEGREES));

					if (diffZ > 10)
						boolean isPossessed = true;

					if (abs(prevZ) > 1024)
						boolean isJacked = true;

					if (diffX > MIN_SIGNAL_CHANGE && diffX < MAX_SIGNAL_CHANGE){ // Update angle only if signal is within acceptable range
						*angleX = (zeroedX)*SIGNAL_TO_DEGREES; //convert signal in range [-50,50] to degrees [-30,30]
						this->prevX = *angleX;
					}
					else{
						*angleX = this->prevX;
					}
					if (diffY > MIN_SIGNAL_CHANGE && diffY < MAX_SIGNAL_CHANGE){ // Update angle only if signal is within acceptable range
						*angleY = (zeroedY)*SIGNAL_TO_DEGREES; //convert signal in range [-50,50] to degrees [-30,30]
						this->prevY = *angleY;
					}
					else{
						*angleY = this->prevY;
					}if (diffZ > MIN_SIGNAL_CHANGE && diffZ < MAX_SIGNAL_CHANGE){ // Update angle only if signal is within acceptable range
						*angleZ = (zeroedZ)*SIGNAL_TO_DEGREES; //convert signal in range [-50,50] to degrees [-30,30]
						this->prevZ = *angleZ;
						if (abs(prevZ) > 10)
							boolean isJacked = true;
					}
					else{
						*angleZ = this->prevZ;
					}

					retVal = 1;
				}
				else{ //data set is not complete
					*angleX = this->prevX;
					*angleY = this->prevY;
					*angleZ = this->prevZ;
					if (true)
						char temp = 0;
					retVal = 1;
				}

			}
			else{ //not a full data set in the buffer
				*angleX = this->prevX;
				*angleY = this->prevY;
				*angleZ = this->prevZ;
				if (true)
					char temp = 0;
				retVal = 1;
			}
		}
		else{ //arduino not connected
			//angleX = 0;
			//angleY = 0;
		}
		return retVal;
	}; //end hapstickGetPosition

	/**
	* Send Force angles for the arduino
	*
	* @param angleX: pointer to X angle pos, angleY: pointer to Y angle pos
	* @returns error: 1 if no error, 0 if arduino is not connected
	*/
	int HapstickLib::hapstickSetForce(int *forceX, int *forceY, int *forceZ) { //add device id later
		char buffer[20];
		int size = 0;
		if (hapstick->isConnected()) {
			if (*forceY == 0){
				size = sprintf(buffer, "%d-%d\n\0", *forceX, *forceZ);
			}
			else{
				size = sprintf(buffer, "%d-%d\n\0", *forceX, *forceY);
			}
			
			hapstick->writeSerialPort(buffer, size);
		}
		else return 0;	// error
		
		return 1;		// no error
	}


	/*******************************************************************************************************************/

	/*
	int HapstickLib::hapstickGetLinearVelocity(double *vel1, double *vel2, double *pos1, double *pos2) {
		char buffer[20];

		if (hapstick->isConnected()) {	
			*vel1 = (*pos1 - prevX) / INCOMING_MSG_TIME;	// divided by 1 millisecond
			*vel2 = (*pos2 - prevY) / INCOMING_MSG_TIME;	// divided by 1 millisecond
		}
		else return 0;	// error

		return 1;		// no error
	}*/

	/*******************************************************************************************************************/