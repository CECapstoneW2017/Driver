/***
 * @file HapstickDevice.cpp
 * @brief HapstickDevice class, interface between Chai3d and Hapstick device
 * @author Colette Knopp
 *
 */

#include "HapstickDevice.h"
#include <iostream>
#include <stdio.h>


	HapstickDevice::HapstickDevice()
	{
		//variables to keep track of previous position coordinates 
		this->prevX = 0;
		this->prevY = 0;
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

	HapstickDevice::~HapstickDevice()
	{
		close();
	}

	/**
	* Return the number of connected Hapstick devices
	*
	* @param & count: number of devices
	* @returns True if no errors occured
	*/
	int HapstickDevice::getDeviceCount(int *count)
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
	int HapstickDevice::open(unsigned int index)
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
	int HapstickDevice::close()
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
	int HapstickDevice::hapstickGetPosition(double angleX, double angleY) //add device id later
	{
		bool retVal = 0; //set to true if position is attained
		int size = 0;
		char input[MAX_DATA_LENGTH + 1];
		
		if (hapstick->isConnected()){
			//read position from serial buffer
			size = hapstick->readSerialPort(input, MAX_DATA_LENGTH);
			if (size != 0) {
				input[size] = 0;

				int lastNewLine = 0;
				if (size > 0){ //if there is data to parse
					for (int index = 0; index < size - MAX_SIZE_OF_DATA; index++) { //find the 2nd to last new line character (assuming there are multiple position coordinates in the buffer)
						if (input[index] == '\n') {
							lastNewLine = index;
						}
					}
				}
				else{ //if no data available, return null
					return NULL;
				}
				int commaIndex;
				// look for comma
				for (int index = lastNewLine; index < size - MAX_SIZE_OF_DATA; index++) {
					if (input[index] == ',') {
						commaIndex = index;
					}
				}


				if (lastNewLine != 0){
					lastNewLine++;  //if multiple coordinates available, parse after '\n'
				}
				//Parse serial input into 2 char strings
				char* x_arr = &input[lastNewLine];
				char* y_arr = &input[commaIndex + 1];
				char foo[10];
				//Convert char arrays into floats
				//sscanf(x_arr, "%f[^,],%f", &angleX, &angleY);
				sscanf(x_arr, "%f,%f%s", &angleX, &angleY, foo);
				sscanf(y_arr, "%f", &angleY);
				this->prevX = angleX;
				this->prevY = angleY;
				retVal = 1;

			}
			else{ //readResult == 0
				angleX = this->prevX;
				angleY = this->prevY;
				retVal = 1;
			}
		}
		else{ //arduino not connected
			//angleX = 0;
			//angleY = 0;
		}
		return retVal;
	}; //end hapstickGetPosition

