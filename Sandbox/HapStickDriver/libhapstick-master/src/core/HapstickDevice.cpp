/***
 * @file FalconDevice.cpp
 * @brief FalconDevice class, glue class for falcon components
 * @author Kyle Machulis (kyle@nonpolynomial.com)
 * @copyright (c) 2007-2009 Nonpolynomial Labs/Kyle Machulis
 * @license BSD License
 *
 * Project info at http://libnifalcon.nonpolynomial.com/
 *
 */

#include "core/HapstickDevice.h"
#include "comm/SerialPort.h"
#include <iostream>

namespace libhapstick 
{
	//variables to keep track of previous position coordinates 
	private double prevX = 0;
	private double prevY = 0;

    FalconDevice::FalconDevice() :
		m_errorCount(0),
		INIT_LOGGER("FalconDevice")
	{
        m_forceVec[0] = 0.0;
        m_forceVec[1] = 0.0;
        m_forceVec[2] = 0.0;
        m_position[0] = 0.0;
        m_position[1] = 0.0;
        m_position[2] = 0.0;

	}

    FalconDevice::~FalconDevice()
	{
    close();
	}
	
	/**
	* Return the number of connected Hapstick devices
	*
	* @param & count: number of devices
	* @returns True if no errors occured
	*/
    bool FalconDevice::getDeviceCount(unsigned int& count)
	{
		//Find a way to determine how many arduinos are connected.
		return m_falconComm->getDeviceCount(count);
	}

	/**
	* Open connection with Hapstick device
	*
	* @param index: index of the device to open a connection with
	* @returns True if connection established successfully
	*/
    bool FalconDevice::open(unsigned int index)
	{
		bool bHapstickConnected = false;
		char* portName = "\\\\.\\COM6"; //TODO: determine this programatically

		//Create instance of Serial Port
		hapstick = new SerialPort(portName);
		
		//Check if arduino is connected
		if (hapstick->isConnected()){
			bHapstickConnected = true;
		}
		
		return bHapstickConnected;
	}

	/**
	* Closes a connection with Hapstick device
	*
	* @param none
	* @returns void
	*/
	void FalconDevice::close()
    {
		//Do something (delete instance of SerialPort or call destructor?)
		hapstick->~SerialPort;
	}

	/**
	* Gets position angles from the arduino
	*
	* @param angleX: pointer to X angle pos, angleY: pointer to Y angle pos
	* @returns error: 0 if no error, -1 if arduino is not connected
	*/
	int FalconDevice::hapstickGetPosition(double* angleX, double* angleY) //add device id later
	{

		if (arduino->isConnected()){
			//read position from serial buffer
			readResult = arduino->readSerialPort(receivedString, MAX_DATA_LENGTH);
			if (readResult != 0) {
				receivedString[readResult] = 0;
				
				int lastNewLine = 0;
				if (size > 0){ //if there is data to parse
					for (int index = 0; index < size - 1; index++) { //find the 2nd to last new line character (assuming there are multiple position coordinates in the buffer)
						if (input[index] == '/n') {
							lastNewLine = index;
						}
					}
				}
				else{ //if no data available, return null
					return NULL;
				}
				int commaIndex;
				// look for comma
				for (int index = lastNewLine; index < size - 1; index++) {
					if (input[index] == ',') {
						commaIndex = index;
					}
				}


				if (lastNewLine != 0){
					lastNewLine++;  //if multiple coordinates available, parse after '\n'
				}
				char* x_arr = &input[lastNewLine];
				char* y_arr = &input[commaIndex + 1];

				float pos[2];

				sscanf(x_arr, "%f", &angleX);
				sscanf(y_arr, "%f", &angleY);
				prevX = *angleX;
				prevY = *angleY;

			}
			else{ //readResult == 0
				&angleX = prevX;
				&angleY = prevY;
			}
		}
		else{ //arduino not connected
			&angleX = 0;
			&angleY = 0;
		}
		return NULL;
	}
	/*bool FalconDevice::setFirmwareFile(const std::string& filename)
    {
		if(m_falconFirmware == nullptr)
		{
			m_errorCode = FALCON_DEVICE_NO_FIRMWARE_SET;
			return false;
		}
		return m_falconFirmware->setFirmwareFile(filename);
	}

	bool FalconDevice::loadFirmware(unsigned int retries, bool skip_checksum)
	{
		if(m_falconFirmware == nullptr)
		{
			m_errorCode = FALCON_DEVICE_NO_FIRMWARE_SET;
			return false;
		}
		return m_falconFirmware->loadFirmware(retries, skip_checksum);
	}

	bool FalconDevice::loadFirmware(bool skip_checksum)
	{
		if(m_falconFirmware == nullptr)
		{
			m_errorCode = FALCON_DEVICE_NO_FIRMWARE_SET;
			return false;
		}
		return m_falconFirmware->loadFirmware(skip_checksum);
	}

	bool FalconDevice::isFirmwareLoaded()
	{
		if(m_falconFirmware == nullptr)
		{
			m_errorCode = FALCON_DEVICE_NO_FIRMWARE_SET;
			return false;
		}
		return m_falconFirmware->isFirmwareLoaded();
	}

	bool FalconDevice::runIOLoop(unsigned int exe_flags)
	{
		if(m_falconFirmware == nullptr)
		{
			m_errorCode = FALCON_DEVICE_NO_FIRMWARE_SET;
			return false;
		}
		if(m_falconKinematic != nullptr && (exe_flags & FALCON_LOOP_KINEMATIC))
		{
			std::array<int, 3> enc_vec;
			m_falconKinematic->getForces(m_position, m_forceVec, enc_vec);
			m_falconFirmware->setForces(enc_vec);
		}
		if(!m_falconFirmware->runIOLoop() && (exe_flags & FALCON_LOOP_FIRMWARE))
		{
			++m_errorCount;
			m_errorCode = m_falconFirmware->getErrorCode();
			return false;
		}
		if(m_falconGrip != nullptr && (exe_flags & FALCON_LOOP_GRIP))
		{
			if(!m_falconGrip->runGripLoop(m_falconFirmware->getGripInfoSize(), m_falconFirmware->getGripInfo()))
			{
				m_errorCode = m_falconGrip->getErrorCode();
				return false;
			}
		}
		if(m_falconKinematic != nullptr && (exe_flags & FALCON_LOOP_KINEMATIC))
		{
			std::array<int, 3> p = m_falconFirmware->getEncoderValues();
			if(!m_falconKinematic->getPosition(p, m_position))
			{
				++m_errorCount;
				m_errorCode = m_falconKinematic->getErrorCode();
				return false;
			}
		}
		return true;
	}*/

	float* getPosition(char* input, int size) {

	}
};
