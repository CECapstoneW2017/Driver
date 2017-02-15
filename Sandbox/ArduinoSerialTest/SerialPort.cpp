/**
* SerialPort.cpp
* Purpose: Create and manage asynchronous serial communications link with an Arduino
*
* @author Brandon Berryman
* @version 1.1 2/15/2017
*/
#include "SerialPort.h"

/**
* Returns active serial port connection
*
* @param *portName The string representation of the virtual COM port
* @returns connected serial port connection
*/
SerialPort::SerialPort(char *portName)
{
	this->connected = false;

	this->handler = CreateFileA(static_cast<LPCSTR>(portName),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	if (this->handler == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
		}
		else
		{
			printf("ERROR!!!");
		}
	}
	else {
		DCB dcbSerialParameters = { 0 };

		if (!GetCommState(this->handler, &dcbSerialParameters)) {
			printf("Failed to get current serial parameters");
		}
		else {
			dcbSerialParameters.BaudRate = CBR_9600;
			dcbSerialParameters.ByteSize = 8;
			dcbSerialParameters.StopBits = ONESTOPBIT;
			dcbSerialParameters.Parity = NOPARITY;
			dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

			if (!SetCommState(handler, &dcbSerialParameters))
			{
				printf("ALERT: could not set Serial port parameters\n");
			}
			else {
				this->connected = true;
				PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
	}
}

/**
* Closes serial port connection
*
* @param NA
* @returns NA
*/
SerialPort::~SerialPort()
{
	if (this->connected) {
		this->connected = false;
		CloseHandle(this->handler);
	}
}

/**
* Reads serial port data
*
* @param *buffer Serial port data buffer
* @param buf_size Size of serial port data buffer
* @returns Data recieved on serial port
*/
int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesRead;
	unsigned int toRead;

	ClearCommError(this->handler, &this->errors, &this->status);

	if (this->status.cbInQue > 0) {
		if (this->status.cbInQue > buf_size) {
			toRead = buf_size;
		}
		else {
			toRead = this->status.cbInQue;
		}
	}

	if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL)) {
		return bytesRead;
	}

	return 0;
}

/**
* Writes data to serial port
*
* @param *buffer Serial port data buffer
* @param buf_size Size of serial port data buffer
* @returns Status of sent data
*/
bool SerialPort::writeSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesSend;

	if (!WriteFile(this->handler, (void*)buffer, buf_size, &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		
		return false;
	}
	else {
		return true;
	}
}

/**
* Checks status of serial port connection
*
* @param NA
* @returns Status of serial port connection
*/
bool SerialPort::isConnected()
{
	return this->connected;
}