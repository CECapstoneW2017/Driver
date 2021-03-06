#include "SerialPort.h"
#include <iostream>
#include <string>

using namespace std;

char* portName = "\\\\.\\COM3";
char receivedString[MAX_DATA_LENGTH + 1];

SerialPort *arduino;

int main(void)
{
	arduino = new SerialPort(portName);
	cout << "is connected: " << arduino->isConnected() << std::endl;
	string input_string;
	int readResult = 0;
	bool sendResult = FALSE;

	while (arduino->isConnected()) {

		//cout << "Write something: \n";
		//getline(cin, input_string);
		//char *c_string = new char[input_string.size() + 1];
		//int c_string_sz = input_string.size() + 1;

		//std::copy(input_string.begin(), input_string.end(), c_string);
		//c_string[input_string.size()] = '\n';

		//sendResult = arduino->writeSerialPort(c_string, c_string_sz); //CHANGED THE 2nd INPUT

		//if (sendResult) {
			readResult = arduino->readSerialPort(receivedString, MAX_DATA_LENGTH);

			if (readResult != 0) {
				receivedString[readResult] = 0;
				cout << receivedString;
			}
			else {
				//cout << "Error: No data Received\n";
			}
		//}
		//else {
		//	cout << "Error: No data sent\n";
		//}

		//delete[] c_string;
	}
}