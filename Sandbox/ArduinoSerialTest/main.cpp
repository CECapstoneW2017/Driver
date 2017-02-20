#include "SerialPort.h"
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace std;

float* getPosition(char*, int);

char* portName = "\\\\.\\COM6";
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
		/*
		//Test two: receive pos coordinates data from the arduino every 200ms, read the data every 1s 
		cout << "Write \"start\": \n";
		getline(cin, input_string);
		char *c_string = new char[input_string.size() + 1];
		int c_string_sz = input_string.size() + 1;

		std::copy(input_string.begin(), input_string.end(), c_string);
		c_string[input_string.size()] = '\n';
		sendResult = arduino->writeSerialPort(c_string, c_string_sz); //CHANGED THE 2nd INPUT
		for (char i = 0; i < 10; i++){
			Sleep(1000);
			readResult = arduino->readSerialPort(receivedString, MAX_DATA_LENGTH);
			if (readResult != 0) {
				receivedString[readResult] = 0;
				cout << receivedString;
			}
			else {
				cout << "Error: No data Received\n";
			}
		}*/
		
		//Test one: write something to the terminal and the arduino will echo
		cout << "Write something: \n";
		getline(cin, input_string);				// get input
		char *c_string = new char[input_string.size() + 1];		// declare new char array
		int c_string_sz = input_string.size() + 1;				// define size of array

		std::copy(input_string.begin(), input_string.end(), c_string);	// copy input into char array
		c_string[input_string.size()] = '\n';							// end char array 
		
		sendResult = arduino->writeSerialPort(c_string, c_string_sz); //CHANGED THE 2nd INPUT

		if (sendResult) {
			//while (sendResult) {	// prints all data send from Arduino, even if it overflows the buffer
				readResult = arduino->readSerialPort(receivedString, MAX_DATA_LENGTH);


				if (readResult != 0) {
					receivedString[readResult] = 0;
					float* position = getPosition(receivedString, readResult);
					char str_out[20];
					sprintf(str_out, "%.2f,%.2f", position[0], position[1]);
					cout << str_out;
					//cout << "\n";
					//cout << position[1];
				}
				else { cout << "Error: No data Received\n"; }	// readResult == 0=
			//}						// prints all data send from Arduino, even if it overflows the buffer
		}
		else { cout << "Error: No data sent\n"; }			// !sendResult

		delete[] c_string;	// discard old data
		
	}
}

float* getPosition(char* input, int size) {
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

	sscanf(x_arr, "%f", &pos[0]);
	sscanf(y_arr, "%f", &pos[1]);

	return pos;
}