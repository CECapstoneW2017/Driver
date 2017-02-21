#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "SerialPort.h"

using namespace std;

char output[MAX_DATA_LENGTH];

char *port_name = "\\\\.\\COM6";

char incomingData[MAX_DATA_LENGTH];

int main()
{
  SerialPort arduino(port_name);
  if (arduino.isConnected()) cout << "Connection Established" << endl;
  else cout << "ERROR, check port name\n";

  while (arduino.isConnected()){
    cout << "Write something: \n";
    std::string input_string;
    getline(cin, input_string);
    char *c_string = new char[input_string.size() + 1];
    std::copy(input_string.begin(), input_string.end(), c_string);
    c_string[input_string.size()] = '\n';
    arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);
    arduino.readSerialPort(output, MAX_DATA_LENGTH);
    puts(output);
    delete[] c_string;
  }
}