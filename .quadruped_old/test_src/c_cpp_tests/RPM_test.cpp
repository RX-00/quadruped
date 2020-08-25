/*
 * This program is to test how fast the servos will
 * respond with this library vs. the python
 * implementation.
 */

#include <iostream>
#include <stdio.h>
#include <cmath>

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

#include "RPMSerialInterface.h"

using namespace std;

int main(int argc, char** argv){
  // create the interface for the maestro
  cout << "serial init" << endl;
  unsigned char deviceNumber = 12;
	unsigned char channelNumber = 2;

	std::string portName = "/dev/ttyACM0";

	unsigned int baudRate = 115200;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ){
      printf("Failed to create serial interface. %s\n", errorMessage.c_str());
      return -1;
  }

  // set the value of channel 6 to 6000 quarter of microseconds (i.e. 1.5 milliseconds)
  serialInterface -> setTargetCP(6, 6000);
  Utils:sleep(1000);

  // delete the interface
  cout << "deleting serial interface" << endl;
  delete serialInterface;
  serialInterface = NULL;

  return 0;
}
