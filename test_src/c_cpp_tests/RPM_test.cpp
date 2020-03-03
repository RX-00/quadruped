/*
 * This program is to test how fast the servos will
 * respond with this library vs. the python
 * implementation.
 */

#include <stdio.h>
#include <cmath>

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

#include "RPMSerialInterface.h"

using namespace std

int main(int argc, char** argv){
  // create the interface for the maestro
  RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface("", 9600);
  if (!serialInterface -> isOpen()){
    cout << "ERROR: serial interface was not open, could not connect, terminating..." << endl;
    return -1;
  }

  // set the value of channel 6 to 6000 quarter of microseconds (i.e. 1.5 milliseconds)
  serialInterface -> setTargetCP(6, 6000);

  // delete the interface
  delete serialInterface;

  return 0;
}
