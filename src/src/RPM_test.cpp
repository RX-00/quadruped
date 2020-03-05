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


// A utility class to provide cross-platform sleep and simple time methods
class Utils
{
public:
	static void sleep( unsigned int _Milliseconds );
	static unsigned long long int getTickFrequency();
	static unsigned long long int getTimeAsTicks();
	static unsigned int getTimeAsMilliseconds();

private:
	static unsigned long long int mInitialTickCount;
};


// Utils class implementation
void Utils::sleep( unsigned int _Milliseconds )
{
#if _WIN32
	::Sleep( _Milliseconds );
#else
	struct timespec l_TimeSpec;
	l_TimeSpec.tv_sec = _Milliseconds / 1000;
	l_TimeSpec.tv_nsec = (_Milliseconds % 1000) * 1000000;
	struct timespec l_Ret;
	nanosleep(&l_TimeSpec,&l_Ret);
#endif
}

unsigned long long int Utils::getTickFrequency()
{
#if _WIN32
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);
	return frequency.QuadPart;
#else
	// The gettimeofday function returns the time in microseconds. So it's frequency is 1,000,000.
	return 1000000;
#endif
}

unsigned long long int Utils::getTimeAsTicks()
{
	unsigned long long int tickCount;
#if _WIN32
	LARGE_INTEGER l;
	QueryPerformanceCounter(&l);
	tickCount = l.QuadPart;
#else
	struct timeval p;
	gettimeofday(&p, NULL);	// Gets the time since the Epoch (00:00:00 UTC, January 1, 1970) in sec, and microsec
	tickCount = (p.tv_sec * 1000LL * 1000LL) + p.tv_usec;
#endif
	if ( mInitialTickCount==0xffffffffffffffffUL )
		mInitialTickCount = tickCount;
	tickCount -= mInitialTickCount;
	return tickCount;
}

unsigned int Utils::getTimeAsMilliseconds()
{
	unsigned int millecondsTime = static_cast<unsigned int>( (getTimeAsTicks() * 1000) / getTickFrequency() );
	return millecondsTime;
}

unsigned long long int Utils::mInitialTickCount = 0xffffffffffffffffUL;



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
  cout << "Serial interface created" << endl;

  // Generate a sinusoid signal to send to the PololuInterface
	const float pi = 3.141592653589793f;
	const unsigned int channelMinValue = 4000;
	const unsigned int channelMaxValue = 8000;
	const unsigned int channelValueRange = channelMaxValue - channelMinValue;
	const unsigned int signalPeriodInMs = 2000;
	unsigned int time0 = Utils::getTimeAsMilliseconds();
	unsigned int timeSinceStart = 0;
	while ( timeSinceStart < 5000 ){
      float k = sin( (pi*2)/signalPeriodInMs * timeSinceStart ) * (float)(channelValueRange/2);
      float channelValue = (float)channelMinValue + (float)channelValueRange/2 + k;
      printf("\rchannelValue=%d", (unsigned int)channelValue );
      serialInterface->setTargetCP( channelNumber, (unsigned short)channelValue );
      timeSinceStart = Utils::getTimeAsMilliseconds() - time0;
      Utils::sleep(5);
  }

  // test servo port 6
  for (int i = 0; i < 11; i++){
    printf("Testing port 6 at pos: %d \n", i * 1000);
    serialInterface -> setTargetCP(6, i * 1000);
    Utils::sleep(1000);
  }

  // delete the interface
  cout << "deleting serial interface" << endl;
  delete serialInterface;
  serialInterface = NULL;

  return 0;
}
