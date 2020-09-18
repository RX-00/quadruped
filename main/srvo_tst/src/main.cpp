/* Test program for sending and receiving of data between the computer and microcontroller
 * also will parse and interpret the data RX. from the microcontroller
 *
 * NOTE: This is using UART Serial protocol at a baud rate of 115200 with default
 *       8N1 format, which is 8 data bits, no parity, 1 stop bit
 *
 * NOTE: data form for motor speed:
 *                 l(+/-)###r(+/-)###
 */

#include "Com.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>
#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "rs232.h"              // serial TX RX
#include "RPMSerialInterface.h" // servos
#include "Utils.h"              // servo utility class for sleep & time methods

#define BUF_SIZE   128
#define PORT_NUM   24 // NOTE: using ttyACM0 -> port number 24
#define BAUDRATE   115200
#define TX_TIME    1500 // usec -> 1.5 sec for stable condition
#define RX_TIME    150  // waits for reply 100ms
#define CYCLE_TIME 150  // sleep for 100ms

//NOTE: modified the RPM library to allow for greater servo range
#define SRVO_MAX 16000
#define SRVO_MIN 1000 // NOTE: I don't think this is the right wave val for "true" min
#define SRVO_MID 6000
#define SRVO_MAX_KNEE   6000
#define SRVO_MIN_KNEE   1000
#define SRVO_R_MAX_KNEE SRVO_MIN_KNEE
#define SRVO_R_MIN_KNEE SRVO_MAX_KNEE
#define SRVO_L_MAX_KNEE SRVO_MAX_KNEE
#define SRVO_L_MIN_KNEE SRVO_MIN_KNEE
#define LEFT_FRONT_SHOULDER_LAT 1
#define RIGHT_FRONT_KNEE 2
#define RIGHT_BACK_KNEE  3
#define LEFT_FRONT_KNEE 4
#define LEFT_BACK_KNEE  5

//TODO: test that class for delaying the serial interface

// default crouching position for the robot
void default_pos(RPM::SerialInterface *serialInterface){
  std::cout << "Moving into default position in 1 seconds..." << std::endl;
  Utils::sleep(1000);
  /*serialInterface -> setTargetCP(LEFT_HIP, 3000);  // LOWER  (back)
  serialInterface -> setTargetCP(RIGHT_HIP, 9000); // HIGHER (back)
  serialInterface -> setTargetCP(LEFT_KNEE, SRVO_MIN);
  serialInterface -> setTargetCP(RIGHT_KNEE, SRVO_MAX);*/
  Utils::sleep(1500);
  exit(1); // NOTE: THIS IS FOR MEGA_IP_TESTING
}

void servo_test(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  std::cout << "Testing servo number: " << 11 << std::endl;
  for (int i = 0; i < 5; i++){
    std::cout << "min position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MIN);
    Utils::sleep(1000);
    std::cout << "middle position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, 6000);
    Utils::sleep(1000);
    std::cout << "max position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MAX);
    Utils::sleep(1000);
  }
  exit(1);
}

// function to test device over serial w/ sinusoidal signals
void sinusoid_signal(RPM::SerialInterface *serialInterface, unsigned char channelNumber){
  // Generate a sinusoid signal to send to the PololuInterface
  std::cout << "Sending sinusoidal signal to device to test device..." << std::endl;
	const float pi = 3.141592653589793f;
	const unsigned int channelMinValue = SRVO_MIN;
	const unsigned int channelMaxValue = SRVO_MAX;
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
  printf("\n");
}

// function to create serial interface for the maestro servo controller
RPM::SerialInterface * serialInterfaceInit(unsigned char deviceNumber, unsigned char channelNumber, std::string portName){
  // create the interface for the maestro
  std::cout << "Serial interface init..." << std::endl;
	unsigned int baudRate = 115200;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ){
    printf("Failed to create serial interface. %s\n", errorMessage.c_str());
    std::cout << "Terminating program..." << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << "Serial interface initiated\n" << std::endl;
  return serialInterface;
}


int main(int argc, char** argv){
  // Serial servo interface
  unsigned char deviceNumber = 12;
	unsigned char channelNumber = 11;
  std::string portName = "/dev/ttyACM0";
  RPM::SerialInterface *servosInterface = serialInterfaceInit(deviceNumber, channelNumber, portName);

  servosInterface -> SerialInterface::mMinChannelValue = SRVO_MIN;
  servosInterface -> SerialInterface::mMaxChannelValue = SRVO_MAX;

  for (int i = 0; i < 3; i++){
    std::cout << "servos min pos" << std::endl;
    servosInterface -> setTargetCP(RIGHT_FRONT_KNEE, SRVO_R_MIN_KNEE);
    servosInterface -> setTargetCP(RIGHT_BACK_KNEE, SRVO_R_MIN_KNEE);
    servosInterface -> setTargetCP(LEFT_FRONT_KNEE, SRVO_L_MIN_KNEE);
    servosInterface -> setTargetCP(LEFT_BACK_KNEE, SRVO_L_MIN_KNEE);
    servosInterface -> setTargetCP(LEFT_FRONT_SHOULDER_LAT, SRVO_MIN);
    Utils::sleep(1000);

    std::cout << "servos max pos" << std::endl;
    servosInterface -> setTargetCP(RIGHT_FRONT_KNEE, SRVO_R_MAX_KNEE);
    servosInterface -> setTargetCP(RIGHT_BACK_KNEE, SRVO_R_MAX_KNEE);
    servosInterface -> setTargetCP(LEFT_FRONT_KNEE, SRVO_L_MAX_KNEE);
    servosInterface -> setTargetCP(LEFT_BACK_KNEE, SRVO_L_MAX_KNEE);
    servosInterface -> setTargetCP(LEFT_FRONT_SHOULDER_LAT, SRVO_MAX);
    Utils::sleep(1000);
  }

  std::cout << "servos mid pos " << SRVO_MID << std::endl;
  servosInterface -> setTargetCP(LEFT_FRONT_SHOULDER_LAT, SRVO_MID);
  Utils::sleep(1000);

  delete servosInterface;
  servosInterface = NULL;

  return 0;
}
