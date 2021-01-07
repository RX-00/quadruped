#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// actual quaternion components in a [w, x, y, z] format
#define OUTPUT_READABLE_QUATERNION

// Euler angles (in degrees) calc from quaternions coming from FIFO, WARN: can suffer from gimbal lock
//#define OUTPUT_READABLE_EULER

// yaw/pitch/roll angles (in degrees) calculated from the quaternions coming from the FIFO
// NOTE: also requires gravity vector calculations, WARN: ypr angles suffer from gimbal lock
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// mpu offsets
//int ax, ay, az, gx, gy, gz;
int ax = -2521;
int ay = 680;
int az = 1974;
int gx = 25;
int gy = 21;
int gz = 40;

// AD0 low = 0x68 (default for InvenSense eval and SparkFun breakout board)
// AD0 high = 0x69
bool ado = false;

double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup(ros::NodeHandle &nh, ros::Publisher &imu_pub_) {
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    if (!mpu.testConnection()){
      printf("Connection Failed -> killing node setup");
      exit(0);
    }

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here!
    mpu.setXGyroOffset(gx);
    mpu.setYGyroOffset(gy);
    mpu.setZGyroOffset(gz);
    mpu.setXAccelOffset(ax);
    mpu.setYAccelOffset(ay);
    mpu.setZAccelOffset(az);


    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

    // setup ros pub
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
    ROS_INFO("IMU SETUP COMPLETE");
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }

    else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        #endif
    }
}

void imu_pub(ros::NodeHandle &nh, ros::Publisher &imu_pub_, sensor_msgs::Imu &imu_data){
  // if the dmp programming fails then don't continue / do anything
  if (!dmpReady){
    ROS_INFO("DMP ready failed, terminating node...");
    return;
  }

  // get FIFO count
  fifoCount = mpu.getFIFOCount();

  // start filling out the imu data msg
  ros::Time now = ros::Time::now();
  imu_data.header.stamp = now;

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (fifoCount >= 42) {
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            //printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);

            // setup and fill imu data
            imu_data.orientation.x = q.x;
            imu_data.orientation.y = q.y;
            imu_data.orientation.z = q.z;
            imu_data.orientation.w = q.w;

            // NOTE: might need to add covariance factors depending on performance...
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            //printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
            // fill imu data (in rad/s (should be))
            imu_data.angular_velocity.x = ypr[2];
            imu_data.angular_velocity.y = ypr[1];
            imu_data.angular_velocity.z = ypr[0];
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);

            // By default, accel is in arbitrary units with a scale of 16384/1g.
            // Per http://www.ros.org/reps/rep-0103.html
            // and http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
            // should be in m/s^2.
            // 1g = 9.80665 m/s^2, so we go arbitrary -> g -> m/s^s
            imu_data.linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
            imu_data.linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
            imu_data.linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            //printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        #endif

            //printf("\n");

            // publish imu data
            imu_pub_.publish(imu_data);
    }
}


int main(int argc, char** argv) {
  // initiate and name ros node
  ros::init(argc, argv, "imu_pub_node");
  ros::NodeHandle nh;

  // initialize imu publisher
  ros::Publisher imu_pub_;

  // setup MPU6050 and ros publisher
  setup(nh, imu_pub_);

  // set ros refresh rate for the main loop
  //ros::Rate rate(1.0/node.getNodeConfig().dt);
  ros::Rate rate(20); // NOTE: I believe the default MPU6050 sample rate is 18Hz so 20Hz should be enough?

  //bool debug_mode = node.getNodeConfig().debug_mode;
  bool debug_mode = false;
  //usleep(1000); // give the imu some time to get ready

  sensor_msgs::Imu imu_data;
  ros::Time begin;

  // main loop will run indefinitely unless there is an interrupt call detected
  while (ros::ok()){

    if (debug_mode){
      begin = ros::Time::now();
    }

    //PUBLISH IMU DATA NOTE: seems like there's something wrong in this function
    imu_pub(nh, imu_pub_, imu_data);

    ros::spinOnce();
    rate.sleep();

    if (debug_mode){
      std::cout << (ros::Time::now() - begin) << std::endl;
    }
  }


  return 0;
}
