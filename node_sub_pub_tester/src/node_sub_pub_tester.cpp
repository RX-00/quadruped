#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"


void jointTrajCallback(const trajectory_msgs::JointTrajectory &msg){
  trajectory_msgs::JointTrajectory traj = msg;
  for (int i = 0; i < 12; i++)
    std::cout << traj.joint_names[i] << std::endl;
  //std::cout << traj.points[0] << std::endl;
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "node_sub_pub_tester");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  ros::Time begin;

  // joint angles subscriber for champ system
  ros::Subscriber joint_angles_sub_ = nh.subscribe("/joint_group_position_controller/command", 1, jointTrajCallback);
  // Joint state publisher for champ system
  ros::Publisher joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states",1);
  // Imu publisher for champ system
  ros::Publisher imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data",1);

  ros::Rate loop_rate(10); // 10 hertz loop
  int count = 0;

  while(ros::ok()){
    //ROS_INFO("test node loop: [%d]", count);
    //sensor_msgs:: = joint_state_msg;
    //sensor_msgs:: = imu_msg;

    //joint_state_pub_.publish(joint_state_msg);
    //imu_pub_.publish(imu_msg);

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
