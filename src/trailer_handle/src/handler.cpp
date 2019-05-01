/** \file
* this is the highlevel control code for the trailer
* developed by: Hongxu Guo
* Date: 5-1-19
*
* version: 1.0
*/


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Char.h>
#include <vector>


void curPos_cb(const std_msgs::UInt16MultiArray::ConstPtr& array);
void armProfile_cb(const std_msgs::UInt16::ConstPtr& idx);
void armStep_cb(const std_msgs::UInt16::ConstPtr& idx);
void drillState_cb(const std_msgs::UInt8::ConstPtr& state);
void servoState_cb(const std_msgs::UInt8::ConstPtr& state);
void mainTasks();


uint16_t curPos[6];
uint16_t goalPos[6];
uint16_t armProfile_idx = 0;
uint16_t armStep_idx = 0;
uint8_t drillState = 0;
uint8_t servoState = 0;

std_msgs::UInt16MultiArray goalPos_ros;
std_msgs::Char command_ros;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "handler");
  /* start cyclic part */
  ros::NodeHandle n;
  ros::Publisher goalPos_pub = n.advertise<std_msgs::UInt16MultiArray>("armGoalPos", 1000);
  ros::Publisher command_pub = n.advertise<std_msgs::Char>("trailerCommand",1000);
  ros::Subscriber curPos_sub = n.subscribe("armCurPos", 100, curPos_cb);
  ros::Subscriber armProfile_sub = n.subscribe("armProfile_idx", 100, armProfile_cb);
  ros::Subscriber armStep_sub = n.subscribe("armStep_idx", 100, armStep_cb);
  ros::Subscriber drillState_sub = n.subscribe("drillState", 100, drillState_cb);
  ros::Subscriber servoState_sub = n.subscribe("servoState", 100, servoState_cb);

  ros::Rate loop_rate(100);

  goalPos[0] = 1111;
  goalPos[1] = 2222;
  goalPos[2] = 3333;
  goalPos[3] = 4444;
  goalPos[4] = 5555;
  goalPos[5] = 6666;
  command_ros.data = 'p';


  while(ros::ok())
  {
    mainTasks();
    command_pub.publish(command_ros);
    goalPos_pub.publish(goalPos_ros);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
void mainTasks()
{
  goalPos_ros.data.clear();
  for (int i = 0; i < 6; i++)
  {
    goalPos_ros.data.push_back(goalPos[i]);
  }

}

void curPos_cb(const std_msgs::UInt16MultiArray::ConstPtr& array)
{
  for (int i = 0; i < 6; i++)
  {
    curPos[i] = (int8_t)(array->data[i]);
  }
}
void armProfile_cb(const std_msgs::UInt16::ConstPtr& idx)
{
  armProfile_idx = idx->data;
  ROS_INFO("armProfile: %d", armProfile_idx);
}
void armStep_cb(const std_msgs::UInt16::ConstPtr& idx)
{
  armStep_idx = idx->data;
  ROS_INFO("armStep: %d", armStep_idx);
}
void drillState_cb(const std_msgs::UInt8::ConstPtr& state)
{
  drillState = state->data;
  ROS_INFO("drillState: %d", drillState);
}
void servoState_cb(const std_msgs::UInt8::ConstPtr& state)
{
  servoState = state->data;
  ROS_INFO("servoState: %d", servoState);
}
