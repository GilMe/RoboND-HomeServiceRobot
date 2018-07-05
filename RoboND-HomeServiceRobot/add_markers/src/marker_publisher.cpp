#include <ros/ros.h>

#include "std_msgs/String.h"

// Note: This node works in conjunction with add marker node and was added for the benefit of add_marker.sh file to work


int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle n;
  ros::Rate r(1);

  // create a goal publisher topic
  ROS_INFO("Opening publisher");
  ros::Publisher goal_status_publisher = n.advertise<std_msgs::String>("goal_status", 4);

  std_msgs::String goal_status_msg;

  //wait 1 seconds
  ros::Duration(1).sleep(); // sleep for 1 seconds

  // make marker appear in pickup location
  ROS_INFO("Robot picking up virtual object");
  goal_status_msg.data = "Moving towards pickup";
  goal_status_publisher.publish(goal_status_msg);

  //wait 5 seconds for pickup
  ros::Duration(5).sleep(); // sleep for 5 seconds

  //make marker disappear from pickup location
  ROS_INFO("Robot picked up virtual object");
  goal_status_msg.data = "Reached pickup";
  goal_status_publisher.publish(goal_status_msg);

  //wait 5 seconds for pickup
  ros::Duration(5).sleep(); // sleep for 5 seconds

  //make arker appear in drop off location
  ROS_INFO("Robot dropped off virtual object");
  goal_status_msg.data = "Reached dropoff";
  goal_status_publisher.publish(goal_status_msg);

  //wait 5 seconds
  ros::Duration(5).sleep(); // sleep for 1 seconds

  return 0;  
}
