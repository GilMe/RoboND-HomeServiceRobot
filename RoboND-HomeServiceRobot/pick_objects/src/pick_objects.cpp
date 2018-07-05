#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "std_msgs/String.h"

#define PICKUP_X 5.0
#define PICKUP_Y -8.5
#define PICKUP_O 1.0

#define DROPOFF_X 0.0
#define DROPOFF_Y -8.5
#define DROPOFF_O 2.0


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // create a goal publisher topic
  ros::Publisher goal_status_publisher = n.advertise<std_msgs::String>("goal_status", 4);;

  //add node handle and tell the action client that we want to spin a thread by default
  MoveBaseClient ac(n,"move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  std_msgs::String goal_status_msg;

  //*********************
  // For pickup
  //*************

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = PICKUP_X;
  goal.target_pose.pose.position.y = PICKUP_Y;
  goal.target_pose.pose.orientation.w = PICKUP_O;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // publish status msg
  goal_status_msg.data = "Moving towards pickup";
  goal_status_publisher.publish(goal_status_msg);

  ROS_INFO("Robot is travelling to the pickup zone");
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot reached pickup zone");
    ROS_INFO("Robot picking up virtual object");

    // publish status msg
    goal_status_msg.data = "Reached pickup";
    goal_status_publisher.publish(goal_status_msg);

    //wait 5 seconds for pickup
    ros::Duration(5).sleep(); // sleep for 5 seconds
    ROS_INFO("Robot picked up virtual object");

  }
  else
    ROS_ERROR("The base failed to reach pickup zone for some reason");

 //*********************
  // For dropoff
  //*************

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = DROPOFF_X;
  goal.target_pose.pose.position.y = DROPOFF_Y;
  goal.target_pose.pose.orientation.w = DROPOFF_O;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);


  // publish status msg
  goal_status_msg.data = "Moving towards dropoff";
  goal_status_publisher.publish(goal_status_msg);

  ROS_INFO("Robot is travelling to the dropoff zone");
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    // publish status msg
    goal_status_msg.data = "Reached dropoff";
    goal_status_publisher.publish(goal_status_msg);

    ROS_INFO("Robot dropped off virtual object");
  }
  else
    ROS_ERROR("The base failed to reach dropoff zone for some reason");

  
  return 0;
}

