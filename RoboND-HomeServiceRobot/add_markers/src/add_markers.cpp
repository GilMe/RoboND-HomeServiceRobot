#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"

#define PICKUP_X 5.0
#define PICKUP_Y -8.5
#define PICKUP_Z 0.0

#define DROPOFF_X 0.0
#define DROPOFF_Y -8.5
#define DROPOFF_Z 0.0

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
ros::Subscriber goal_status_sub;
bool done = false;

// The laser_callback function will be called each time a laser scan data is received
void status_callback(const std_msgs::String::ConstPtr& scan_msg)
{
  //getting the msg that was published
  std_msgs::String goal_status_msg = *scan_msg;
  ROS_INFO("Inside callback. Got goal status update");


  if(goal_status_msg.data == "Moving towards pickup"){
    ROS_INFO("Created pickup marker");
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = PICKUP_X;
    marker.pose.position.y = PICKUP_Y;
    marker.pose.position.z = PICKUP_Z; 
  }
  else if(goal_status_msg.data == "Reached pickup"){
    ROS_INFO("Deleted pickup marker");
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
  }
  else if(goal_status_msg.data == "Moving towards dropoff"){
    ROS_INFO("Moving towards dropoff - no marker generated");
  }
  else if(goal_status_msg.data == "Reached dropoff"){
    ROS_INFO("Created dropoff marker");
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = DROPOFF_X;
    marker.pose.position.y = DROPOFF_Y;
    marker.pose.position.z = DROPOFF_Z;
    done = true;
  }
  else
    return;

  
  marker_pub.publish(marker);
  return;

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  ros::Rate r(1);


  //********************
  // Setting up general marker frame message variables
  // *******
 
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the orientation
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.5f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  // ***********
  // Setting up pupblisher and subscriber
  // *****************

  ROS_INFO("Opening publisher");
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ROS_INFO("Opening subscriber");  
  // Subscribe to the /goal_status topic and call the laser_callback function
  goal_status_sub = n.subscribe("/goal_status", 10, status_callback);

  // Enter an infinite loop where the status_callback function will be called when new status messages arrive
  ros::Duration time_between_ros_wakeups(0.001);
  while (ros::ok()) {
      ros::spinOnce();
      time_between_ros_wakeups.sleep();
  }

  return 0;
  
}
