#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#define PICKUP_X 5.0
#define PICKUP_Y -8.5
#define PICKUP_Z 0.0

#define DROPOFF_X 0.0
#define DROPOFF_Y -8.5
#define DROPOFF_Z 0.0


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //********************
  // General marker frame setup
  // *******
 
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

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

  // ************
  // For pickup 
  // ************

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header


  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.position.z = PICKUP_Z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  //*****
  // ADD PICKUP MARKER

  // Set the marker action to add marker.
  marker.action = visualization_msgs::Marker::ADD;

  // Publish the marker

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

  ROS_INFO("Created pickup marker");
  marker_pub.publish(marker);

  // Wait 5 seconds for the pickup
  ros::Duration(5).sleep(); // sleep for 5 seconds

  //*****
  // REMOVE PICKUP MARKER

  // Set the marker action to delete marker.
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETE;  

  marker_pub.publish(marker);
  ROS_INFO("Deleted pickup marker");

  // Wait 5 seconds for the pickup
  ros::Duration(5).sleep(); // sleep for 5 seconds

  // ************
  // For dropoff 
  // ************

  // Set the marker action to delete marker.
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD; 

  marker.pose.position.x = DROPOFF_X;
  marker.pose.position.y = DROPOFF_Y;
  marker.pose.position.z = DROPOFF_Z;

  marker_pub.publish(marker);
  ROS_INFO("Created dropoff marker");


  return 0;
  
}
