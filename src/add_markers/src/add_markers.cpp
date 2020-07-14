#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>       /* fabs */


static const uint32_t shape = visualization_msgs::Marker::CUBE;
static const float pickup[3] = {2.0, 3.0, 1.0};       // pickup zone (x,y,w)
static const float dropoff[3] = {1.0, 0.0, 1.0};     // dropoff zone (x,y,w)
 
static const float eps[2] = {0.5, 0.3};   // epsilon for position and orientation

// Use static variable to save the state of at_pickup and at_dropoff
static bool at_pickup = false;
static bool at_dropoff = false;

bool reach(const float target[3], const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
  return std::abs(target[0] - msg->pose.pose.position.x) < eps[0] && std::abs(target[1] - msg->pose.pose.position.y) < eps[0] && std::abs(target[2] - msg->pose.pose.orientation.w) < eps[1];
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    at_pickup = reach(pickup, msg);
    at_dropoff = reach(dropoff, msg);
}

static visualization_msgs::Marker generate_markers(const float taget[], int n){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = n;
  marker.pose.position.x = taget[0];
  marker.pose.position.y = taget[1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = taget[2];
  marker.scale.x = 0.7;
  marker.scale.y = 0.7;
  marker.scale.z = 0.7;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  return marker;
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_sub = n.subscribe("amcl_pose", 1, amcl_pose_callback);

  bool pickup_done = false;
  bool dropoff_done = false;
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

  ROS_INFO("Publishing pick-up marker");
  marker_pub.publish(generate_markers(pickup, visualization_msgs::Marker::ADD));

  while (ros::ok()){
    while (!at_pickup){
      ros::spinOnce();
    }

    if (!pickup_done)
    {
      ROS_INFO("Publishing pick-up marker removal");
      marker_pub.publish(generate_markers(pickup, visualization_msgs::Marker::DELETE));
      pickup_done = true;
    }

    while (!at_dropoff)
      ros::spinOnce();
    if (!dropoff_done)
    {
      ROS_INFO("Publishing marker to be displayed");
      marker_pub.publish(generate_markers(dropoff, visualization_msgs::Marker::ADD));
      dropoff_done = true;
      ros::Duration(10.0).sleep();
    }
  }

}
