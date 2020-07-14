#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

static const uint32_t shape = visualization_msgs::Marker::CUBE;
static const float pickup[3] = {2.0, 3.0, 1.0};
static const float dropoff[3] = {-2.0, 2.0, 1.0};

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
  ros::Rate r(10);
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

  // Add the pickup marker
  marker_pub.publish(generate_markers(pickup, visualization_msgs::Marker::ADD));
  // pause for 5 seconds
  ros::Duration(5.0).sleep();
  // delete for 5 seconds
  marker_pub.publish(generate_markers(pickup, visualization_msgs::Marker::DELETE));
  // pause for 5 seconds
  ros::Duration(5.0).sleep();
  // Add the dropoff marker
  while (ros::ok()){
    marker_pub.publish(generate_markers(dropoff, visualization_msgs::Marker::ADD));
    r.sleep();
  }
}
