#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set Phase (modified)
  uint8_t phase = 0;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "cube_marker";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
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

    switch (phase)
    {
        case 1:
            ROS_INFO("phase 1 : Publish the marker at the pickup zone and wait 5 seconds");
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 5;
            marker.pose.position.y = 5;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker_pub.publish(marker);
            sleep(5);
        break;
        
        case 2:
            ROS_INFO("phase 2 : Hide the marker and wait 5 seconds");
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            sleep(5);
        break;
        
        case 3:
            ROS_INFO("phase 3 : Publish the marker at the drop off zone");
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 1;
            marker.pose.position.y = -3;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker_pub.publish(marker);
            sleep(5);
            break;
    }
    phase++;

    if(phase>3)
        return 0;
    r.sleep();
  }
}
