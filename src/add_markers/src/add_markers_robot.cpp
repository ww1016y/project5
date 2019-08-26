#include <ros/ros.h>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#define REACH_DIST 0.4

geometry_msgs::Pose2D current_pose;

void odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
  // initial set for robot rotation (90 deg)
  current_pose.x = msg->pose.pose.position.y;
  current_pose.y = msg->pose.pose.position.x*(-1);

  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll,pitch,yaw);
  current_pose.theta=yaw;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_robot_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  bool load_flag = false;
  float dist_x = 0;
  float dist_y = 0;

  // add odometry subscriber
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set Phase (modified)
  uint8_t phase = 0;
  uint8_t goal_num = 0;

  float goals[2][3] = {{5,5,1},{1,-3,-1}};

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

  ROS_INFO_ONCE("phase 1 : Publish the marker at the pickup zone");
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goals[goal_num][0];
  marker.pose.position.y = goals[goal_num][1];

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

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      // ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    dist_x = fabs(goals[goal_num][0]-current_pose.x);
    dist_y = fabs(goals[goal_num][1]-current_pose.y);
    
    if(!load_flag)
    {
      marker_pub.publish(marker);
      if((dist_x<=REACH_DIST)&&(dist_y<=REACH_DIST))
      {
          ROS_INFO_ONCE("phase 2 : Remove the marker");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        goal_num++;     
        load_flag = true;
      }
    }

    else
    {
      if((dist_x<=REACH_DIST)&&(dist_y<=REACH_DIST))
      {
        ROS_INFO_ONCE("phase 3 : Publish the marker at the drop off zone");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goals[goal_num][0];
        marker.pose.position.y = goals[goal_num][1];
        marker_pub.publish(marker);
        return 0;
      }
    }
    marker_pub.publish(marker);

    ros::spinOnce();
    r.sleep();
  }
}
