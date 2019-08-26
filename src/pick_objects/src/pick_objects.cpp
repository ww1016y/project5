#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_1;
  move_base_msgs::MoveBaseGoal goal_2;

  // set up the frame parameters
  goal_1.target_pose.header.frame_id = "map";
  goal_1.target_pose.header.stamp = ros::Time::now();
  goal_2.target_pose.header.frame_id = "map";
  goal_2.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach each goals
  goal_1.target_pose.pose.position.x = 5.0;
  goal_1.target_pose.pose.position.y = 5.0;
  goal_1.target_pose.pose.orientation.w = 1.0;

  goal_2.target_pose.pose.position.x = 1.0;
  goal_2.target_pose.pose.position.y = -3.0;
  goal_2.target_pose.pose.orientation.w = -1.0;


  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is traveling to the pickup zone");
  ac.sendGoal(goal_1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot picked up the virtual object");
    sleep(5);    //Wait for 5 seconds
  }
  else
  {
    ROS_INFO("The robot failed to reach pickup zone");
    return 1;
  }

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is traveling to the drop off zone");
  ac.sendGoal(goal_2);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot dropped the virtual object");
  }
  else
  {
    ROS_INFO("The robot failed to reach drop off zone");
    return 1;
  }
  while(true)

  return 0;
}
