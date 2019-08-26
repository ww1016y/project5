#!/bin/bash

## deploy a turtlebot in my environment
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/myworld.world" &
sleep 10

## perform SLAM
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 2

## observe the map in rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2

## manually control the robot
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
