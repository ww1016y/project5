#!/bin/bash

## deploy a turtlebot in my environment
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/myworld.world" &
sleep 10

## localize the turtlebot
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/map.yaml" &
sleep 2

## observe the map in rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2

## pick up virtual objects
xterm -e "rosrun pick_objects pick_objects_node"
sleep 2
