# project5 Home Service Robot Project
project5

The list of the official ROS packages 


gmapping: https://github.com/ros-perception/slam_gmapping 


turtlebot_teleop: https://github.com/turtlebot/turtlebot 


turtlebot_rviz_launchers: https://github.com/turtlebot/turtlebot_interactions 


turtlebot_gazebo: https://github.com/turtlebot/turtlebot_simulator


 .Project5                                
 
    ├── add_markers                           # add_markers package                   
    │   ├── launch                            # launch folder for launch files   
    │   │   ├── home_robot.rviz               # rviz preset for home_service.sh
    │   │   └── set_goal.launch               # launch file for home_service.sh
    │   ├── src                            
    │   │   └── add_markers.cpp               # marker test code (for add_marker.sh)
    │   ├── CMakeLists.txt                    # compiler instructions
    │   └── package.xml                       # package info
    ├── map                                   # gazebo world file + pgm file                   
    │   ├── empty.world                       # test world file   
    │   ├── myworld.world                         # my gazebo world file   
    │   ├── map.pgm                 # my gazebo map file   
    │   └── map.yaml                # my gazebo map preset   
    ├── pick_objects                          # robot teleoperation package                   
    │   ├── src
    │   │   └── pick_objects.cpp              # teleoperation test code
    │   ├── CMakeLists.txt                    # compiler instructions
    │   └── package.xml                       # package info
    ├── scripts                               # shell script files                   
    │   │   ├── add_marker.sh
    │   │   ├── home_service.sh
    │   │   ├── launch.sh
    │   │   ├── pick_objects.sh
    │   │   ├── test_navigation.sh
    │   │   └── test_slam.sh
    ├── slam_gmapping                        # SLAM package                   
    ├── turtlebot                            # turtlebot package (for teleop)                  
    ├── turtlebot_apps                       # teleop_twist_keyboard package                   
    ├── turtlebot_interactions               # for turtlebot_rviz_launchers                  
    ├── turtlebot_msgs                       # prerequisites                   
    └── turtlebot_simulator                  # for turtlebot_gazebo
