#!/bin/sh

#Extracting working directory from the test base
cmd=$(pwd)
cd ..
cd ..
Base=$(pwd)

#Deploy source file and launch turtlebot_world.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

#Deploy gmapping_demo.launch or slam_gmapping
xterm -e "cd ${Base} && source devel/setup.bash && rosrun gmapping slam_gmapping" &
sleep 5

#Deploy view_navigation.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#Deploy keyboard_teleop.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch" 
