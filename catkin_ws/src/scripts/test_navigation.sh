#!/bin/sh

#Extracting working directory from the test base
cmd=$(pwd)
cd ..
cd ..
Base=$(pwd)

#Deploy source file and launch turtlebot_world.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

#Deploy amcl demon from turtlebot simulator package
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

#Deploy view_navigation.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" 

