#!/bin/sh

#Extracting working directory from the test base
cmd=$(pwd)
cd ..
cd ..
Base=$(pwd)

#Deploy source file and launch turtlebot_world.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

#Deploy amcl demo from turtlebot simulator package
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

#Deploy view_navigation.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#Deploy pick_objects node
xterm -e "cd ${Base} && source devel/setup.bash && rosrun pick_objects pick_objects" &
sleep 5

#Deploy add_markers.launch
xterm -e "cd ${Base} && source devel/setup.bash && roslaunch add_markers markers.launch"
