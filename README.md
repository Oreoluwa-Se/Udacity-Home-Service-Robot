# Udacity-Home-Service-Robot
This project aims to design a robot's environment using gazebo and program the robot to map its environment and autonomously navigate to pre-determined pick-up and drop-off locations.
List of steps to be completed:


* Build a map of the environment using gmapping and teleop.
* Use Adaptive Monte Carlo Localisation to detect the robot position within the known map.
* Use the ROS move_base library to plot a path to a target pose and navigate to it.
* Write a pick_objects node to encompass the path planning and driving libraries, listening for goal poses.
* Write a add_marker node to display marker at pickup and drop-off points.

## Project Requirements and Setups
### Update the system:
```bash
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-navigation
```

### install xterm 

```bash
$ sudo apt-get install xterm
```
For this setup, catkin_ws is the name of the active ROS workspace.
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
```

Clone the required repository to the `~/catkin_ws/src` folder. Note that this repository already includes official ROS packages compatible with this repository: 
[gmapping](https://github.com/ros-perception/slam_gmapping), [turtlebot_teleop](https://github.com/turtlebot/turtlebot), 
[turtlebot_rviz_launchers](https://github.com/turtlebot/turtlebot_interactions), and [turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator).

Install package dependencies with `rosdep install [package-name]`

Copy content of this repository to catkin_ws/src

## Directory Structure
Example of how the workspace should be similar to:
```
catkin_ws/src
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├── scripts                        # shell scripts files
    │   ├── ...
    ├──RvizConfig                      # rviz configuration files
    │   ├── ...
    ├──move_read                       # tracker server files 
    │   ├── srv/tracker.srv
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```

## Run project
Source and build the project:
```sh
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make
```
Run `./home_service.sh` in `scripts` directory to deploy the home service robot.

## Video
https://youtu.be/tnTRSRqo0G4
