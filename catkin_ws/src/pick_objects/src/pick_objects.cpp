#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <move_read/tracker.h>
#include <ros/ros.h>
#include <tf/tf.h>


class Pick_Objects{
  // Define a client for to send goal requests to the move_base server through a SimpleActionClient
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  ros::NodeHandle n;
  ros::ServiceServer pose_;
  move_base_msgs::MoveBaseGoal goal;

  public:
   Pick_Objects(){
    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";

    pose_ = n.advertiseService("/add_markers/pose", &Pick_Objects::handle_new_locations, 
      this);
	}

  // define a callback function when new pickup and drop off locations are included
  bool handle_new_locations(move_read::tracker::Request& req, 
    move_read::tracker::Response& res){
    
    // display recieved pickup points
    ROS_INFO("Positions recieved - x: %1.2f, y: %1.2f, z: %1.2f", 
      (float)req.point_x, (float)req.point_y, (float)req.point_z);
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // define a (2, 3) array to store pickup and drop off zones
    double goals[3] = {(float)req.point_x, (float)req.point_y, (float)req.point_z};

    // current time
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goals[0];
    goal.target_pose.pose.position.y = goals[1];
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goals[2]);

    
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // output destination location
    ROS_INFO("Heading towards location");

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Destination reached");

      //pause for 5 seconds
      ros::Duration(5).sleep();

    }  else {
      ROS_INFO("Location not reached");
    }

    return true;     
  }
};


int main(int argc, char** argv){
  // Initialize the pick objects node
  ros::init(argc, argv, "pick_objects");  

  // call service class
  Pick_Objects pick_object;

  // Handle ROS communication events
  ros::spin(); 

  // End of main
  return 0;
}
