#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <move_read/tracker.h>
#include <ros/ros.h>
#include <tf/tf.h>

class Location{
	// initialize node handles, publishers, and servers
	ros::NodeHandle n;
	ros::Publisher marker_pub;
	ros::ServiceClient pose_;
	visualization_msgs::Marker marker;
	
	public:
		// create vector to track pick-up and drop off positions
		std::vector<float> positions = std::vector<float>(6);
		
		Location(){
			// get node name
			std::string node_name = ros::this_node::getName();

			//populate vector of positions -> from preset parameters 
      		// in launch files
			n.getParam(node_name + "/pick_up_x", positions[0]);
			n.getParam(node_name + "/pick_up_y", positions[1]);
			n.getParam(node_name + "/pick_up_z", positions[2]);
			n.getParam(node_name + "/drop_off_x", positions[3]);
			n.getParam(node_name + "/drop_off_y", positions[4]);
			n.getParam(node_name + "/drop_off_z", positions[5]);

			// publish pose to /add_markers/pose topic
			pose_ = n.serviceClient<move_read::tracker>("/add_markers/pose");

			// advertise marker and locations - pick up and drop off
			marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      
      		// calls the callback function
			destination_callback();

		}

    	// The brains of the operation
		void destination_callback(){
			// create pose variable and variable to communicate with server
			geometry_msgs::Pose loc_;
			move_read::tracker trans;
			
			// two cycles for pick-up and drop-off spots
			for (int i = 0; i < 6; i=i+3){
				// extract locations from the storage
				loc_.position.x = positions[i];
				trans.request.point_x = positions[i];
				loc_.position.y = positions[i + 1];
				trans.request.point_y = positions[i + 1];
				loc_.position.z = positions[i + 2];
				trans.request.point_z = positions[i + 2];

				// initialize the marker
				create_marker(loc_);
        
        		// shows marker at pick-up point
				if (i == 0){
					show_marker(loc_);
				} 

				// request move to locations
				if (!pose_.call(trans)){
					ROS_ERROR("Failed to call the add_markers/pose service");
				}

				// operation after moving hide the marker at pick-up point
        		// show when at drop-off location
				if (i == 0){
					hide_marker();
				} else {
					show_marker(loc_);

				}

			}

		}

		// publish marker
		void publish_marker(){
        	// ensure subscriber to the visualization message
			while (marker_pub.getNumSubscribers() < 1){
				ROS_WARN_ONCE("Please create a subscriber to the marker");
			}
			if (ros::ok()){
				marker_pub.publish(marker);
			}
		}

		// show marker
		void show_marker(geometry_msgs::Pose& point_){
			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration();
			publish_marker();
		}

		// function to hide marker
		void hide_marker(){
			marker.action = visualization_msgs::Marker::DELETE;
			publish_marker();
		}

		// function to initialize the marker type
		void create_marker(geometry_msgs::Pose& point_){
			
			//set initial shape type to cube
			uint32_t shape = visualization_msgs::Marker::CUBE;

			//set the frame ID and time stamp
		  	marker.header.frame_id = "map";	
			marker.header.stamp = ros::Time::now();

			// create marker name and id number and type
			marker.ns = "add_markers";
			marker.id = 0;
			marker.type = shape;

			// marker action to take
			marker.action = visualization_msgs::Marker::ADD;

			// marker pose and orientation
			marker.pose.position.x = point_.position.x;
			marker.pose.position.y = point_.position.y;
			marker.pose.position.z = point_.position.z;
			marker.pose.orientation = tf::createQuaternionMsgFromYaw(point_.position.z);

			// marker scale properties
			marker.scale.x = 1.0f;
			marker.scale.y = 1.0f;
			marker.scale.z = 1.0f;

			// marker color
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0f;

		}

// END OF CLASS
};

// main function
int main( int argc, char** argv )
{
	// initialize ros and node name
	ros::init(argc, argv, "add_markers");
  
  	// Begins program
	Location locations;

	// Handle ROS communication events
  	ros::spin(); 

  	// END OF PROGRAM
  	return 0;
}
