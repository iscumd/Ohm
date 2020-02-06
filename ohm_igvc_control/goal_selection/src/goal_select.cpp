#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/waypoint.h>
#include <ohm_igvc_srvs/coordinate_convert.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movement_server;
typedef actionlib::SimpleClientGoalState Goals;

class waypoint {
	public:
		waypoint() : move_requests("move_base", true) {
			ros::NodeHandle nh;

			ros::NodeHandle nh_private("~");
			nh_private.param("odom_frame_id", odom_frame_id, std::string("odom"));
			nh_private.param("base_frame_id", base_frame_id, std::string("base_link"));	
			nh_private.param("map_frame_id", map_frame_id, std::string("map"));

			nh.param("auto_state_string", auto_state_string, std::string("auto"));

			ros::Subscriber drivemodeSub = nh.subscribe("/signal/drivemode", 0, &waypoint::drive_mode_callback, this);
			
			waypoint_srv = nh.serviceClient<ohm_igvc_srvs::waypoint>("waypoint");
			coordinate_convert_srv = nh.serviceClient<ohm_igvc_srvs::coordinate_convert>("coordinate_convert");
		}

		void drive_mode_callback(const std_msgs::String::ConstPtr &drive_mode) {
			if(current_drive_mode != drive_mode->data){
				waypoint_id = 0;
				current_drive_mode = drive_mode->data;
				get_waypoint();
				start_path();
			}
			
		}


		void start_path(){
			if(current_drive_mode == auto_state_string) {
				move_base_msgs::MoveBaseGoal goal_req;
				goal_req.target_pose.header.frame_id = goal_frame_id;
				goal_req.target_pose.header.stamp = ros::Time::now();
				goal_req.target_pose.pose = goal;

				move_requests.sendGoal(goal_req, boost::bind(&waypoint::goal_complete, this, _1, _2));	
			}
		}
  
		bool get_waypoint() {
				ohm_igvc_srvs::waypoint req_wp;

				req_wp.request.ID = waypoint_id;

				if(!waypoint_srv.call(req_wp)) return false; // probably means there are no more waypoints left
			
				if((req_wp.response.waypoint.header.frame_id == odom_frame_id) || (req_wp.response.waypoint.header.frame_id == base_frame_id) || (req_wp.response.waypoint.header.frame_id == map_frame_id)){
					goal_frame_id = req_wp.response.waypoint.header.frame_id;					
					goal.position.x = req_wp.response.waypoint.latitude;
					goal.position.y = req_wp.response.waypoint.longitude;
					goal.orientation = tf::createQuaternionMsgFromYaw(req_wp.response.waypoint.heading);
				}

				else{
					ohm_igvc_srvs::coordinate_convert conv_wp;
					conv_wp.request.coordinate = req_wp.response.waypoint;

					if(!coordinate_convert_srv.call(conv_wp)) return false;

					goal = conv_wp.response.coordinate;
				}

				return true;
		}
  
		void goal_complete(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result) { 
			if(current_drive_mode == auto_state_string){

				move_base_msgs::MoveBaseGoal goal_req;

				goal_req.target_pose.header.frame_id = goal_frame_id;
				goal_req.target_pose.header.stamp = ros::Time::now();
				goal_req.target_pose.pose = goal;

				move_requests.sendGoal(goal_req, boost::bind(&waypoint::goal_complete, this, _1, _2));
			}
		}
	
	private:
		ros::Subscriber drivemodeSub;
		ros::ServiceClient waypoint_srv;
		ros::ServiceClient coordinate_convert_srv;
		movement_server move_requests;

		int waypoint_id;
		std::string current_drive_mode;

		std::string odom_frame_id;
		std::string base_frame_id;
		std::string map_frame_id;
		std::string goal_frame_id;
		std::string auto_state_string;

		geometry_msgs::Pose goal;
};

int main(int argc, char **argv){
	
	ros::init(argc, argv, "goal_select");
	
	waypoint goal_selection;
	
	ros::spin();
  
	return 0;
}
