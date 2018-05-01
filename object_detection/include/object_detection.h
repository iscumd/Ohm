#ifndef OBJ_DETECT_H
#define OBJ_DETECT_H
#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <ohm_igvc_msgs/Range.h>
#include <ohm_igvc_msgs/RangeArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "util.h"

#include <list>
#include <cmath>
#include <vector>
#include <iterator>
#include <algorithm>
#include <boost/math/constants/constants.hpp>


class object_detector {
	public:
		object_detector(ros::NodeHandle nh);

		//*** MAIN FUNCTIONS ***//

		void find_point_groups(const sensor_msgs::PointCloud::ConstPtr &pcl);
		void find_valid_ranges(std::vector<std::list<geometry_msgs::Point32>> groups);

		//*** HELPER FUNCTIONS ***//

		void publish_obstacles_rviz(std::vector<std::list<geometry_msgs::Point32>> groups);
		geometry_msgs::Point point32_to_point(geometry_msgs::Point32 p);
		double quaternion_to_heading(tf::Quaternion q);
		
		// templates

		template <typename point_iterator>
		point_iterator find_next_point(point_iterator current, point_iterator end);

	private:
		int forgivable;
		double eps;

		int min_group_count;

		// ROS specific
		ros::NodeHandle node;
		ros::Subscriber pcl_input;
		ros::Publisher object_groups_pub, object_markers_pub, range_pub;
		std::string pcl_frame;

		// robot stuff
		tf::TransformListener pose_listener;
		ros::Time last_received_pcl;
		double min_obstacle_distance;
};

#endif
