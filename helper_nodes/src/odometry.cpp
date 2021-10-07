#include <ros/ros.h>
#include <ros/console.h>
#include "util.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <isc_shared_msgs/EncoderCounts.h>
#include <nav_msgs/Odometry.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/coordinate_convert.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <vn300/Pose.h>
#include <string>
#include <cmath>

class odometry {
  public:
	odometry();
	void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &fix);
	void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);
	void encoder_callback(const isc_shared_msgs::EncoderCounts::ConstPtr &counts);
	bool convert_callback(ohm_igvc_srvs::coordinate_convert::Request &rq, ohm_igvc_srvs::coordinate_convert::Response &rp);

	double gps_x(double lon) { 
		//ROS_INFO("K_EW = %f", K_EW);
		//ROS_INFO("lon = %f", lon);
		//ROS_INFO("start lon = %f", origin.longitude);
		return (K_EW * (lon - origin.longitude)); 
	};

	double gps_y(double lat) { 
		//ROS_INFO("K_NS = %f", K_NS);
		//ROS_INFO("lat = %f", lat);
		//ROS_INFO("start lat = %f", origin.latitude);
		return (K_NS * (lat - origin.latitude));
	};

  private:
	// ros objects
	ros::Subscriber position_sub, imu_sub, encoder_sub;
	ros::Publisher pose_pub, odom_pub;
	ros::ServiceServer coord_convert;
	ros::NodeHandle node;
	ros::Time last_odom_update;

	// frame id's for TF
	std::string world_frame_id, map_frame_id, odom_frame_id, base_link_frame_id;

	// world frame variables
	ohm_igvc_msgs::Target origin;
	double K_NS, K_EW;
	geometry_msgs::Pose2D world_position;

	// odom frame variables
	double tick_to_m;
	double baseline;
	geometry_msgs::Pose2D odom_position;
	nav_msgs::Odometry odom;

	// rotation variables
	sensor_msgs::Imu imu_data, imu_data_prev;
	double rotation_delta;

	// tf
	tf::TransformBroadcaster world_br, odom_br;
	geometry_msgs::TransformStamped t;
};

odometry::odometry() {
	double diameter;
	int ticks_per_rev;

	ros::NodeHandle nh_private("~");

	nh_private.param("K_NS", K_NS, 111120.00);
	nh_private.param("origin_latitude", origin.latitude, 0.0); // in degrees
	nh_private.param("origin_longitude", origin.longitude, 0.0);

	nh_private.param("world_frame", world_frame_id, std::string("world"));
	nh_private.param("map_frame", map_frame_id, std::string("map"));
	nh_private.param("odom_frame", odom_frame_id, std::string("odom"));
	nh_private.param("base_frame", base_link_frame_id, std::string("base_link"));

	nh_private.param("ticks_per_rev", ticks_per_rev, 200);
	nh_private.param("diameter", diameter, 13.0); // in cm
	nh_private.param("baseline", baseline, 50.0); // in cm

	K_EW = K_NS * std::cos(utility::geometry::radians(origin.latitude));

	ROS_INFO("K_NS = %f", K_NS);
	ROS_INFO("K_EW = %f", K_EW);

	position_sub = node.subscribe<sensor_msgs::NavSatFix>("navsat", 5, &odometry::gps_callback, this);
	imu_sub = node.subscribe<sensor_msgs::Imu>("imu", 5, &odometry::imu_callback, this);
	coord_convert = node.advertiseService("coordinate_convert", &odometry::convert_callback, this);

	pose_pub = node.advertise<geometry_msgs::Pose2D>("pose", 1);
	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1);

	odom_position.x = 0.0;
	odom_position.y = 0.0;
	odom_position.theta = 0.0;

	tick_to_m = ((utility::geometry::pi * diameter) / ticks_per_rev) / 100.0; // calculate how far one tick takes the wheel
}

void odometry::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &fix) {
	world_position.x = gps_x(fix->longitude);
	world_position.y = gps_y(fix->latitude);

	world_position.theta = tf::getYaw(imu_data.orientation);

	t.header.stamp = ros::Time::now();
	t.header.frame_id = world_frame_id; // THIS SHOULD BE WORLD TO MAP, NOT MAP TO BASE LINK
	t.child_frame_id = map_frame_id; // PLZ FIX U MORON

	t.transform.translation.x = world_position.x;
	t.transform.translation.y = world_position.y;
	t.transform.translation.z = 0.0;
	t.transform.rotation = imu_data.orientation;

	world_br.sendTransform(t);

	pose_pub.publish(world_position);
}

void odometry::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
	using namespace utility::circular_range;
	using namespace utility::geometry;
	imu_data = *imu;
	odom_position.theta = radians(wrap(degrees(direction(tf::getYaw(imu_data.orientation), odom_position.theta) * smallest_difference(tf::getYaw(imu_data.orientation), odom_position.theta)), 360.0));
}

void odometry::encoder_callback(const isc_shared_msgs::EncoderCounts::ConstPtr &counts) {
	ros::Time current_time = ros::Time::now();
	double timestep = (current_time - last_odom_update).toSec(); // calculate timestep

	double distance_traveled = ((counts->left_count * tick_to_m) + (counts->right_count * tick_to_m)) / 2; // calculate distance traveled in cm

	odom_position.x += distance_traveled * std::cos(utility::geometry::radians(odom_position.theta)); // update x position
	odom_position.y += distance_traveled * std::sin(utility::geometry::radians(odom_position.theta)); // update y position

	// fill out transform to publish
	t.header.stamp = current_time;
	t.header.frame_id = odom_frame_id;
	t.child_frame_id = base_link_frame_id;

	t.transform.translation.x = odom_position.x;
	t.transform.translation.y = odom_position.y;
	t.transform.translation.z = 0.0;
	t.transform.rotation = imu_data.orientation;

	// fill out odom message
	odom.header = t.header;
	odom.pose.pose.position.x = odom_position.x;
	odom.pose.pose.position.y = odom_position.y; // z field is unused
	odom.pose.pose.orientation = imu_data.orientation;

	odom.child_frame_id = t.child_frame_id;

	odom.twist.twist.linear.x = (distance_traveled / timestep); // calculate linear velocity and convert to m/s
	odom.twist.twist.angular = imu_data.angular_velocity;

	odom_br.sendTransform(t);
	odom_pub.publish(odom);

	// update the time
	last_odom_update = current_time;
}

bool odometry::convert_callback(ohm_igvc_srvs::coordinate_convert::Request &rq, ohm_igvc_srvs::coordinate_convert::Response &rp) {
	rp.coordinate.position.x = gps_x(rq.coordinate.longitude);
	rp.coordinate.position.y = gps_y(rq.coordinate.latitude);
	rp.coordinate.orientation = tf::createQuaternionMsgFromYaw(rq.coordinate.heading);

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "odometry");

	odometry node;

	ros::spin();

	return 0;
}
