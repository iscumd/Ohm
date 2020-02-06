#include "arduino_bridge.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "arduino_bridge");

	arduino_bridge bridge;

	ros::Rate rt(1);
	
	while (!bridge.connect() && ros::ok()) {
		rt.sleep();
		ROS_INFO("Attempting to connect to arduino!");
	}

	ros::spin();

	return 0;
}
