#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <exception>

#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vn300/Status.h>

class arduino_bridge {
	public:
		arduino_bridge();
		~arduino_bridge();

		bool connect();
		void disconnect();
		bool is_connected() { return serial_device.isOpen(); };
		
		bool send(std::string cmd);
		void receive(std::string response);

		void drive_mode_cb(const std_msgs::String::ConstPtr &msg);
	private:
		std::string device_name;	

		serial::Serial serial_device;
        	serial::utils::SerialListener serial_listener;

		ros::Subscriber drive_mode_sub;

		std_msgs::Bool msg;
};
