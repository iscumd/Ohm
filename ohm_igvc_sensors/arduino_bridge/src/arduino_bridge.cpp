#include "arduino_bridge.h"

arduino_bridge::arduino_bridge() {
	/*
		params:
			string device -> the name of the serial port to communicate on (default: /dev/ttyACM0)
		
		subscriptions:
			std_msgs/String drive_mode -> input topic for the current drive_mode (i.e. auto or manual)
	*/

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("device", device_name, std::string("/dev/ttyACM0"));

	drive_mode_sub = nh.subscribe("drive_mode", 1, &arduino_bridge::drive_mode_cb, this);

	estop_state = nh.advertise<isc__msgs::Bool>("estop", 10);
	kill_state = nh.advertise<isc__msgs::Bool>("kill", 10);
	pause_state = nh.advertise<isc__msgs::Bool>("pause", 10);
}

arduino_bridge::~arduino_bridge() {
	disconnect();
}

bool arduino_bridge::connect() {
	if (serial_device.isOpen()) {
		disconnect();	
	}
	
	// configure the serial port
	serial_device.setPort(device_name);
	serial_device.setBaudrate(9600);
	serial_device.setBytesize(serial::eightbits);
	serial_device.setParity(serial::parity_none);

	serial::Timeout t = serial::Timeout::simpleTimeout(10);
	serial_device.setTimeout(t);

	// configure the serial listener
	serial_listener.setChunkSize(64);
	serial_listener.setTokenizer(serial::utils::SerialListener::delimeter_tokenizer("\r\n"));
	serial_listener.setDefaultHandler(boost::bind(&arduino_bridge::receive, this, _1));
	
	// open port and start listening
	try {
		serial_device.open();
		serial_listener.startListening(serial_device);
	} catch (std::exception &e) {
		ROS_ERROR("Serial exception!: %s", e.what());
		return false;
	}

	ROS_INFO("Connected to Arduino!");

	return true;
}

void arduino_bridge::disconnect() {
	if (serial_device.isOpen()) {
		serial_listener.stopListening();
		serial_device.close();
	}
}

bool arduino_bridge::send(std::string cmd) {
	ROS_DEBUG("Sending command to Arduino: %s", cmd.c_str());
	int send_count = 0;
		
	send_count = serial_device.write(cmd);
	send_count += serial_device.write("\r\n");

	return (send_count == (cmd.length() + 2)); // 2 is the length of line endings
}

void arduino_bridge::receive(std::string response) {
	ROS_DEBUG("Received from Arduino: %s", response.c_str());
	std::vector<std::string> data;
	boost::split(data, response, boost::is_any_of(","));
	if (data.at(0)  == "$"){

		msg = boost::lexical_cast<bool>(data.at(1));
		estop_state.publish(msg);

		msg = boost::lexical_cast<bool>(data.at(2));
		kill_state.publish(msg);

		msg = boost::lexical_cast<bool>(data.at(3));
		pause_state.publish(msg);
	}
	
	
}

void arduino_bridge::drive_mode_cb(const std_msgs::String::ConstPtr &msg) {
	if (msg->data == "auto") {
		send("A");
	} else {
		send("M");
	}
}
