#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "common.cpp"

// Advertise poses for Vicon objects from UDP datastream

int main(int argc, char **argv) {
	// Initialise the node
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node("vicon2pose");

	// Get common parameters
	std::string frame_name;
	get_common_parameters(node,frame_name);

	// Get parameters for UDP setup
	int buffer_size;
	int bind_port;
	std::string bind_address_str;
	get_UDP_parameters(node,buffer_size,bind_port,bind_address_str);

	// Get a file descriptor for the UDP port
	int fd = get_UDP_fd(bind_port,bind_address_str);
	if( fd == -1 ) {
		return 1;	
		}
	
	// Receive buffer
	unsigned char buf[buffer_size];

	// Publisher setup
	using PosePublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;
	auto publishers = std::map<std::string,std::shared_ptr<PosePublisher>>();

	// Listen for UDP Data
	while (rclcpp::ok()) {
		auto packet = recvViconPacket(fd,buf,buffer_size);

		for(auto object : packet.objects ) {
			std::string objectName = std::string(object->ItemName);
			if(publishers.find(objectName) == publishers.end()) {
				// If we don't have a publisher for this object, create one
				publishers[objectName] = node.create_publisher<geometry_msgs::msg::PoseStamped>("vicon/" + objectName, 1);
				}

			auto object_pose = viconObjectToPoseStamped(object);
			object_pose.header.frame_id = frame_name;
			object_pose.header.stamp = node.get_clock()->now();

			publishers[objectName]->publish(object_pose);
			}
		
		}
	return 0;
	}
