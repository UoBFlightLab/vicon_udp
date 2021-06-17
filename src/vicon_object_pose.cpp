#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "common.cpp"

// Advertise poses for a target Vicon object from UDP datastream

int main(int argc, char **argv) {
	// Initialise the node
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node("vicon_object_pose");

	node.declare_parameter<std::string>("target_object","");

	std::string targetObject;
	auto param_was_set = node.get_parameter("target_object",targetObject);
	if(!param_was_set || targetObject == "") {
		perror("Could not get target object\n");
		return 1;
		}
	
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
	auto publisher = node.create_publisher<geometry_msgs::msg::PoseStamped>("/vicon/" + targetObject, 1);

	// Listen for UDP Data
	while (rclcpp::ok()) {
		auto packet = recvViconPacket(fd,buf,buffer_size);

		for(auto object : packet.objects ) {
			std::string objectName = std::string(object->ItemName);
			if( objectName != targetObject ) {
				// Not our target object
				continue;
				}

			auto object_pose = viconObjectToPoseStamped(object);
			object_pose.header.frame_id = frame_name;
			object_pose.header.stamp = node.get_clock()->now();

			publisher->publish(object_pose);
			}
		
		}

	return 0;
	}
