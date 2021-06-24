#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include "common.cpp"

// Advertise TF2 frames for Vicon objects from UDP datastream

int main(int argc, char **argv) {
	// Initialise the node
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node("vicon2tf");

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
	auto tfPublisher = node.create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);

	// Listen for UDP Data
	while (rclcpp::ok()) {
		auto packet = recvViconPacket(fd,buf,buffer_size);
		
		tf2_msgs::msg::TFMessage output_tf;

		for(auto object : packet.objects ) {
			// Check if we already have a publisher for this object
			std::string objectName = std::string(object->ItemName);

			// Populate object information
			geometry_msgs::msg::TransformStamped object_tf;
			object_tf.header.frame_id = frame_name;
			object_tf.header.stamp = node.get_clock()->now();
			object_tf.child_frame_id = objectName;
			object_tf.transform.translation.x = object->TransX/1000.0;
			object_tf.transform.translation.y = object->TransY/1000.0;
			object_tf.transform.translation.z = object->TransZ/1000.0;
			object_tf.transform.rotation = eulXYZ_to_q(object->RotX,object->RotY,object->RotZ);

			// Add to published TF
			output_tf.transforms.push_back(object_tf);
			}
		tfPublisher->publish(output_tf);
		}
	return 0;
	}
