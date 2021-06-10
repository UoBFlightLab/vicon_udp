#include <vector>
#include <string>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ViconTracker/UDPInterface.hpp"

using namespace Eigen;

static int DEFAULT_PORT = 51001;
static int DEFAULT_BUFSIZE = 1024;

// Advertise poses for Vicon objects from UDP datastream

geometry_msgs::msg::Quaternion eulXYZ_to_q(double roll, double pitch, double yaw);


using PosePublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;

int main(int argc, char **argv) {
	// Initialise the node
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node("vicon2pose");

	node.declare_parameter<std::string>("target_object","");

	std::string targetObject;
	auto param_was_set = node.get_parameter("target_object",targetObject);
	if(!param_was_set || targetObject == "") {
		perror("Could not get target object\n");
		return 1;
	}
	
	// Get parameters for UDP setup
	int buffer_size;
	node.get_parameter_or("buffer_size",buffer_size,DEFAULT_BUFSIZE);
	int bind_port;
	node.get_parameter_or("bind_port",bind_port,DEFAULT_PORT);
	std::string bind_address_str;
	node.get_parameter_or("bind_address",bind_address_str,std::string("0.0.0.0"));

	// UDP Socket
	struct sockaddr_in bind_address;        /* our address */
	int fd;                                 /* our socket */
	unsigned char buf[buffer_size];         /* receive buffer */

	// create a UDP socket
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 1;
		}
	
	// Enable software/hardware timestamping
	// https://stackoverflow.com/questions/47313383/linux-udp-datagrams-and-kernel-timestamps-lots-of-examples-and-stackoversflow
	/*
	int timestampOn = SOF_TIMESTAMPING_RX_SOFTWARE
	                | SOF_TIMESTAMPING_RX_HARDWARE
	                | SOF_TIMESTAMPING_SOFTWARE
	                | SOF_TIMESTAMPING_RAW_HARDWARE;
	int ret = setsockopt(fd,SOL_SOCKET,SO_TIMESTAMPING,&timestampOn,sizeof(timestampOn));
	
	if (ret < 0) {
		fprintf(stderr, "setsockopt failed: %s\n",strerror(errno));
		return ret;
		}
	*/

	// Setup the bind address
	memset((uint8_t*) &bind_address, 0, sizeof(bind_address));  
	// Convert string representation of IP address to struct
	if( !inet_pton(AF_INET, bind_address_str.c_str(), &(bind_address.sin_addr)) ) {
		perror("Error parsing bind_address parameter");
		return 1;
		}
	bind_address.sin_family = AF_INET;
	bind_address.sin_port = htons(bind_port);

	// Bind the socket to any valid IP address and a specific port
	if (bind(fd, (struct sockaddr *)&bind_address, sizeof(bind_address)) < 0) {
		perror("bind failed");
		return 1;
		}

	// Publisher setup
	auto publishers = std::map<std::string,std::shared_ptr<PosePublisher>>();

	// Listen for UDP Data
	while (rclcpp::ok()) {
		int recvlen = recvfrom(fd, buf, buffer_size, 0, NULL, NULL);
		if (!(recvlen == 256 || recvlen == 512 || recvlen == 1024)) {
			continue;
			}
		
		// Parse buffer
		ViconTracker::UDPStreamPacket packet(buf,buffer_size);

		for(auto object : packet.objects ) {
			std::string objectName = std::string(object->ItemName);
			if(publishers.find(objectName) == publishers.end()) {
				// If we don't have a publisher for this object, create one
				publishers[objectName] = node.create_publisher<geometry_msgs::msg::PoseStamped>("/vicon/" + objectName, 1);
				}

			// Populate object information
			geometry_msgs::msg::PoseStamped object_pose;
			object_pose.header.frame_id = "vicon";
			object_pose.header.stamp = node.get_clock()->now();
			object_pose.pose.position.x = object->TransX/1000.0;
			object_pose.pose.position.y = object->TransY/1000.0;
			object_pose.pose.position.z = object->TransZ/1000.0;
			object_pose.pose.orientation = eulXYZ_to_q(object->RotX,object->RotY,object->RotZ);

			if (objectName == targetObject) {
				// Add to published TF
				publishers[objectName]->publish(object_pose);
				}
			}
		
		}
	return 0;
	}

geometry_msgs::msg::Quaternion eulXYZ_to_q(double roll, double pitch, double yaw) {
  // Using eigen, and vicon UDP specific rotation order: XYZ

  geometry_msgs::msg::Quaternion q;

  Eigen::Quaterniond qe;
  qe = Eigen::AngleAxisd(roll, Vector3d::UnitX())
	  * Eigen::AngleAxisd(pitch, Vector3d::UnitY())
	  * Eigen::AngleAxisd(yaw, Vector3d::UnitZ());

  q.x = qe.x();
  q.y = qe.y();
  q.z = qe.z();
  q.w = qe.w();

  return q;
}
