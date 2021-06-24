#include <string>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include "ViconTracker/UDPInterface.hpp"

void get_common_parameters(rclcpp::Node const& node, std::string& frame_name) {
    constexpr static char const DEFAULT_FRAMENAME[] = "vicon";

    node.get_parameter_or("frame_name",frame_name,std::string(DEFAULT_FRAMENAME));
    }

void get_UDP_parameters(rclcpp::Node const& node, int& buffer_size, int& bind_port, std::string& bind_address_str) {
    constexpr static int const DEFAULT_PORT = 51001;
    constexpr static int const DEFAULT_BUFSIZE = 1024;
    constexpr char const DEFAULT_BINDADDR[] = "0.0.0.0";

    node.get_parameter_or("buffer_size",buffer_size,DEFAULT_BUFSIZE);
	node.get_parameter_or("bind_port",bind_port,DEFAULT_PORT);
	node.get_parameter_or("bind_address",bind_address_str,std::string(DEFAULT_BINDADDR));
    }

int get_UDP_fd(int const bind_port, std::string const bind_address_str) {
    // UDP Socket
	struct sockaddr_in bind_address;        /* our address */
	int fd;                                 /* our socket */

	// create a UDP socket
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("Failed to create socket\n");
		return -1;
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
		return -1;
		}
	bind_address.sin_family = AF_INET;
	bind_address.sin_port = htons(bind_port);

	// Bind the socket to any valid IP address and a specific port
	if (bind(fd, (struct sockaddr *)&bind_address, sizeof(bind_address)) < 0) {
		perror("Binding to address failed");
		return -1;
		}
    
    return fd;
    }

ViconTracker::UDPStreamPacket recvViconPacket(int const fd, unsigned char* const buf, int const buffer_size) {
    int recvlen = recvfrom(fd, buf, buffer_size, 0, NULL, NULL);
	if (!(recvlen == 256 || recvlen == 512 || recvlen == 1024)) {
		return ViconTracker::UDPStreamPacket();
		}
		
	// Parse buffer
	return ViconTracker::UDPStreamPacket(buf,buffer_size);
    }

geometry_msgs::msg::Quaternion eulXYZ_to_q(double roll, double pitch, double yaw) {
    // Using eigen, and vicon UDP specific rotation order: XYZ
    using namespace Eigen;

    geometry_msgs::msg::Quaternion q;

    Quaterniond qe;
    qe = Eigen::AngleAxisd(roll, Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Vector3d::UnitZ());

    q.x = qe.x();
    q.y = qe.y();
    q.z = qe.z();
    q.w = qe.w();

    return q;
    }

geometry_msgs::msg::PoseStamped viconObjectToPoseStamped(ViconTracker::TrackerObject_raw const* const object) {
    // Populate object information
	geometry_msgs::msg::PoseStamped object_pose;
	object_pose.pose.position.x = object->TransX/1000.0;
	object_pose.pose.position.y = object->TransY/1000.0;
	object_pose.pose.position.z = object->TransZ/1000.0;
	object_pose.pose.orientation = eulXYZ_to_q(object->RotX,object->RotY,object->RotZ);

    return object_pose;
    }
