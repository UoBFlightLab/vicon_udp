#include <vector>
#include <string>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>

#include "ViconTracker/UDPInterface.hpp"

using namespace Eigen;

static int DEFAULT_PORT = 51001;
static int DEFAULT_BUFSIZE = 1024;

// Advertise TF2 frames for Vicon objects from UDP datastream

geometry_msgs::Quaternion eulXYZ_to_q(double roll, double pitch, double yaw);


int main(int argc, char **argv) {
	// Initialise the node
	ros::init(argc, argv, "vicon2tf");
	ros::NodeHandle nh("~");
	
	// Get parameters for UDP setup
	int buffer_size  = nh.param("buffer_size",DEFAULT_BUFSIZE);
	int bind_port    = nh.param("bind_port",DEFAULT_PORT);
	std::string bind_address_str = nh.param("bind_address",std::string("0.0.0.0"));

	// UDP Socket
	struct sockaddr_in bind_address;        /* our address */
	int fd;                                 /* our socket */
	unsigned char buf[buffer_size];         /* receive buffer */

	// create a UDP socket
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 0;
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
		}
	bind_address.sin_family = AF_INET;
	bind_address.sin_port = htons(bind_port);

	// Bind the socket to any valid IP address and a specific port
	if (bind(fd, (struct sockaddr *)&bind_address, sizeof(bind_address)) < 0) {
		perror("bind failed");
		return 0;
		}

	// Publisher setup
	auto tfPublisher = nh.advertise<tf2_msgs::TFMessage>("/tf", 1);

	// Listen for UDP Data
	while (ros::ok()) {
		int recvlen = recvfrom(fd, buf, buffer_size, 0, NULL, NULL);
		if (!(recvlen == 256 || recvlen == 512 || recvlen == 1024)) {
			ros::spinOnce();
			continue;
			}
		
		tf2_msgs::TFMessage output_tf;
		
		// Frame number and how many items in message to look at
		ViconTracker::UDPStreamPacket packet(buf,buffer_size);

		for(auto object : packet.objects ) {
			// Check if we already have a publisher for this object
			std::string objectName = std::string(object->ItemName);

			// Populate object information
			geometry_msgs::TransformStamped object_tf;
			object_tf.header.frame_id = "world";
			object_tf.header.stamp = ros::Time::now();
			object_tf.child_frame_id = objectName;
			object_tf.transform.translation.x = object->TransX/1000.0;
			object_tf.transform.translation.y = object->TransY/1000.0;
			object_tf.transform.translation.z = object->TransZ/1000.0;
			object_tf.transform.rotation = eulXYZ_to_q(object->RotX,object->RotY,object->RotZ);

			// Add to published TF
			output_tf.transforms.push_back(object_tf);
			}
		tfPublisher.publish(output_tf);
		ros::spinOnce();
		}
	return 0;
	}

geometry_msgs::Quaternion eulXYZ_to_q(double roll, double pitch, double yaw) {
  // Using eigen, and vicon UDP specific rotation order: XYZ

  geometry_msgs::Quaternion q;

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
