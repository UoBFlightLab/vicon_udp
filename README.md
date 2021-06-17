# `vicon_udp`

This ROS package consumes the UDP stream from Vicon Tracker and exposes it to ROS topics.

The UDP stream is limited compared to the TCP interface, but has the advantage of running over UDP, which in some cases
can reduce the latency of tracking information.

There are two main ways that the information can be exposed to ROS:
 - as `/tf` transforms from `vicon2tf`, or
 - as a set of `/vicon/${NAME}` pose topics from  `vicon2pose`

There is also a third executable, `vicon_object_pose` which publishes the information from a single Vicon object. In
some scenarios, this can reduce network traffic, as ROS traffic is not broadcast alongside the original Vicon stream.

## Usage

### All poses to `/tf`

`vicon2tf` listens to the UDP stream and publishes to the `/tf` topic with a `tf2_msgs/TFMessage` containing the
transforms of all objects contained in that packet. Each transform has a frame of `"vicon"` and has the object name from
the Vicon stream as the `child_frame_id`. It can be run using:

`ros2 run vicon_udp vicon2tf`

> TODO: Invalid Vicon names


### All poses from single node

`vicon2pose` listens to the UDP stream and publishes a topic for each name it sees in the Vicon stream. The poses for
each object are published as `geometry_msgs/PoseStamped` messages on individual topics with named of the form:
`vicon/${OBJECT_NAME}`. It can be run using:

`ros2 run vicon_udp vicon2pose`

> TODO: Handle invalid ROS topic names in Vicon stream

There is a launch file available:

`ros2 launch vicon_udp vicon2pose.launch.xml`

### Publish pose of a single object

`vicon_object_pose` listens to the UDP stream and publishes the poses of a single object on the `~/pose` topic. The
target object is configured with a parameter.

`ros2 run vicon_udp vicon_object_pose --ros-args -p target_object:=drone1`

### Common parameters

All of the executables files take two parameters to setup the connection to Vicon Tracker. Details in the table:

Parameter      | Default   | Description
---------------|-----------|---------------------
`bind_address` | `0.0.0.0` | Address to listen on
`bind_port`    | `51001`   | Port to listen on
`buffer_size`  | `1024`    | Default receive buffer size
`frame_name`   | `vicon`   | Default frame name for Poses or TFs


### Common launch file arguments

All of the launch files take two arguments that pass values through to the executable parameters. Details in the table:

Argument       | Default   | Description
---------------|-----------|---------------------
`bind_address` | `0.0.0.0` | Address to listen on
`bind_port`    | `51001`   | Port to listen on
`frame_name`   | `vicon`   | Default frame name for Poses or TFs
