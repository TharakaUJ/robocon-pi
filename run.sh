#!/bin/bash

# Script to run ROS bridge and topics

# Source ROS setup (update the path if needed)
source /opt/ros/noetic/setup.bash
source ./src/devices.env

# Start rosbridge server in the background
roslaunch rosbridge_server rosbridge_websocket.launch &

# Wait for rosbridge to start
sleep 3

# Example: Start a topic publisher (replace with your actual topic/node)
# rosrun your_package your_topic_publisher_node &

# Example: Start a topic subscriber (replace with your actual topic/node)
# rosrun your_package your_topic_subscriber_node &

echo "ROS bridge and topics are running."

# Wait for background processes
wait