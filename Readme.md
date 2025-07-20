# Robocon Pi

## Setup Instructions

Before running any ROS 2 nodes, **you must source the ROS 2 environment and set up the workspace folder**.  
Alternatively, you can simply run:

```bash
source setup.bash
```

This script will handle both the ROS 2 environment and folder setup for you.

## Firmware

All ESP32 firmware sketches are located in the `arduino` folder.

## Device Port Mappings

Port mappings for each device are specified in the `devices.env` file.


## Frequently Used Commands

Here are some common commands for working with this project:

```bash
# Build the ROS 2 workspace
colcon build

# Source the workspace after building
source install/setup.bash

# List available ROS 2 nodes
ros2 node list

# Run a specific ROS 2 node (replace <package> and <executable>)
ros2 run <package> <executable>

# Check device connection
ls /dev/tty*
```

## Additional ROS 2 Commands

Here are some more useful ROS 2 commands:

```bash
# List all ROS 2 topics
ros2 topic list

# Echo messages from a topic (replace <topic_name>)
ros2 topic echo <topic_name>

# Publish a message to a topic (replace <topic_name> and <msg_type>)
ros2 topic pub <topic_name> <msg_type> '{data: ...}'

# List all ROS 2 services
ros2 service list

# Call a service (replace <service_name> and <srv_type>)
ros2 service call <service_name> <srv_type> '{...}'

# List all ROS 2 parameters for a node (replace <node_name>)
ros2 param list /<node_name>

# Set a parameter for a node (replace <node_name> and <param>)
ros2 param set /<node_name> <param> <value>
```