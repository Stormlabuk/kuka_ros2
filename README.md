# Kuka Ros 2

Kuka iiwa communicatiosn with ros2. All ros --> Kuka comms occurs through UDP.

## Dir List
1. Disaster recovery instructions --> how to restore the low-level software on Kuka.
2. Sunrise --> The low-level java code that must be installed to Kuka Sunrise cabinet through Sunrise Workbench.
3. src --> this contains the ROS2 Humble packages that must be installed on the Ubuntu 22 OS, which should have ROS2 Humble installed.

## IP Map
IP of the cabinet: 172.31.1.147.
IP of controlling laptop: 172.31.1.150

## Usage

When correctly installed, there are 3 programs that may be run from the smartpad.
1. Ros2PositionControl --> this program allows joint position control. The corresponding ROS2 package package is robot_comms.
2. Ros2VelocityControl --> this program allows joint position control. The corresponding ROS2 package package is robot_ comms_velocity.
3. Ros2CartesianControl --> this program allows joint position control. The corresponding ROS2 package package is robot_ comms_cart.

