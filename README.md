# VIO (Visual Inertial Odometry)


This package is a bridge from PX4 to the Realsense T265 camera, which provides odometry data.

## Dependencies
* librealsense: [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense)

   - For Jetson Nano [follow these instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) and install librealsense2. *T265 no longer works on the newest librealsense version. Will have to look into how to download a previous version of it.

* ROS Kinetic (Ubuntu 16.04) or ROS Melodic (Ubuntu 18.04): [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation) (We used ROS Melodic for the tests)


## Installation
These steps contain the installation process, software dependencies and building instructions.

1. This is a ROS package, it assumes you have either ROS Kinetic (Ubuntu 16.04) or ROS Melodic (Ubuntu 18.04) installed, instructions can be found [here](http://wiki.ros.org/ROS/Installation). Please also make sure you install librealsense from [here](https://github.com/IntelRealSense/librealsense). The specific installation instructions for Jetson Nano are located [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md).

1. Install catkin and create your catkin workspace directory.

   ```bash
   sudo apt install python-catkin-tools
   mkdir -p ~/catkin_ws/src
   ```

1. Clone this repository in your catkin workspace.

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/lesc-utfpr/px4_odom_bridge.git
   ```

1. Install MAVROS (version 0.29.0 or above).
   > **Note:** Instructions to install MAVROS from sources can be found [here](https://dev.px4.io/en/ros/mavros_installation.html).
   
   * Melodic
     ```bash
     sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
     ```
   * Kinetic
     ```bash
     sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras
     ```

1. Don't forget to install the [GeographicLib](https://geographiclib.sourceforge.io/) datasets:
   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   sudo bash ./install_geographiclib_datasets.sh   
   ```

1. Install the [realsense2_camera](https://github.com/IntelRealSense/realsense-ros#installation-instructions) package for access to T265 in ROS:
   ```bash
   sudo apt install ros-melodic-realsense2-camera
   ```

1. Install the ROS [Point Cloud Library](http://wiki.ros.org/pcl_ros) (PCL):

   * Melodic
     ```bash
     sudo apt install ros-melodic-pcl-ros
     ```
   * Kinetic
     ```bash
     sudo apt install ros-kinetic-pcl-ros
     ```

1. Build the package:

   ```bash
   cd ~/catkin_ws
   catkin build px4_realsense_bridge
   ```

1. Run the ROS node:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   roslaunch px4_realsense_bridge bridge_mavros.launch
   ```

  > **Note** This launch file starts the mavros node as well. Mavros needs to be running but if it is started elsewhere the basic launch file *bridge.launch* can be used. The file *bridge_mavros_sitl.launch* is only for using this node in combination with the PX4 SITL toolchain (Simulation). Instructions on how to setup the PX4 SITL toolchain can be found [here](http://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies). 
