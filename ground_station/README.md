# Ground station

This folder contains the ROS2 node which a user interacts to plus other nodes and codes usefull for data analysis.
These nodes were developed for **ROS2 Humble**.

## 0. ROS2 setup

All nodes contained in this folder needs ROS2 installed on your system, to setup the ROS2 environment refer to the official [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (this link brings to the linux instructions).

## 1. Nodes build and activation

All nodes can be builded running the following command inside the *ground_station* folder

    colcon build

(it takes 10 minutes to build everything)
After the build, set up the environment with:

    source install/local_setup.bash

Now you can run the nodes.
To run the *command publisher* node execute

    ros2 run control_nodes command_publisher

then you have to insert the commands and control the drone

### If you followed all the guides you are now ready to conrtol your drone, before the flight i suggest you to check if everything is fine, the simplest checks are explained in the *Quickstart* section of the main [guide](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/README.md)
