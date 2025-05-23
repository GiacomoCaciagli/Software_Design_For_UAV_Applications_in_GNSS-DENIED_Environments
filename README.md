# Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments

This repository contains a thesis work about the design and implementation of a UAV application for indoor flight, it contains all the necessary code to execute the application. This application works using ROS2, PX4 v1.15.2 and MicroXRCE, it was developed for a drone composed by a VOXL and a kakute h7 but the architecture can work with any drone. The  This work was developed in collaboration with LINKS foundation.

The following guide explains how to setup the hardware and upload the software to create the architecture shown at the end of the list:

1. [Firmware setup](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/Firmware_setup.md)

2. [Parser upload and setup](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/libapq8086-io/README.md)

3. [Docker image build and setup](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/voxl_container/README.md)

4. [Ground station build and activation](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/ground_station/README.md)


![Alt text](general_architecture.png)

## Quickstart

When the setup is complete everything should work automatically.

To check if the architecture has been built correctly, you have to execute the commmand:

```bash
ros2 topic list
```

The system needs time to set everything up so it could take a while to see something, anyway, as a result of the previous command you should see many topics that start with **/fmu/in** and **/fmu/out**, these are the firmware topics.

To check if everything is working fine execute:

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry
```

If everything is fine you should see a stream of *VehicleOdometry* messages.

If something wrong, like not seeing many topics or not seeing the stream of messages, refer to this [guide](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/Error_handling.md).

Now that the application is working you have to start the *Command_publisher* node with:

```bash
ros2 run control_node command_publisher
```

Now you can start sending commands and control the drone.
The commands are single lowercase words and you can send only one command at a time, a list of all commands currently available is [here](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/Commands.md) (the list indicates the order to start a flight). Every command will be executed after pressing *Enter* with the exception of the commands *go* and *yaw* that require more parameters.
