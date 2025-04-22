# Error handling

This guide lists the main problems that i encountered developing and running this application. I remind you that this application take a while to set up, so, after connecting the battery to the drone you have to wait at least 20 to 30 seconds (sometimes more than 1 minute) to check if everything is correct and working. The simplest checks are:

1. run the following command and check amount of topics:

        ros2 topic list

2. run the following command and check if you receive something:

    ```bash
    ros2 topic echo /fmu/in/vehicle_visual_odometry
    ```

## 1. No topic seen

In this case there are two possibilities:

1. the system were you ran the command and the VOXL doesn't see each other 

    in this case you have to restart the network turning off and on the router.

2. the container didn't start

    in this case the *docker daemon* or the *docker autorun* didn't start, you have to restart the VOXL (and maybe the flight controller).

(Another possibility is that you checked the topics before the automatic setup finished)

## 2. Only 6 */fmu* topic seen

In this case the parser didn't start or the MicroXRCE connection didn't work.
To resolve the parser problem you have to:

1. go into the VOXL with:

        adb shell

        bash

    or

        ssh root@<VOXL ip>

    (The password is oelinux123)

2. execute the command:

        systemctl start voxl-parser

Now you can exit from the VOXL system and everything should work fine.

To resolve the MicroXRCE connection you have to reboot both the VOXL and the flight controller.

## 3. No messages are published over the */fmu/in/vehicle_visual_odometry* topic

The solution of this problem depends on which image are you using.

If you are using the UWB image there could be a connection issue or the system that sends the messages to the *coordinate publisher* is not working.

If you are using the VICON image, this problem can happen in two cases:

1. The node didn't connect to the VICON stream, in this case you have to reboot the system (and maybe the flight controller).

    Be sure that the container is started when the VICON stream is already up.

2. The node that connects to the VICON stream didn't start, in this case you have to:

    1. go into the VOXL with:

        adb shell

        bash

    or

        ssh root@<VOXL ip>

    (The password is oelinux123)

    2. enter the container with:

        docker exec <container_name> -it bash

    3. inside the container run:

        (TODO)

    if the result of the previous command is *Transition successful* you can exit from the container and the VOXL and everything should be fine, if not you can manually start the node inside the container with:

        (TODO) &

    and then, when the command is end, execute again the command:

        (TODO)

    if this doesn't work you have to reboot the system