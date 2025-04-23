# voxl_container

This guide explains how to build, upload and set the docker image necessary to make the application work. This folder contains two different docker images for two different scenarios but their build and upload procedure is the same.

[comment]: <> (This folder contains two different docker images, they differ only for the *coordinate publisher* node, in the VICON folder the *coordinate publisher* is programmed to receive the VICON data stream, in the other folder the node expects to receive a UDP message containing the required data. Their build and upload procedure is the same)

## 1. Build and export the image

To build the image go, from terminal, into one of the two folders in this folder and execute:

    docker build --platform linux/arm64 -t <image_name> .

This would take a while, even two hours in worst case. When the image is built, launch it with:

    form linux/arm64 --rm --net=host --ipc=host --user devuser --privileged -it <image_name> /entrypoint.sh bash

Now, in another terminal, export the container with:

    docker export <container_name> <file_name>.tgz

(The container name, if not known, can be found using *tab*).

When this command ends, you can stop the container.

## 2. upload and create the image on the VOXL

For this part of the guide i suppose that the VOXL is already setup, if not, follow point 0 of this [guide](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/libapq8086-io/README.md).

Connect to the VOXL using a cable.

Inside the folder where you exported the container run this command:

    adb push <file_name>.tgz /data

(*/data* can be any folder on the VOXL)
This process will take usually 10 to 15 minutes to complete, after that, enter the VOXL:

    adb shell

    bash

or

    ssh root@<VOXL ip>

(The password is oelinux123)

Once connected to the VOXL go to the folder where you saved the exported container and run:

    docker import <file_name>.tgz <image_name>

This will create the image on the VOXL, this <image_name> parameter can be different from the previous one and will be used for the next steps.

## 3. Set the image to run on start-up

Now that the VOXL has the image to run, to set it to run on start-up is necessary to:

0. go into the VOXL with:

        adb shell

        bash

    or

        ssh root@<VOXL ip>

    (The password is oelinux123)

1. open the file executed by the *docker-autorun* service with:

        vi /etc/modalai/docker-autorun-script.sh 

2. copy these lines into the file

        #!/bin/bash
        ## /etc/modalai/docker-autorun-script.sh
        
        # This script is called by docker-autorun.service on boot (if enabled)
        # Feel free to modify this file as you see fit to do whatever you want
        # this is just a starting point
        # Make sure these are run in non-interactive mode! e.g. use -n option with
        # voxl-docker or dont use -it with docker run
        
        ## Hello World Example
        docker stop <container_name>
        docker rm <container_name>
        sleep 10
        
        docker run --rm --privileged --net=host --name <container_name> -v /home/root:/root/yoctohome/:rw -w /root/ <image_name> /entrypoint.sh

    If you want to change the image to run on start-up you have to change the <image_name> parameter.

    Save and exit (*esc -> :wq*)

### Now the desired image will automatically run at the start-up of the board, the last step is to download the *command publisher* node, the guide to do it is [here](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/ground_station/README.md).
