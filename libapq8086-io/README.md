# libapq8086-io

This guide shows how to setup the VOXL board and upload the parser. The parser is a fundamental service that i developed to enable the MicroXRCE communication between the firmware and the VOXL, it uses the *libapq8096* dependencies and functions to work.

## 0. VOXL setup

1. **Set up ADB following this [guide](https://docs.modalai.com/setting-up-adb/).**

1. I suggest to update the SDK (Software Development Kit) of the VOXL at the version 1.12.x following this [guide](https://docs.modalai.com/flash-system-image/) (Is not necessary to configure the board for a particular SKU), i don't know if the next steps or the overall architecture works with older versions.

3. Connect to the board through an USB--type-c cable.

    Enter to the system with the commands:

    ```bash
    adb shell
    ```

    and then:

    ```bash
    bash
    ```

    Now that we are inside the board run:

    ```bash
    voxl-configure-docker-support
    ```
    (If you are in doubt during the configuration say yes to all)

    This resets and activates the services *docker-daemon* and *docker-autorun*.  

4. Activation of the wi-fi connection, it can be done by running this command on the host system (the board have to be connected to the system):

    ```bash
    adb shell voxl-wifi station <SSID> <Password>
    ```

    Then, after rebooting the device, the connection should work.

## 1. Parser upload

The library should be already builded but, if you want to modify something and rebuild it, you have to follow these steps:

1. the build procedure requires the voxl-hexagon (>=1.1) docker image, follow the voxl-docker [instructions](https://gitlab.com/voxl-public/voxl-docker) to download it

2. Launch the voxl-hexagon docker image in the libvoxl-io source directory.

        ~/libvoxl_io$ voxl-docker -i voxl-hexagon
        user@57f6e83bba92:~$

3. Install dependencies inside the docker. Specify the dependencies should be pulled from either the development (dev) or stable modalai package repos. If building the master branch you should specify stable, otherwise dev.

        ./install_build_deps.sh apq8096 staging

4. Run build.sh inside the docker.

        user@57f6e83bba92:~$ ./build.sh

To upload the library with the parser you have to:

1. Generate a deb or ipk package inside the docker.

        user@57f6e83bba92:~$ ./make_package.sh ipk

2. You can now push the ipk package to the VOXL and install with opkg however you like. To do this over ADB, you may use the included helper script: deploy_to_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.
        
        ~/libvoxl_io$ ./deploy_to_voxl.sh

All these instruction can be found in the original [repository](https://gitlab.com/voxl-public/voxl-sdk/core-libs/libapq8096-io) of the *libapq8096* library.

## 2. Parser setup

(TODO)

### The parser setup is complete, the next step is the build and upload of the VOXL container, the instructions can be found [here](https://github.com/GiacomoCaciagli/Software_Design_For_UAV_Applications_in_GNSS-DENIED_Environments/blob/main/voxl_container/README.md) 