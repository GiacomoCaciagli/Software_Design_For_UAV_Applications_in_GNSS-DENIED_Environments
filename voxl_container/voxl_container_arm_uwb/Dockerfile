ARG FROM_IMAGE=arm64v8/ros:humble-ros-base
ARG PKG_NAME=ros_template
ARG PKG_WS=/ws

######### CLONING and INSTALLING #################
FROM $FROM_IMAGE AS base
SHELL ["/bin/bash", "-c"]

# Install the main dependences
# ARG define a variable that users can pass at build time (when use the docker command)
ARG PKG_WS 
WORKDIR $PKG_WS/src/$PKG_NAME
RUN apt-get update -q && \
    sudo apt-get install net-tools && \
    sudo apt-get install -y iputils-ping && \
    rm -rf /var/lib/apt/lists/*

ARG PKG_WS
WORKDIR $PKG_WS
RUN apt-get update && \
    rosdep install -y --from-paths ./src --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# Build all the packages
ARG PKG_WS
WORKDIR $PKG_WS
COPY ./src $PKG_WS/src

COPY ./tools/prebuild ${PKG_WS}/..
#COPY ./tools/unbuild ${PKG_WS}/..
RUN cd ${PKG_WS}/../Micro-XRCE-DDS-Agent && \
    #mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig /usr/local/lib

RUN mkdir ${PKG_WS}/build
    COPY ./tools/ros_build_materials/build ${PKG_WS}/build
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" 

# Setup the entry point and config file
# ARG ros_template
COPY ./entrypoint.sh /
COPY ./fastdds.xml /root/.ros

# Set the entrypoint
RUN chmod 777 /entrypoint.sh

# Set the dev arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Add dev dependences
RUN apt-get update && \
    rm -rf /var/lib/apt/lists/*

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME && \
    useradd --uid ${UID} --gid ${GID} --create-home ${USERNAME} && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    mkdir -p /home/${USERNAME} && \
    chown -R ${UID}:${GID} /home/${USERNAME}

# Setup the access right
ARG PKG_WS
RUN chown -R ${UID}:${GID} ${PKG_WS}

# Create the fastdds.xml configuration
ARG PKG_NAME
RUN mkdir /home/${USERNAME}/.ros 
COPY ./fastdds.xml /home/${USERNAME}/.ros
RUN chown -R ${UID}:${GID} /home/${USERNAME}/.ros

# Configuration for terminator
#COPY tools/terminator_config /home/${USERNAME}/.config/terminator/config
ENV ROS_DOMAIN_ID=20
RUN export ROS_DOMAIN_ID=ROS_DOMAIN_ID

USER ${USERNAME}
RUN echo "source /entrypoint.sh && source /etc/bash_completion" >> /home/${USERNAME}/.bashrc
#RUN source ros2 run director director_node &
#RUN source ros2 run director coordinate_publisher &
ENTRYPOINT [ "/entrypoint.sh" ]


