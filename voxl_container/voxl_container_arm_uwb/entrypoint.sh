#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Configurate the right FastDDS version
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml" >> ~/.bashrc

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# Source the local package
echo "source /ws/install/setup.bash" >> ~/.bashrc
source /ws/install/setup.bash 

# & ros2 run director director_node & ros2 run director coordinate_publisher & 
# Execute the command passed into this entrypoint
# true if variable is unset or == ""
if [ -z "$@" ]
then 
    ros2 run director director_node &
    ros2 run director coordinate_publisher &
    MicroXRCEAgent udp4 -p 8888
else
    exec "$@"
fi

exec "$@"
#export ROS_DOAMIN_ID="$@"
