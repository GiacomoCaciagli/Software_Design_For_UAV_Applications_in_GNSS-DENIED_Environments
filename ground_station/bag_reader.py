import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import Stores, get_typestore, get_types_from_idl, get_types_from_msg
from pathlib import Path
import statistics

# read definition
msg_text0 = Path('src/px4_msgs/msg/VehicleOdometry.msg').read_text()
msg_text1 = Path('src/px4_msgs/msg/SensorCombined.msg').read_text()
msg_text2 = Path('src/px4_msgs/msg/TrajectorySetpoint.msg').read_text()

# Plain dictionary to hold messae definition
add_types = {}

# Add definition from msg file to dict
add_types.update(get_types_from_msg(msg_text0,'px4_msgs/msg/VehicleOdometry'))
add_types.update(get_types_from_msg(msg_text1,'px4_msgs/msg/SensorCombined'))
add_types.update(get_types_from_msg(msg_text2,'px4_msgs/msg/TrajectorySetpoint'))

typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

def bag_reader(filename):

    time = []

    x = [] # in meters
    y = [] # in meters
    z = [] # in meters

    w = []
    q1 = []
    q2 = []
    q3 = []

    vx = []
    vy = []
    vz = []

    with Reader('bags/'+filename) as reader:
        # topic and msgtype information is available on .connections list

        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            #print(connection.topic, connection.msgtype)

            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            #print(msg.timestamp) #q:w,1,2,3

            time.append((msg.timestamp))

            x.append(float(msg.position[0]))
            y.append(float(msg.position[1]))
            z.append(float(msg.position[2]))

            w.append(float(msg.q[0]))
            q1.append(float(msg.q[1]))
            q2.append(float(msg.q[2]))
            q3.append(float(msg.q[3]))

            vx.append(float(msg.velocity[0]))
            vy.append(float(msg.velocity[1]))
            vz.append(float(msg.velocity[2]))
    
    return time,x,y,z,w,q1,q2,q3,vx,vy,vz

def sensor_bag_reader(filename):

    time = []

    gx = [] # in meters
    gy = [] # in meters
    gz = [] # in meters

    ax = []
    ay = []
    az = []

    with Reader('bags/'+filename) as reader:
        # topic and msgtype information is available on .connections list

        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            #print(connection.topic, connection.msgtype)

            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            #print(msg.timestamp) #q:w,1,2,3

            time.append((msg.timestamp))

            gx.append(float(msg.gyro_rad[0]))
            gy.append(float(msg.gyro_rad[1]))
            gz.append(float(msg.gyro_rad[2]))

            ax.append(float(msg.accelerometer_m_s2[0]))
            ay.append(float(msg.accelerometer_m_s2[1]))
            az.append(float(msg.accelerometer_m_s2[2]))
    
    return time,gx,gy,gz,ax,ay,az

def trajectory_bag_reader(filename):

    time = []

    x = [] # in meters
    y = [] # in meters
    z = [] # in meters

    with Reader('bags/'+filename) as reader:
        # topic and msgtype information is available on .connections list

        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            #print(connection.topic, connection.msgtype)

            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            #print(msg.timestamp) #q:w,1,2,3

            time.append((msg.timestamp))

            x.append(float(msg.position[0]))
            y.append(float(msg.position[1]))
            z.append(float(msg.position[2]))

    return time,x,y,z
    
def main():

    file = "uwb_slow_mission1"

    filename_input = "input_data_" + file
    time_input,x_input,y_input,z_input,w_input,q1_input,q2_input,q3_input,vx_input,vy_input,vz_input = bag_reader(filename_input)

    filename_output = "output_data_" + file
    time_output,x_output,y_output,z_output,w_output,q1_output,q2_output,q3_output,vx_output,vy_output,vz_output = bag_reader(filename_output)

    
    filename_setpoint = "setpoint_" + file
    time_setpoint,x_setpoint,y_setpoint,z_setpoint = trajectory_bag_reader(filename_setpoint)
    
    
    filename_vicon = "vicon_data_" + file
    time_vicon,x_vicon,y_vicon,z_vicon,w_vicon,q1_vicon,q2_vicon,q3_vicon,vx_vicon,vy_vicon,vz_vicon = bag_reader(filename_vicon)
    
    '''
    filename_sensor = "sensor_data_" + file
    time_sensor,gx,gy,gz,ax,ay,az = sensor_bag_reader(filename_sensor)
    '''
    #print(time_input[0],time_output[0],time_vicon[0])
    #print(time_input[len(time_input)-1],time_output[len(time_output)-1],time_vicon[len(time_vicon)-1])
    #print(time_input[len(time_input)-1]-time_input[0],time_output[len(time_output)-1]-time_output[0],time_vicon[len(time_vicon)-1]-time_vicon[0])
    #print(time_vicon[0]-time_output[0])

    start = time_input[0]
    for i in range(len(time_input)):
        time_input[i] = (time_input[i]-start)/1000000
        if (time_input[i]<=0):
            time_input[i]=time_input[0]
        
    start = time_output[0]
    for i in range(len(time_output)):
        time_output[i] = (time_output[i]-start)/1000000
        if (time_output[i]<=0):
            time_output[i]=time_output[0]
    
    '''
    start = time_setpoint[0]
    for i in range(len(time_setpoint)):
        time_setpoint[i] = (time_setpoint[i]-start)/1000000
        if (time_setpoint[i]<=0):
            time_setpoint[i]=time_setpoint[0]
    '''
    
    for i in range(len(time_vicon)):
        time_vicon[i] = (time_vicon[i]-start)/1000000
        if (time_vicon[i]<=0):
            time_vicon[i]=0
    
    '''
    for i in range(len(time_sensor)):
        time_sensor[i] = (time_sensor[i]-start)/1000000
        if (time_sensor[i]<=0):
            time_sensor[i]=0
    '''
    
    fig, (g1,g2,g3) = plt.subplots(3)
    print(time_output[0])

    g1.plot(time_input,x_input, label='x_input')
    g1.plot(time_output,x_output, label='x_output')
    g1.plot(time_vicon,x_vicon, label='x_vicon')

    g2.plot(time_input,y_input, label='y_input')
    g2.plot(time_output,y_output, label='y_output')
    g2.plot(time_vicon,y_vicon, label='y_vicon')

    g3.plot(time_input,z_input, label='z_input')
    g3.plot(time_output,z_output, label='z_output')
    g3.plot(time_vicon,z_vicon, label='z_vicon')

    g1.legend()
    g2.legend()
    g3.legend()
    
    #plt.show()
    
    fig, (g4,g5,g6,g7) = plt.subplots(4)

    g4.plot(time_input,w_input, label='w_input')
    g4.plot(time_output,w_output, label='w_output')
    #g4.plot(time_vicon,w_vicon, label='w_vicon')

    g5.plot(time_input,q1_input, label='q1_input')
    g5.plot(time_output,q1_output, label='q1_output')
    #g5.plot(time_vicon,q1_vicon, label='q1_vicon')

    g6.plot(time_input,q2_input, label='q2_input')
    g6.plot(time_output,q2_output, label='q2_output')
    #g6.plot(time_vicon,q2_vicon, label='q2_vicon')

    g7.plot(time_input,q3_input, label='q3_input')
    g7.plot(time_output,q3_output, label='q3_output')
    #g7.plot(time_vicon,q3_vicon, label='q3_vicon')

    g4.legend()
    g5.legend()
    g6.legend()
    g7.legend()

    '''
    fig, (g8,g9,ga) = plt.subplots(3)

    g8.plot(time_setpoint,x_setpoint, label='gx')

    g9.plot(time_setpoint,y_setpoint, label='gy')

    ga.plot(time_setpoint,z_setpoint, label='gz')

    g8.legend()
    g9.legend()
    ga.legend()
    '''
    '''
    fig, (g8,g9,ga) = plt.subplots(3)

    g8.plot(time_sensor,gx, label='gx')

    g9.plot(time_sensor,gy, label='gy')

    ga.plot(time_sensor,gz, label='gz')

    g8.legend()
    g9.legend()
    ga.legend()

    fig, (gb,gc,gd) = plt.subplots(3)

    gb.plot(time_sensor,ax, label='ax')

    gc.plot(time_sensor,ay, label='ay')

    gd.plot(time_sensor,az, label='az')

    gb.legend()
    gc.legend()
    gd.legend()
    
    fig, (ge,gf,gh) = plt.subplots(3)

    ge.plot(time_input,vx_input, label='vx')

    gf.plot(time_input,vy_input, label='vy')

    gh.plot(time_input,vz_input, label='vz')

    ge.legend()
    gf.legend()
    gh.legend()
    
    '''
    plt.show()

    for i in range(len(time_output)):
        if time_output[i] > 10.5:
            init = i
            break
    for i in range(len(time_output)):
        if time_output[i] > 50:
            end = i
            break

    print(init,end)

    '''
    fig, (g1,g2) = plt.subplots(2)

    g1.plot(time_output[init:end],x_output[init:end], label="x_output")
    g1.plot(time_input[init:end],x_input[init:end], label="x_input")

    g2.plot(time_output[init:end],y_output[init:end], label="y_output")
    '''
    min_x = min(x_output[init:end])
    max_x = max(x_output[init:end])
    min_y = min(y_output[init:end])
    max_y = max(y_output[init:end])

    x_output = a = [abs(x) for x in x_output[init:end]]
    y_output = a = [abs(x) for x in y_output[init:end]]

    print("X position\nAverage error: ", statistics.mean(x_output)," Min: ", min_x," Max: ", max_x)
    print("Y position\nAverage error: ", statistics.mean(y_output)," Min: ", min_y," Max: ", max_y)

    print(max(x_input[init:end]))
    #g2.plot(time_output[init:end],y_output, label="y_output_abs")

    

if __name__ == "__main__":
    main()



