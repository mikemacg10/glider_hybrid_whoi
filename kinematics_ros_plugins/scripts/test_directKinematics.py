#!/usr/bin/env python3

import rospy
import time
import math
from frl_vehicle_msgs.msg import UwGliderCommand

import matplotlib.pyplot as plt
#import pandas as pd

# make a list from 0 to 2*pi with 0.1 step
x = [i/10 for i in range(0, 63)]
print(x)

def command(startTime):
    while not rospy.is_shutdown():
        if rospy.get_time()-startTime >= 1:  # initiate after 2 seconds
            # dummy initial command
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            pub.publish(command)
            time.sleep(1)

            # print("\n----- Descend with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle)------")
            # command = UwGliderCommand()
            # command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = -0.4
            command.target_pumped_volume = -100.0
            command.rudder_control_mode = 2
            command.rudder_angle = 3
            command.target_heading = math.pi/2.0
            command.motor_cmd_type = 1
            command.target_motor_cmd = 100.0
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(45)

            print("\n----- Ascend with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle)------")
            # command = UwGliderCommand()
            # command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = -0.4
            command.target_pumped_volume = 100.0
            command.rudder_control_mode = 2
            command.rudder_angle = 2
            command.target_heading = math.pi/2
            command.motor_cmd_type = 1
            command.target_motor_cmd = 1.0
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            print("\n----- Descend with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle)------")
            # command = UwGliderCommand()
            # command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = 0.4
            command.target_pumped_volume = -100.0
            command.rudder_control_mode = 2
            command.rudder_angle = 1
            command.target_heading = math.pi/2
            command.motor_cmd_type = 1
            command.target_motor_cmd = 1.0
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)

            print("\n----- Ascend with Pitch control (Batt pos) + Buoyancy engine + Rudder control (angle)------")
            # command = UwGliderCommand()
            # command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = -0.4
            command.target_pumped_volume = 100.0
            command.rudder_control_mode = 1
            command.target_heading = math.pi/2
            command.motor_cmd_type = 1
            command.target_motor_cmd = 1.0
            rospy.loginfo(command)
            pub.publish(command)
            time.sleep(5)


                # Shutdown
            rospy.signal_shutdown('\n\nDONE!')

        # rate.sleep()

if __name__ == '__main__':
    try:
        # start node
        pub = rospy.Publisher('/glider_hybrid_whoi/kinematics/UwGliderCommand', UwGliderCommand, queue_size=10)
        rospy.init_node('commander', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        startTime = rospy.get_time()

        # send command
        command(startTime)

    except rospy.ROSInterruptException:
        pass
