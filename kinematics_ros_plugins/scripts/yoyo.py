#!/usr/bin/env python3


import rospy
import time
import math
import numpy as np
from frl_vehicle_msgs.msg import UwGliderCommand
from sensor_msgs.msg import NavSatFix, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL



class GliderController:
    def __init__(self):
        self.pub = rospy.Publisher('/glider_hybrid_whoi/kinematics/UwGliderCommand', UwGliderCommand, queue_size=10)
        rospy.Subscriber("deadreckon", NavSatFix, self.callback_pressure)
        rospy.Subscriber("/glider_hybrid_whoi/altimeter", DVL, self.callback_DVL)
        self.rate = rospy.Rate(10) # 10hz
        self.startTime = rospy.get_time()
        self.pitch = np.deg2rad(-20)
        self.oil = -100.0

    def callback_pressure(self, data):
        print(f"Depth: {data.altitude}")
        if data.altitude > -20:
            print("GO DOWN")
            self.pitch = np.deg2rad(20)
            self.oil = -200.0
            self.command()
        elif data.altitude < -50:
            print("GO UP")
            self.pitch = np.deg2rad(-20)
            self.oil = 200.0
            self.command()
        
    def callback_DVL(self, data):
        print(data.altitude)
        if data.altitude != -1 and data.altitude < 20:
            print("GO UP")
            self.pitch = np.deg2rad(-0)
            self.oil = 200.0
            self.command()

    def command(self):
        # dummy initial command
        command = UwGliderCommand()
        command.header.stamp = rospy.Time.now()
        self.pub.publish(command)
        
        # while not rospy.is_shutdown():

        command = UwGliderCommand()
        command.header.stamp = rospy.Time.now()
        command.pitch_cmd_type = 3
        command.target_pitch_value = self.pitch # pos of Battery
        command.target_pumped_volume = self.oil # vol of Oil
        command.rudder_control_mode = 2
        command.rudder_angle = 4
        command.target_rudder_angle = np.deg2rad(0)
        command.target_heading = 0
        command.motor_cmd_type = 1
        command.target_motor_cmd = 75.0
        self.pub.publish(command)
        print("Published")

        # # Shutdown
        # rospy.signal_shutdown('\n\nDONE!')

def main(args=None):
    rospy.init_node('commander', anonymous=True)
    yoyo = GliderController()
    yoyo.command()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass