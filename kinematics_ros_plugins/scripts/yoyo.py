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
        # self.rate = rospy.Rate(10) # 10hz
        self.startTime = rospy.get_time()
        self.pitch = np.deg2rad(-20)
        self.oil = 1000.0
        self.thrust = 0.0
        self.check = rospy.Timer(rospy.Duration(4), self.timer_callback)

    def timer_callback(self, event):
        # I think this should update goals based on state estimation

        # in state estimation, we should send the goal to the controller for heading
        # and depth
        #self.pitch = np.deg2rad(-20)
        #self.oil = self.oil

        self.thrust = 0.50
        self.command()
        pass

    def callback_pressure(self, data):
        if data.altitude > -2:
            print("GO DOWN")
            self.pitch = np.deg2rad(25)
            self.oil = -1500.0
            self.thrust = 1.0
            self.command()
        elif data.altitude < -70:
            print("GO UP")
            self.pitch = np.deg2rad(-25)
            self.oil = 1500.0
            self.thrust = 1.0
            self.command()
        
    def callback_DVL(self, data):
        if data.altitude != -1 and data.altitude < 10:
            print("GO UP")
            self.pitch = np.deg2rad(-25)
            self.oil = 1000.0
            self.thrust = 1.0
            self.command()

    def command(self):      
        # while not rospy.is_shutdown():

        command = UwGliderCommand()
        command.header.stamp = rospy.Time.now()
        command.pitch_cmd_type = 3
        command.target_pitch_value = self.pitch # pitch angle
        command.target_pumped_volume = self.oil # vol of Oil
        # command.rudder_control_mode = 2
        # command.rudder_angle = 4
        # command.target_rudder_angle = np.deg2rad(1)

        command.rudder_control_mode = 1
        command.target_heading = np.deg2rad(270)
        command.motor_cmd_type = 1
        command.target_motor_cmd = self.thrust
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