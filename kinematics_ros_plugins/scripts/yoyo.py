#!/usr/bin/env python3


import rospy
import time
import math
import numpy as np
from frl_vehicle_msgs.msg import UwGliderCommand, UwGliderStatus
from sensor_msgs.msg import NavSatFix, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL



class GliderController:
    def __init__(self):
        self.pub = rospy.Publisher('/glider_hybrid_whoi/kinematics/UwGliderCommand', UwGliderCommand, queue_size=10)
        rospy.Subscriber("deadreckon", NavSatFix, self.callback_pressure)
        rospy.Subscriber("/glider_hybrid_whoi/altimeter", DVL, self.callback_DVL)
        rospy.Subscriber("/glider_hybrid_whoi/kinematics/UwGliderStatus", UwGliderStatus, self.robotStateCallback)
        # self.rate = rospy.Rate(10) # 10hz
        self.startTime = rospy.get_time()
        self.pitch = np.deg2rad(-20)
        self.oil = 1000.0
        self.thrust = 0.0
        self.gliderHeading = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.depth = 0
        
        # create a list of goals        
        self.goal_lat = -3
        self.goal_lon = -5
        self.Kp = -0.2
        self.rudderAngle = 0

        self.check = rospy.Timer(rospy.Duration(4), self.timer_callback)

    def timer_callback(self, event):
        # I think this should update goals based on state estimation
        goal_bearing = self.calculate_bearing(self.lat, self.lon, self.goal_lat, self.goal_lon)
        deltaHeading =  goal_bearing - self.gliderHeading
        print(f"Delta Heading: {np.rad2deg(deltaHeading)}")

        # # start with a P controller for heading
        if abs(deltaHeading) > np.deg2rad(1):
            self.rudderAngle = self.Kp * deltaHeading
            print(f"Rudder Angle: {self.rudderAngle}")
        elif abs(deltaHeading) < 1:
            self.rudderAngle = 0 

        # clamp the rudder angle
        if self.rudderAngle > np.deg2rad(20):
            self.rudderAngle = np.deg2rad(20)
        elif self.rudderAngle < np.deg2rad(-20):
            self.rudderAngle = np.deg2rad(-20)

        # in state estimation, we should send the goal to the controller for heading
        # and depth
        #self.pitch = np.deg2rad(-20)
        #self.oil = self.oil

        self.thrust = 0.50
        self.command()
        pass


    def robotStateCallback(self, data):
        # save glider heading"
        self.gliderHeading = (data.heading)

    def callback_pressure(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.alt = data.altitude

        if self.alt > -2:
            print("GO DOWN")
            self.pitch = np.deg2rad(25)
            self.oil = -1500.0
            self.thrust = 1.0
            self.command()
        elif self.alt < -100:
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
        command.rudder_control_mode = 2
        command.rudder_angle = 4
        command.target_rudder_angle = self.rudderAngle # rudder angle

        # command.rudder_control_mode = 1
        # command.target_heading = np.deg2rad(270)
        command.motor_cmd_type = 1
        command.target_motor_cmd = self.thrust
        self.pub.publish(command)
        print("Published")

        # # Shutdown
        # rospy.signal_shutdown('\n\nDONE!')

    # # Function to calculate change in heading
    # def heading_change(robot_lat, robot_lon, robot_heading, goal_lat, goal_lon):
    #     goal_bearing = self.calculate_bearing(robot_lat, robot_lon, goal_lat, goal_lon)
    #     heading_diff = goal_bearing - robot_heading
    #     heading_diff = (heading_diff + 180) % 360 - 180  # Normalize to [-180, 180]
    #     return heading_diff

    
    # Function to calculate bearing between two lat-lon points
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = np.sin(dlon) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
        bearing = (np.arctan2(x, y))
        return (bearing + 2*np.pi) %  2*np.pi



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