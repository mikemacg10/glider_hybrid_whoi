import rospy
import time
import math
import numpy as np
from frl_vehicle_msgs.msg import UwGliderCommand
from sensor_msgs.msg import NavSatFix
from uuv_sensor_ros_plugins_msgs.msg import DVL

#!/usr/bin/env python3



class GliderController:
    def __init__(self):
        rospy.init_node('commander', anonymous=True)
        self.pub = rospy.Publisher('/glider_hybrid_whoi/kinematics/UwGliderCommand', UwGliderCommand, queue_size=10)
        rospy.Subscriber("deadreckon", NavSatFix, self.callback_pressure)
        rospy.Subscriber("/glider_hybrid_whoi/altimeter", DVL, self.callback_DVL)
        self.rate = rospy.Rate(10) # 10hz
        self.startTime = rospy.get_time()

    def callback_pressure(self, data):
        depth = data.altitude
        print(depth)

    def callback_DVL(self, data):
        altitude = data.altitude
        print(altitude)

    def command(self):
        x = np.arange(-math.pi, math.pi, 0.1)
        for i in x:
            command = UwGliderCommand()
            command.header.stamp = rospy.Time.now()
            command.pitch_cmd_type = 1
            command.target_pitch_value = -0.4
            command.target_pumped_volume = 100.0
            command.rudder_control_mode = 1
            command.target_heading = 0
            command.motor_cmd_type = 1
            command.target_motor_cmd = 1.0
            rospy.loginfo(command)
            self.pub.publish(command)
            time.sleep(10)

        # Shutdown
        rospy.signal_shutdown('\n\nDONE!')

if __name__ == '__main__':
    try:
        controller = GliderController()
        controller.command()
    except rospy.ROSInterruptException:
        pass
