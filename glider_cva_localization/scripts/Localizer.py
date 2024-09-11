#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import time
import math
import numpy as np
from sensor_msgs.msg import NavSatFix 
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import LinkStates
from frl_vehicle_msgs.msg import UwGliderStatus

from modules.core.TDOA import tdoa, hydrophonePairOrintation, calcAmbigousBearings, make_combos, min_var

class Localizer:
    def __init__(self):
        self.pub = rospy.Publisher('/glider_hybrid_whoi/CVALocalization', PoseStamped, queue_size=10)
        # rospy.Subscriber("deadreckon", NavSatFix, self.callback_pressure)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.callbackHydrophoneState)
        rospy.Subscriber("/glider_hybrid_whoi/kinematics/UwGliderStatus", UwGliderStatus, self.robotStateCallback)
        self.startTime = rospy.get_time()
        self.check = rospy.Timer(rospy.Duration(2), self.timer_callback)

        self.hydrophoneState = []
        # TODO: MAKE ONE VAR FOR GLIDER POSE
        self.basePose = np.array([0, 0, 0])
        self.gliderHeading = 0

    def timer_callback(self, _):
        # for now use a static target position for testing
        targetPose = PoseStamped()
        targetPose.pose.position.x = 1000
        targetPose.pose.position.y = 5000
        targetPose.pose.position.z = -100

        targetPose = np.array([targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z])

        # TODO: Implement cross correlation, TODA and ambigous bearing calculations

        TDOA = tdoa(self.hydrophoneState, targetPose)
        hydrophonePairAngles, hydrophonePairDisplacment = hydrophonePairOrintation(self.hydrophoneState)
        # print(f"Angles pairs: {np.rad2deg(hydrophonePairAngles)}")
        ambigousBearing = calcAmbigousBearings(TDOA, hydrophonePairAngles, hydrophonePairDisplacment)
        combos = make_combos(3, 12, ambigousBearing)
        minVar = min_var(combos)
        print(f"Min Var: {minVar}")
        angle = np.rad2deg(np.arctan2(targetPose[0] - self.basePose[0], 
                                                     targetPose[1] - self.basePose[1]))
        print(f"Angle: {angle}")



    def callbackHydrophoneState(self, data):
      # Callback fn to continuously update the hydrophone state
      # Data will be used for ambigous bearing calcs. 
      hydrophone_names = ["glider_hybrid_whoi::glider_hybrid_whoi/hydrophone_1", "glider_hybrid_whoi::glider_hybrid_whoi/hydrophone_2",
                           "glider_hybrid_whoi::glider_hybrid_whoi/hydrophone_3", "glider_hybrid_whoi::glider_hybrid_whoi/hydrophone_4"]
      positions = []
      # pull gase base position
      baseIndex = data.name.index('glider_hybrid_whoi::glider_hybrid_whoi/base_link')
      basePosition = data.pose[baseIndex].position

      self.basePose = np.array([basePosition.x, basePosition.y, basePosition.z])

      for name in hydrophone_names:
          position_index = data.name.index(name)
          position = data.pose[position_index].position
          position.x -= basePosition.x
          position.y -= basePosition.y
          position.z -= basePosition.z
          hydrophonePosition = np.array([position.x, position.y, position.z])

          positions.append(hydrophonePosition)
          
      self.hydrophoneState = (positions)

    def robotStateCallback(self, data):
        # save glider heading"
        self.gliderHeading = data.heading

        
    def completeCrossCorrelation(self):      
        pass
    
    def completeLocalization(self):
        pass
    

def main():
    rospy.init_node('commander', anonymous=True)
    _ = Localizer()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass