#!/usr/bin/env python

"""
This script is used to localize the glider using the CVA method.

Author: Michael MacGillivray
Date: 2024-09-12

"""


import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import time
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix 
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import LinkStates
from frl_vehicle_msgs.msg import UwGliderStatus

from modules.core.TDOA import tdoa, hydrophonePairOrintation, calcAmbigousBearings, make_combos, min_var, kmeans, getRangeToTarget, find_best_match, generate_lookup_table
from modules.core.CC import generateSignal, correlateSignals, HilbertTransform, addNoise, delaySignal


class Localizer:
    def __init__(self):
        self.whaleData = pd.read_csv('/home/michael/uuv_ws/src/JASCODATA/CJM_North Atlantic right whale_1d_4_set0_Scenario1.csv')
        # subtract the larhest value in the x (m), y (m) and depth (m) columns from the respective columns
        # just for testing purposes
        # TODO: remove this when the real data is used, will remake the csv soon. 
        self.whaleData['x (m)'] = self.whaleData['x (m)'] - min(self.whaleData['x (m)']) + 500
        self.whaleData['y (m)'] = self.whaleData['y (m)'] - min(self.whaleData['y (m)']) + 500
        self.whaleData['depth (m)'] = self.whaleData['depth (m)']
        self.bearings = np.arange(180, -181, -1)

        self.targetPose = np.array([0, 0, 0])
        self.pub = rospy.Publisher('/glider_hybrid_whoi/CVALocalization', PoseStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/whalePose/estimatedPose', PoseStamped, queue_size=10)
        # rospy.Subscriber("deadreckon", NavSatFix, self.callback_pressure)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.callbackHydrophoneState)
        rospy.Subscriber("/glider_hybrid_whoi/kinematics/UwGliderStatus", UwGliderStatus, self.robotStateCallback)
        self.startTime = rospy.get_time()
        
        self.whalePub = rospy.Timer(rospy.Duration(5), self.pubWhaleData)

        self.check = rospy.Timer(rospy.Duration(10), self.timer_callback)

        self.falseAlarmTimer = rospy.Timer(rospy.Duration(20), self.falseAlarm)

        self.hydrophoneState = []
        #TODO: MAKE ONE VAR FOR GLIDER POSE
        self.basePose = np.array([0, 0, 0])
        self.gliderHeading = 0
        
        # Generate a signal for testing
        self.signal = generateSignal(48000, 1000, 2000, 2, 0.75)

        


    def timer_callback(self, _):

        # TODO: Implement cross correlation, TODA and ambigous bearing calculations

        TDOA_true, toa_true = tdoa(self.hydrophoneState, self.targetPose)
        hydrophonePairAngles, hydrophonePairDisplacment = hydrophonePairOrintation(self.hydrophoneState, self.basePose)
        TDOA = self.completeCrossCorrelation(toa_true)

        ambigousBearing, weightDict = calcAmbigousBearings(TDOA, hydrophonePairAngles, hydrophonePairDisplacment)

        if np.isnan(ambigousBearing).sum() > 3:
            print("More than 3 values are NaN")
            print("No estimate for this call")
            return
        
        timeNow = time.time()
        combos = make_combos(3, ambigousBearing)

        # use Chris Widdes metheod to find the min var bearing
        minVar  = min_var(combos, weightDict)
        print(f"Time to run Min Var: {time.time() - timeNow}")
        print(f"Min Var: {minVar}\n")

        # # Use Lookup Table TDOA solver
        timeNow = time.time()
        self.lookupTable = generate_lookup_table(self.hydrophoneState, self.bearings)
        searchAOA = find_best_match(self.lookupTable, TDOA)
        print(f"Time to run Search: {time.time() - timeNow}")
        print(f"Search AOA: {searchAOA + self.gliderHeading}\n")

        # get range to target for localization purposes
        rangeToTarget = getRangeToTarget(self.basePose, self.targetPose)
        errorRange = np.random.normal(0, rangeToTarget/5)
        rangeToTarget += errorRange
        
        # use Kmean to find the best bearing
        # TODO: Find a better way to add the number of clusters and max iterations
        timeNow = time.time()
        clusterCenters = kmeans(ambigousBearing, 8, 10)
        print(f"Time to run Kmeans: {time.time() - timeNow}")
        print(f"Cluster Centers: {clusterCenters}\n")
        

        angle = np.rad2deg(np.arctan2(self.targetPose[0] - self.basePose[0], 
                                                self.targetPose[1] - self.basePose[1]))
        
        print(f"True Angle: {angle}")

        estimatedPose = self.completeLocalization(self.basePose, minVar, rangeToTarget) 

        print(f"Angular Error: {angle - minVar}\n")



    def callbackHydrophoneState(self, data):
      # Callback fn to continuously update the hydrophone state
      # Data will be used for ambigous bearing calcs.
      #TODO: set name of the robot in the launch to spawn multiple. 
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
        #   position.x -= basePosition.x
        #   position.y -= basePosition.y
        #   position.z -= basePosition.z
          hydrophonePosition = [position.x, position.y, position.z]

          positions.append(hydrophonePosition)
          
      self.hydrophoneState = (positions)

    def robotStateCallback(self, data):
        # save glider heading"
        self.gliderHeading = data.heading

        
    def completeCrossCorrelation(self, TOA):
        # use the true TOA to generate signals for the cross correlation calculation

        TOA = TOA - max(TOA)
        TOA = TOA + np.random.uniform(-0.0005, 0.0005, 4)
        signals = np.array([delaySignal(self.signal, delay, 48000) for delay in TOA])
        # add noise to signals
        signals = np.array([addNoise(signal, np.random.uniform(10, 10)) for signal in signals])

        TDOA = np.array([correlateSignals(signals[i], signals[j]) for i in range(len(signals)) for j in range(i+1, len(signals))])
        return TDOA
    
    def completeLocalization(self, basePose, minVar, rangeToTarget):
        # use the minVar and rangeToTarget to estimate the target pose
        # this will be used to test the state estimation

        # get the x and y position of the target
        x = basePose[0] + rangeToTarget * np.sin(np.deg2rad(minVar))
        y = basePose[1] + rangeToTarget * np.cos(np.deg2rad(minVar))
        z = basePose[2] + self.targetPose[2]

        estimatedPose = PoseStamped()
        estimatedPose.pose.position.x = x
        estimatedPose.pose.position.y = y
        estimatedPose.pose.position.z = z

        self.pub.publish(estimatedPose)

        return np.array([x, y, z])

    def falseAlarm(self, _):
        # send a solved bearing through that is very incorrect
        # this will be used to test the state estimation
        incorrectBearing = np.random.uniform(0, 360)
        rangeToTarget = getRangeToTarget(self.basePose, self.targetPose) 
        rangeToTarget += np.random.normal(0, rangeToTarget/2)

        print("False Positive detection Sent")
        self.completeLocalization(self.basePose, incorrectBearing, rangeToTarget) 

    
    def pubWhaleData(self, _):
        # read the rossim time
        time = rospy.get_time()
        # pull the row in the whale data that is closest to the current time
        data = self.whaleData.iloc[(self.whaleData['time (s)']-time).abs().argsort()[:1]]
        targetPose = PoseStamped()
        targetPose.pose.position.x = data['x (m)'].values[0]
        targetPose.pose.position.y = data['y (m)'].values[0]
        targetPose.pose.position.z = data['depth (m)'].values[0]

        self.pub2.publish(targetPose)


        self.targetPose = np.array([targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z])



def main():
    rospy.init_node('commander', anonymous=True)
    _ = Localizer()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass