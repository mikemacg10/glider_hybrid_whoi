#!/usr/bin/env python3

import rospy
import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-interactive plotting
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Proj, transform


from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped

# Define UTM projection (here we use UTM zone 33N as an example)
proj_utm = Proj(proj='utm', zone=20, ellps='WGS84', datum='WGS84')

# Define WGS84 projection (standard lat/long system)
proj_latlon = Proj(proj='latlong', datum='WGS84')
# from utils import circle_minus

class GliderStatusPlotter:
    def __init__(self):
        self.basePose = None
        self.targetPose = None
        self.whalePose = None
        self.fig, self.ax = plt.subplots(1, 1)
        self.fig_error, self.ax_error = plt.subplots(1, 1)

    def callback(self, data):
        baseIndex = data.name.index('glider_hybrid_whoi::glider_hybrid_whoi/base_link')
        basePosition = data.pose[baseIndex].position

        if self.basePose is None:
            self.basePose = np.array([[basePosition.x, basePosition.y, basePosition.z]])
        else:
            # append the new data to the np array 
            new_pose = np.array([[basePosition.x, basePosition.y, basePosition.z]])
            self.basePose = np.vstack((self.basePose, new_pose))

        if self.basePose is None or self.targetPose is None or self.whalePose is None:
            return

        self.ax.clear()
        self.ax.plot(self.basePose[:, 0], self.basePose[:, 1], label='Base Pose')
        self.ax.plot(self.targetPose[:, 0], self.targetPose[:, 1], 'r*', label='Target Pose')
        self.ax.plot(self.whalePose[:, 0], self.whalePose[:, 1], label='Whale Pose')
        self.ax.set_title('X vs Y')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()

        plt.tight_layout()
        plt.savefig('src/glider_hybrid_whoi/glider_cva_localization/PLOTS/state.png')  # Save the plot to a file

        if not hasattr(self, 'listOfError'):
            self.listOfError = []

        error = np.linalg.norm(self.whalePose[-1, :2] - self.targetPose[-1, :2])

        if len(self.listOfError) > 0 and error == self.listOfError[-1]:
            return
        else:
            self.listOfError.append(error)

        # Plot error vs time
        self.ax_error.clear()
        self.ax_error.plot(range(len(self.listOfError)), self.listOfError, label='Error over Time')
        self.ax_error.set_title('Error vs Time')
        self.ax_error.set_xlabel('Time')
        self.ax_error.set_ylabel('Error')
        self.ax_error.legend()

        plt.tight_layout()
        plt.savefig('src/glider_hybrid_whoi/glider_cva_localization/PLOTS/error.png')  # Save the error plot to a file







    def listen(self):
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        rospy.Subscriber("/glider_hybrid_whoi/CVALocalization", PoseStamped, self.pose_stamped_callback)
        rospy.Subscriber("/whalePose/estimatedPose", PoseStamped, self.whale_pose_stamped_callback)
        rospy.spin()

    def pose_stamped_callback(self, data):
        if self.targetPose is None:
            self.targetPose = np.array([[data.pose.position.x, data.pose.position.y, data.pose.position.z]])
        else:
            # append the new data to the np array 
            new_pose = np.array([[data.pose.position.x, data.pose.position.y, data.pose.position.z]])
            self.targetPose = np.vstack((self.targetPose, new_pose))

    def whale_pose_stamped_callback(self, data):
        if self.whalePose is None:
            self.whalePose = np.array([[data.pose.position.x, data.pose.position.y, data.pose.position.z]])
        else:
            # append the new data to the np array 
            new_pose = np.array([[data.pose.position.x, data.pose.position.y, data.pose.position.z]])
            self.whalePose = np.vstack((self.whalePose, new_pose))

if __name__ == '__main__':
    rospy.init_node('glider_status_plotter')
    plotter = GliderStatusPlotter()
    plotter.listen()