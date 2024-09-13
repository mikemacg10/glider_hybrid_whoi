#!/usr/bin/env python3

import rospy
import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-interactive plotting
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Proj, transform


from gazebo_msgs.msg import LinkStates

# Define UTM projection (here we use UTM zone 33N as an example)
proj_utm = Proj(proj='utm', zone=20, ellps='WGS84', datum='WGS84')

# Define WGS84 projection (standard lat/long system)
proj_latlon = Proj(proj='latlong', datum='WGS84')
# from utils import circle_minus

class GliderStatusPlotter:
    def __init__(self):
        self.basePose = None
        self.fig, self.ax = plt.subplots(1, 1)

    def callback(self, data):
        # self.yaw_values.append(np.rad2deg((msg.heading)))

        baseIndex = data.name.index('glider_hybrid_whoi::glider_hybrid_whoi/base_link')
        basePosition = data.pose[baseIndex].position

        if self.basePose is None:
            self.basePose = np.array([[basePosition.x, basePosition.y, basePosition.z]])
        else:
            # append the new data to the np array 
            new_pose = np.array([[basePosition.x, basePosition.y, basePosition.z]])
            self.basePose = np.vstack((self.basePose, new_pose))

        self.ax.clear()
        self.ax.plot(self.basePose[:, 0], self.basePose[:, 1])
        self.ax.set_title('X vs Y')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        plt.tight_layout()
        plt.savefig('src/glider_hybrid_whoi/glider_cva_localization/PLOTS/state.png')  # Save the plot to a file

        # with open('src/glider_hybrid_whoi/glider_cva_localization/PLOTS/state.csv', 'w') as f:
        #     f.write('x,y\n')
        #     f.write(f'{self.basePose[0]},{self.basePose[1]}\n')

    def listen(self):
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('glider_status_plotter')
    plotter = GliderStatusPlotter()
    plotter.listen()