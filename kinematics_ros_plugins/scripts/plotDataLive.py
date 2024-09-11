#!/usr/bin/env python3

import rospy
import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-interactive plotting
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Proj, transform


from frl_vehicle_msgs.msg import UwGliderStatus

# Define UTM projection (here we use UTM zone 33N as an example)
proj_utm = Proj(proj='utm', zone=20, ellps='WGS84', datum='WGS84')

# Define WGS84 projection (standard lat/long system)
proj_latlon = Proj(proj='latlong', datum='WGS84')
# from utils import circle_minus

class GliderStatusPlotter:
    def __init__(self):
        self.yaw_values = []
        self.pitch_values = []
        self.roll_values = []
        self.depth_values = []
        self.time_values = []
        self.lat = []
        self.lon = []
        self.x = []
        self.y = []
        self.displacment = 0   
        self.fig, self.ax = plt.subplots(4, 1)
        # self.fig2, self.ax2 = plt.subplots(2, 1)

    def callback(self, msg):
        self.yaw_values.append(np.rad2deg((msg.heading)))
        self.pitch_values.append(np.rad2deg((msg.pitch)))
        self.roll_values.append(np.rad2deg((msg.roll)))
        self.depth_values.append((msg.depth))
        self.time_values.append((msg.header.stamp.to_sec()))
        self.lat.append(msg.latitude)
        self.lon.append(msg.longitude)

        # convert lat and lon to x and y
        x, y = transform(proj_latlon, proj_utm, msg.longitude, msg.latitude)
        self.x.append(x)
        self.y.append(y)
        print(len(self.x))
        if len(self.x) < 1:
            self.displacment = 0
        else:
            self.displacment += np.sqrt((self.x[0] - self.x[1])**2 + (self.y[0] - self.y[1])**2)
        print(f'Displacment: {self.displacment}')


        self.ax[0].clear()
        self.ax[0].plot(self.time_values, self.yaw_values)
        self.ax[0].set_title('heading vs Time')
        self.ax[0].set_xlabel('Time (s)')
        self.ax[0].set_ylabel('heading')

        self.ax[1].clear()
        self.ax[1].plot(self.time_values, self.pitch_values)
        self.ax[1].set_title('Pitch vs Time')
        self.ax[1].set_xlabel('Time (s)')
        self.ax[1].set_ylabel('Pitch')
        
        self.ax[2].clear()
        self.ax[2].plot(self.time_values, self.roll_values)
        self.ax[2].set_title('Roll vs Time')
        self.ax[2].set_xlabel('Time (s)')
        self.ax[2].set_ylabel('Roll')

        self.ax[3].clear()
        self.ax[3].plot(self.time_values, self.depth_values)
        self.ax[3].set_title('Depth vs Time')
        self.ax[3].set_xlabel('Time (s)')
        self.ax[3].set_ylabel('Depth')

        plt.tight_layout()
        plt.savefig('/home/michael/uuv_ws/src/glider_hybrid_whoi/kinematics_ros_plugins/IMAGES/state.png')  # Save the plot to a file

        # self.ax2[0].clear()
        # self.ax2[0].plot(self.x, self.y)
        # self.ax2[0].set_title('Lat vs Lon')
        # self.ax2[0].set_xlabel('x')
        # self.ax2[0].set_ylabel('y')

        # plt.tight_layout()
        # plt.savefig('/home/michael/uuv_ws/src/glider_hybrid_whoi/kinematics_ros_plugins/IMAGES/XY.png')  # Save the plot to a file


        # save to csv
        with open('/home/michael/uuv_ws/src/glider_hybrid_whoi/kinematics_ros_plugins/IMAGES/state.csv', 'w') as f:
            f.write('time,heading,pitch,roll,depth,lat,lon\n')
            for i in range(len(self.time_values)):
                f.write(f'{self.time_values[i]},{self.yaw_values[i]},{self.pitch_values[i]},{self.roll_values[i]},{self.depth_values[i]},{self.lat[i]},{self.lon[i]}\n')

    def listen(self):
        rospy.Subscriber('/glider_hybrid_whoi/kinematics/UwGliderStatus', UwGliderStatus, self.callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('glider_status_plotter')
    plotter = GliderStatusPlotter()
    plotter.listen()