#!/usr/bin/env python
'''
Node to implement dead reckoning estimation.

'''

# System imports
from math import *

# ROS/Gazebo imports
import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import FluidPressure

def decibars2depth_salt(decibars,latitude):
    '''
    From Seabird application note 69 (AN69).
    Return depth in meters
    Latitude is given in units of decimal degrees
    '''
    p = decibars
    x = ( sin(latitude/57.29578) )**2
    g = 9.780318*(1.0+( 5.2788e-3 + 2.36e-5 * x) * x ) + 1.092e-6 * p
    d = ((((-1.82e-15 * p+2.279e-10)*p-2.2512e-5)*p+9.72659)*p) / g
    return d

def pascals2depth(pascals):
    ''' 
    Wrapper function with latitude hardcoded for now
    '''
    decibars = pascals/1.0e5
    return decibars2depth_salt(decibars, 41.0)

def  mdeglat(lat):
    '''
    Provides meters-per-degree latitude at a given latitude
    From AlvinXY 

    Args:
      lat (float): latitude
    Returns:
      float: meters-per-degree value
    '''
    latrad = lat*2.0*pi/360.0 ;

    dy = 111132.09 - 566.05 * cos(2.0*latrad) \
         + 1.20 * cos(4.0*latrad) \
         - 0.002 * cos(6.0*latrad)
    return dy

def mdeglon(lat):
    '''
    Provides meters-per-degree longitude at a given latitude
    From AlvinXY

    Args:
      lat (float): latitude in decimal degrees
    Returns:
      float: meters per degree longitude
    '''
    latrad = lat*2.0*pi/360.0 
    dx = 111415.13 * cos(latrad) \
         - 94.55 * cos(3.0*latrad) \
	+ 0.12 * cos(5.0*latrad)
    return dx

class Node():
    def __init__(self, init_lat, init_lon, angleofattack_deg):

        # Current estimate
        self.dr_msg = NavSatFix()
        self.dr_msg.latitude = init_lat
        self.dr_msg.longitude = init_lon
        
        # Fixec AOA
        self.angleofattack_rad = angleofattack_deg * pi/180.0
        
        # Keep track of time and depth
        self.t0 = rospy.get_time()
        while self.t0 < 0.01:
            rospy.logwarn("Waiting for ROS tme to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()
        self.d0 = None
        # Flag for when we receive GPS updates
        self.new_gps = False

        # Init messages
        self.imu_msg = Imu()
        self.pressure_msg = FluidPressure()
        self.gps_msg = NavSatFix()
        
        # Pubs and subs
        self.sub_imu = rospy.Subscriber("imu", Imu, self.callback_imu)
        self.sub_pressure = rospy.Subscriber("pressure",
                                             FluidPressure,
                                             self.callback_pressure)
        self.sub_gps = rospy.Subscriber("fix", NavSatFix, self.callback_gps)
        self.pub_fix = rospy.Publisher("deadreckon", NavSatFix, queue_size=1)

        
    def callback_imu(self, data):
        self.imu_msg = data
        
    def callback_pressure(self, data):
        self.pressure_msg = data

    def callback_gps(self, data):
        self.gps_msg = data
        self.new_gps = True
        
    def update(self):
        '''
        Estimate
        '''
        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-3:
            rospy.logwarn("Timestep is too small (%f) - skipping this update"
                          %dt)
            return
        self.t0 = now
                
        # Convert fluid pressure to depth
        depth = pascals2depth(self.pressure_msg.fluid_pressure)

        # Need to intialize depth on first pass
        if self.d0 is None:
            self.d0 = depth
            return

        # Convert IMU attitude to Euler
        q = (self.imu_msg.orientation.x,
             self.imu_msg.orientation.y,
             self.imu_msg.orientation.z,
             self.imu_msg.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(q)

        # If we have GPS - just use that
        if self.new_gps:
            self.dr_msg.latitude = self.gps_msg.latitude
            self.dr_msg.longitude = self.gps_msg.longitude
            self.dr_msg.header.stamp = rospy.Time.now()
            self.pub_fix.publish(self.dr_msg)
            return
            
        # Algo. from:
        # https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/3
        dz = depth - self.d0
        self.d0 = depth
        # Assume constant angle of attack
        # Note that coordinates are in ENU
        aoa = 4.0 * pi/180.0
        glide_angle = rpy[1] + aoa
        if abs(glide_angle) < pi/100.0:
            v_x = 0.0
            v_y = 0.0
        else: 
            v_x = dz/(dt*tan(glide_angle))*sin(rpy[2])
            v_y = dz/(dt*tan(glide_angle))*cos(rpy[2])
        dx = v_x * dt
        dy = v_y * dt

        # Increment lat/lon 
        self.dr_msg.latitude += dy/mdeglat(self.dr_msg.latitude)
        self.dr_msg.longitude += dx/mdeglat(self.dr_msg.latitude)

        # Publish
        self.dr_msg.header.stamp = rospy.Time.now()
        self.pub_fix.publish(self.dr_msg)
        return
    
if __name__ == '__main__':

    # Start node
    rospy.init_node('deadreckoning_estimator')

    # Initial lat/lon.  Default is in Buzzard's Bay
    init_lat = rospy.get_param('~init_lat', 41.5501)
    init_lon = rospy.get_param('~init_lon', -70.71428)
    # Specified in degrees, positive is bow down
    angleofattack_deg = rospy.get_param('~angleofattack',  -4.0 )
    
    # Update rate
    update_rate = rospy.get_param('~update_rate', 5.0)
    
    # Initiate node object
    node=Node(init_lat, init_lon, angleofattack_deg)

    # Spin
    r = rospy.Rate(update_rate)
    try:
        while not rospy.is_shutdown():
            node.update()
            r.sleep()
    except rospy.ROSInterruptException:
        pass