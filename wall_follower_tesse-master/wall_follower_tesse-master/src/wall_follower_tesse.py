#!/usr/bin/env python2

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tesse_ros_bridge.msg import CollisionStats
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

import numpy as np
from pid import PID

# TODO: integrate `ros_utils` into workspace `racecar_ws`.
# ros_utils import PID

class WallFollower:
	# get constants from params file
	SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
	DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
	ODOMETRY_TOPIC = rospy.get_param("wall_follower/odometry_topic")
	SIDE = rospy.get_param("wall_follower/side")
	VELOCITY = rospy.get_param("wall_follower/velocity")
	DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
	TRACK = rospy.get_param("wall_follower/track")

	def __init__(self):
		# Publishers
		self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 20)
		self.wall_pub = rospy.Publisher("/wall_marker", Marker, queue_size = 10)
		# self.pose_pub = rospy.Publisher("/initialpose", PoseStamped, queue_size = 10)

		# Subscribers
		self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
		# self.odom_sub = rospy.Subscriber(self.ODOMETRY_TOPIC, Odometry, self.odom_callback)
		# self.imu_sub = rospy.Subscriber("/tesse/imu/clean/imu", Imu, self.imu_callback)
		# self.collision_sub = rospy.Subscriber("/tesse/collision", CollisionStats, self.collision_callback)

		# PID controller
		self.controller = PID(0.001, 0.0, 0.0)
        # can switch to minimum or more "linear-type" regression....
        # set laser scans to be agnostic to position... like may not hardcoding search spots?

	# def laser_range_average(self, ranges):
    #     return np.average(ranges)

	def scan_lin_reg(self, scan_msg):
		#''' Returns the slope and y-intercept (m, b) of the least squares regression line. '''
		ranges_s = np.array(scan_msg.ranges)
		angles_s = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

		n = len(ranges_s)
		if self.SIDE == -1: # right_follow
		    i_s = np.arange(0, 2*n/3)
		elif self.SIDE == +1: # left_follow
		    i_s = np.arange(n/3, n)

		ranges_s = ranges_s[i_s]
		angles_s = angles_s[i_s]

		ranges = np.array(scan_msg.ranges)

		# 1a. Filter out likely outliers
		is_nearby = np.where(ranges_s < 3.0*min(ranges_s))
		if len(is_nearby) >= 2:
		   ranges_s = ranges_s[is_nearby]
		   angles_s = angles_s[is_nearby]

		# 2. Find scan positions (x, y)_s in R^2 (wrt sensor s)
		x_s = ranges_s*np.cos(angles_s)
		y_s = ranges_s*np.sin(angles_s)

		# 3. Perform linear regression on the remaining positions p_r (LSLR)
		m, b = np.polyfit(x_s, y_s, 1)

		return m, b

	def scan_callback(self, scan_msg):
		# In robot frame r
		wall_m, wall_b = self.scan_lin_reg(scan_msg)
		x_0 = -wall_b/wall_m

		# Heading: angle between current pose and wall
		desired_heading = np.arctan2(-self.SIDE*self.DESIRED_DISTANCE, abs(x_0))
		# rospy.loginfo("Desired Heading:\t%f deg", desired_heading/np.pi*180.0)

		# Publish AckermannDriveStamped to /tesse/drive
		drive_msg = AckermannDriveStamped()
		drive_msg.header.frame_id = "base_link_gt"
		drive_msg.header.stamp = rospy.get_rostime()

		drive_msg.drive.steering_angle = -self.SIDE * self.controller.eval(0.0, reference=desired_heading)
		# drive_msg.drive.steering_angle_velocity = 0.0
		drive_msg.drive.speed = self.VELOCITY
		# drive_msg.drive.acceleration = 0.0
		# drive_msg.drive.jerk = 0.0
		self.drive_pub.publish(drive_msg)

		# Publish Marker to visualization_marker
		marker = Marker()
		marker.header.frame_id = "base_link_gt"
		marker.header.stamp = rospy.get_rostime()
		marker.type = Marker.LINE_STRIP
		marker.action = Marker.ADD;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = wall_b;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		self.wall_pub.publish(marker)

	def odom_callback(self, odom_msg):
		""" Odometry: Callback """
		pass

	def imu_callback(self, imu_msg):
		""" Inertial Measuement Unit: Callback """
		pass

	def collision_callback(self, stats_msg):
		''' Collision Stats: Callback
		Consider printing collisons using loginfo. '''
		pass


if __name__ == "__main__":
	rospy.init_node("wall_follower_tesse")
	stalker = WallFollower()
	rospy.spin()
