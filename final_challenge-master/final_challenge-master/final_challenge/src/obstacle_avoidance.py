#!/usr/bin/env python2

import numpy as np

import tf
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from ackermann_msgs.msg import AckermannDriveStamped

class ObstacleAvoidance:
    
	SCAN_TOPIC = rospy.get_param("final_challege/scan_topic", "/tesse/front_lidar/scan")
	DRIVE_TOPIC = rospy.get_param("final_challege/drive_topic", "/tesse/drive")
	VELOCITY = rospy.get_param("final_challenge/velocity","0.5")
	ODOM_TOPIC = rospy.get_param("final_challenge/odom_topic", "/tesse/odom")
	COLLISION_TOPIC = rospy.get_param("final_challenge/collision_topic", "/tesse/collision")
	
        def __init__(self):
		# Initialize your publishers and subscribers here
		self.steering_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 10)
		self.laser_data = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.laser_callback)
		self.odom_data = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odom_callback)
		#self.collision_data = rospy.Subscriber(self.COLLISION_TOPIC, CollisionStats, self.collision_callback)
		self.p_robot = None
		self.heading = None
		self.current_pos = None
		self.wheelbase_length   = 2.5
		self.map_from_world = np.array([166.45+25.3482,-232.95-5.6846])
		self.goal_point = -np.array([290, -215]) + self.map_from_world
		

	def odom_callback(self, data):
		(self.current_pos, self.heading) = self.pose_from_msg(data.pose.pose)
		self.current_pos = -self.current_pos + self.map_from_world
		self.heading = self.heading + np.pi
		#print("current pos" , self.current_pos)

	def laser_callback(self, data):
		#make ranges a numpy array
		self.ranges_np = np.array(data.ranges)	

		#angles in degrees 
		self.angles = (180.0/np.pi)*np.arange(data.angle_min, data.angle_max, data.angle_increment)

                #this needs to be tuned
		offset = 8
		side_offset = 20
		dist_from_barrier = 6
		side_dist_from_barrier = 6
		barrier_left = False
		barrier_right = False
		angle = self.evaluate(self.goal_point)
                straight_velocity = 13
		velocity = straight_velocity
		turn_velocity = 13
		angle_threshold = np.pi/4
                turn_angle = np.pi/3
                if self.current_pos[0]:
			x_coord = self.current_pos[0]
			y_coord = self.current_pos[1]

                # downsample lidar and angles so there aren't as many measurements
		self.ranges_np = self.ranges_np[0:self.ranges_np.size: 10]
		self.angles = self.angles[0:self.angles.size:10]

		self.length = len(self.ranges_np)

                #check if there is a barrier in front, taking the minimum of lidar in front
		front_min = np.min(self.ranges_np[(self.length/2)-offset:(self.length/2)+offset:1]) 
		
		front_min_index = np.where(self.ranges_np == front_min)
		front_min_angle = self.angles[front_min_index]	

		#Obstacle Avoidance
		if (front_min < dist_from_barrier):
			
			# don't need to look behind the car so done look at full length
                        left_min = np.min(self.ranges_np[self.length/4:(self.length/2)-side_offset:1])			
			left_max = np.max(self.ranges_np[self.length/4:(self.length/2)-side_offset:1])
			
			right_min = np.min(self.ranges_np[(self.length/2)+side_offset:(3*self.length)/4:1])
			right_max = np.max(self.ranges_np[(self.length/2)+side_offset:(3*self.length)/4:1])
			
			if (front_min < 2):
				angle = -angle
				velocity = -straight_velocity
			elif (front_min_angle[0] < -angle_threshold):
				
				#if there is a barrier to the right, and the left side is open				
				if (left_min > side_dist_from_barrier):
					print('turn left, BARRIER TO RIGHT and left is open')
					angle = turn_angle
					velocity = turn_velocity
				# if there is a barrier to the right, and the left side is not open				
				else:
					angle = -turn_angle
					velocity = turn_velocity
			elif (front_min_angle[0] > angle_threshold):
				#if barrier to the left, and right side is open
				if (right_min > side_dist_from_barrier):
					print('turn right, BARRIER TO LEFT and right is open')
					angle = -turn_angle
					velocity = turn_velocity
				else:
					angle = turn_angle
					velocity = turn_velocity
			else:
				if (right_max > left_max):
					print('turn right')
				        angle = -turn_angle
					velocity = turn_velocity
				else:
					print('turn left')
					angle = turn_angle
					velocity = turn_velocity
			'''
                        elif (front_min_angle[0] < -angle_threshold):
				
				#if there is a barrier to the right, and the left side is open				
				if (left_min > side_dist_from_barrier):
					print('turn left, BARRIER TO RIGHT and left is open')
					angle = turn_angle
					velocity = turn_velocity
				# if there is a barrier to the right, and the left side is not open				
				else:
					angle = -turn_angle
					velocity = turn_velocity
			elif (front_min_angle[0] > angle_threshold):
				#if barrier to the left, and right side is open
				if (right_min > side_dist_from_barrier):
					print('turn right, BARRIER TO LEFT and right is open')
					angle = -turn_angle
					velocity = turn_velocity
				else:
					angle = turn_angle
					velocity = turn_velocity
                        else:
				if (right_min > left_min):
					print('turn right')
				        angle = -turn_angle
					velocity = turn_velocity
				else:
					print('turn left')
					angle = turn_angle
					velocity = turn_velocity
			'''
		print('angle', angle)
		print('velocity', velocity)
		self.publish_steering(angle, velocity)
			
	#HELPER FUNCTIONS
	def evaluate(self, p_goal):
		''' Find a steering angle that will approach the transient goal.
		'''
		dp = p_goal - self.current_pos
		d = np.linalg.norm(dp)
		theta = np.arctan2(dp[1], dp[0]) - self.heading
		# Steering angle: delta = arctan( 2 L sin(theta) / d )
		delta = np.arctan2(2*self.wheelbase_length*np.sin(theta), d)
		print "PurePursuit.evaluate\td: %3.3f,\ttheta: %.3f" % (d, theta)
		return delta * 0.3
	
	def publish_steering(self, angle, velocity):
		drive = AckermannDriveStamped()
		drive.header.stamp = rospy.get_rostime()
		drive.drive.steering_angle = angle
		drive.drive.steering_angle_velocity = 0		
		drive.drive.speed = velocity
		drive.drive.acceleration = 0
		drive.drive.jerk = 0
		self.steering_pub.publish(drive)

	def pose_from_msg(self, msg):
		'''
        	Returns a 1x2 array [x, y] and the yaw angle from a ROS message of type
        	geometry_msgs.msg.Pose.
		'''
		pos = np.array([0.0, 0.0])
		pos[0] = msg.position.x
		pos[1] = msg.position.y
		q = msg.orientation
		_, _, yaw = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
		return (pos, yaw)


if __name__ == "__main__":
	rospy.init_node('obstacle_avoidance')
	ObstacleAvoidance()
	rospy.spin()
