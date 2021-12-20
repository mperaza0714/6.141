#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
from math import radians, degrees
import numpy as np
import threading
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

class ParticleFilter:

    def __init__(self):
        self.max_particles = 200
        self.prev_theta = 0
        self.particles = np.zeros((self.max_particles, 3))
        self.probs = np.ones(self.max_particles)
        self.probs/=np.sum(self.probs)
        self.prev_time = rospy.Time.now()
        self.lock = threading.RLock()

        self.pf_listener = tf.TransformListener()

        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame", "/base_link_pf")

        # Initialize the models
        self.sensor_model = SensorModel()
        self.motion_model = MotionModel()

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.laser_callback,
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odometry_callback,
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_initilization_callback,
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

       # self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, self.handle_transform, queue_size = 1)
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

        self.particle_pub = rospy.Publisher("/pf/pose/particles", PoseArray, queue_size = 1)
        self.average_marker = rospy.Publisher("/pf/pose/particles_average", Marker, queue_size = 1)




    def quaternion2angle(self, q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
        return yaw


        # pose initilization callback function
    def pose_initilization_callback(self, pose_data):
        #self.particles = np.zeros((self.max_particles,3))

        self.particles[:,0] = pose_data.pose.pose.position.x + np.random.normal(0.0,0.5,self.max_particles) # loc, scale, size
        self.particles[:,1] = pose_data.pose.pose.position.y + np.random.normal(0.0,0.5,self.max_particles)
        self.particles[:,2] = self.quaternion2angle(pose_data.pose.pose.orientation) + np.random.normal(0.0,0.5,self.max_particles)
        #self.prev_theta = pose_data.pose.pose.orientation
        self.prev_time = pose_data.header.stamp
        # odometry callback function

    def odometry_callback(self, odom_data):
        #pass
        # TODO: this call to evaluate may be incorrect
       # print("In odometry_callback")
        self.timestamp = odom_data.header.stamp
        x = odom_data.twist.twist.linear.x
        y = odom_data.twist.twist.linear.y
        theta = odom_data.twist.twist.angular.z
        dt = (odom_data.header.stamp - self.prev_time).to_sec()
        self.prev_time = odom_data.header.stamp
        xdt = x*dt
        ydt = y*dt
        thetadt = theta*dt

        self.odom = [xdt, ydt, thetadt]

        with self.lock:
            self.particles = self.motion_model.evaluate(self.particles, self.odom)
            print("in odometry lock")
            # self.pub_mlc()
        #self.laser_callback(self.laser_data1)

    # laser callback function
    def laser_callback(self, laser_data):
       # print("In laser_callback")
        if self.sensor_model.map_set:
            self.timestamp = laser_data.header.stamp
            self.observation = np.asarray(laser_data.ranges)

            self.probs = self.sensor_model.evaluate(self.particles, self.observation)
          #  self.probs/= np.sum(self.probs) movig to pub_mlc
            self.pub_mlc()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz




    def mean_angle(self, theta):
        x = np.sum(np.sin(theta))/len(theta)
        y = np.sum(np.cos(theta))/len(theta)
        rad = np.arctan2(x, y)
        return rad
        #return degrees(phase(sum(rect(1, radians(d)) for d in deg)/len(deg)))

    def get_average(self, particles):
        x = np.mean(particles[:,0])
        y = np.mean(particles[:,1])
        theta_list = particles[:,2]
        #theta = round(self.mean_angle(theta_list), 4)
        theta = self.mean_angle(theta_list)
        return [x, y, theta]

    def handle_transform(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((self.average[0], self.average[1], 0),
                    tf.transformations.quaternion_from_euler(0, 0, self.average[2]),
                    self.timestamp,
                    self.particle_filter_frame,
                    "/map")


    def get_highest_prob_particle(self):
        highest_prob = np.where(self.probs == np.amax(self.probs))
        highest_prob_particle = self.particles[highest_prob][0]
        return highest_prob_particle

    def downsample(self):
        index = np.arange(0, self.max_particles, 1)
        self.probs /= np.sum(self.probs)
        self.downsample_particles_index = np.random.choice(index, self.max_particles, True, self.probs)
        self.particles = self.particles[self.downsample_particles_index,:]
        self.average = self.get_average(self.particles)

        # self.particle = self.particles + np.random.normal(0.0,0.5,(self.max_particles, 3))

    def pub_mlc(self):
        print("in mlc before lock")
        with self.lock:
            print("in mlc in lock")
            self.downsample()
            highest_prob_particle =  self.get_highest_prob_particle()
            #if
            self.average = highest_prob_particle

        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = self.timestamp
        self.odom_msg.header.frame_id = "/map"
        self.odom_msg.pose.pose.position.x = self.average[0]
        self.odom_msg.pose.pose.position.y = self.average[1]
        quaternion_mcl = tf.transformations.quaternion_from_euler(0, 0, self.average[2])
        self.odom_msg.pose.pose.orientation.x = quaternion_mcl[0]
        self.odom_msg.pose.pose.orientation.y = quaternion_mcl[1]
        self.odom_msg.pose.pose.orientation.z = quaternion_mcl[2]
        self.odom_msg.pose.pose.orientation.w = quaternion_mcl[3]

        self.odom_pub.publish(self.odom_msg)
        # self.handle_transform()
        self.publish_markers()

        # listener = tf.TransformListener()
        # (trans,rot) = listener.lookupTransform(self.particle_filter_frame, '/map', self.prev_time)

    def publish_markers(self):
        poses_array = PoseArray()
        poses_array.header.stamp = rospy.Time.now()
        poses_array.header.frame_id = "/map"
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, p[2])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            poses_array.poses.append(pose)
        self.particle_pub.publish(poses_array)
        marker = Marker()
        marker.header.frame_id = "/map"
        #print(self.average)
        marker.pose.position.x = self.average[0]
        marker.pose.position.y = self.average[1]
        quaternion_pt = tf.transformations.quaternion_from_euler(0, 0, self.average[2])
        marker.pose.orientation.x = quaternion_pt[0]
        marker.pose.orientation.y = quaternion_pt[1]
        marker.pose.orientation.z = quaternion_pt[2]
        marker.pose.orientation.w = quaternion_pt[3]
        marker.color.r, marker.color.g, marker.color.b = (0, 255, 0)
        marker.scale.x, marker.scale.y, marker.scale.z = (0.3, 0.3, 0.3)
        marker.type= Marker.CUBE
        marker.color.a = 0.5
        self.average_marker.publish(marker)





        # Publish a transformation frame between the map
        # and the particle_filter_frame.
    def rotate_r2W(self, theta_r2W):
        return np.array([
            [np.cos(theta_r2W), -np.sin(theta_r2W), 0.0],
            [np.sin(theta_r2W), np.cos(theta_r2W),  0.0],
            [0.0,               0.0,                1.0]
            ])



if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
