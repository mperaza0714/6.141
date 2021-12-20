#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils

from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        self.trajectory         = utils.LineTrajectory("/followed_trajectory")
        # TODO: Ground truth odometry is for testing Part C ONLY.
        # To get credit for Part C and Part D, must use localized odometry (Lab 5)
        self.odom_topic         = rospy.get_param("~odom_topic")
        self.odom_gt_topic      = "/odom"
        self.odom_sub           = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=1)
        self.traj_sub           = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub          = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)

        self.look_rad_pub       = rospy.Publisher("/pf/lookahead/rad", Marker, queue_size=1)
        self.look_pos_pub       = rospy.Publisher("/pf/lookahead/pos", Marker, queue_size=1)
        self.closest_pub        = rospy.Publisher("/pf/lookahead/closest", Marker, queue_size=1)

        self.l_max              = 10.0 # Maximum Lookahead (m)
        self.v_max              = 13.5 # Maximum Speed (m/s)

        self.lookahead          = self.l_max # Adaptive Lookahead
        self.speed              = self.v_max # Adaptive Speed
        self.steering_angle     = 0.0 # Turning Angle (rad)
        #self.wheelbase_length   = 0.325 # Wheelbase Length (m) in 2D sim
        self.wheelbase_length   = 2.5 # Wheelbase Length (m) in 3D sim

        # See self.trajectory_callback for initialization.
        self.seg_points         = None
        self.seg_start          = None
        self.seg_end            = None
        self.seg_distances      = None
        # See self.odometry_callback for initialization.
        self.p_robot            = None
        self.heading            = None
        self.prev_explored      = list() #segment indices

        # these topics are for coordinate space things
        self.map_from_world = np.array([166.45+25.3482,-232.95-5.6846])

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message.
        '''
        print("New Trajectory:    ", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)

        self.seg_points     = np.array(self.trajectory.points)
        self.seg_start      = self.seg_points[:-1, :]
        self.seg_end        = self.seg_points[1:, :]
        self.seg_distances  = np.array(self.trajectory.distances)
        self.prev_explored  = list() #segment indices

        self.trajectory.publish_viz(duration=0.0)

    def odometry_callback(self, msg):
        ''' Runs pure pursuit upon an odometry update, then publishes new drive instructions.
        '''
        if self.seg_points is None:
            # rospy.logerr("currently empty trajectory")
            return

        print "\nPURE PURSUIT"

        # 0. Unpack robot pose (position and heading)
        (self.p_robot, self.heading) = utils.pose_from_msg(msg.pose.pose)
        self.p_robot = -self.p_robot + self.map_from_world
        self.heading = self.heading + np.pi

        # 1. Get closest point on trajectory from self.get_closest_point
        (p_close, i_close) = self.get_closest_point()
        print "Closest Point:\t", p_close, "on segment", i_close

        # 2. Get transient goal from self.get_lookahead_point
        # if i_close == self.seg_end.shape[0]:
        #     p_goal = self.seg_end[-1, :]
        #     self.goal_flag = True
        #     print "PURSUING GOAL POSE"
        # else:
        self.adapt_lookahead(False)
        p_goal = self.get_lookahead_point(i_close)
        print "Transient Goal:\t", p_goal

        # 3. Run pure pursuit algorithm on transient goal.
        self.steering_angle = self.evaluate(p_goal)
        self.adapt_speed(False, True)
        print "Speed:\t%3.3f m/s" % (self.speed)
        print "Steering Angle:\t%3.3f deg" % (self.steering_angle*180.0/np.pi)

        # 4. Publish Messages.
        self.publish_drive()
        # self.viz_closest(p_close)
        # self.viz_lookahead(self.lookahead, p_goal)

    def get_closest_point(self):
        '''
        Choose the segment of the desired trajectory closest to the robot.
        1. Find the shortest distance on each segment to the robot.
        2. Choose the segment with the shortest distance.
        Returns:
            p_closest: the point on the trajectory with the shortest distance to the robot.
            i_closest: the index of the trajectory segment where p_close is found.
        '''

        # Refer to example: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment/1501725#1501725
        p, v, w = self.p_robot, self.seg_start, self.seg_end
        l = np.linalg.norm(w - v, axis=1)

        # We clamp t from [0,1] to handle points outside the segment vw.
        t = utils.dot(p - v, w - v) / l**2.0
        t = np.maximum(0.0, np.minimum(1.0, t))
        t = np.where(t >= 0.0, t, 0.0)
        t = np.where(t <= 1.0, t, 1.0)
        t_mesh = np.tile(t, reps=(2,1)).T

        projections = v + t_mesh*(w - v) # Projections falls on each segment
        robot_distances = np.linalg.norm(projections - p, axis=1)
        proj_index = np.argmin(robot_distances)

        if len(self.prev_explored) == 0:
            self.prev_explored.append(proj_index)
            return projections[proj_index], proj_index

        # rejects points that too far ahead of current robot index
        i = self.prev_explored[-1]
        if proj_index - i > 1 or proj_index - i < 0:
            proj_index = i if robot_distances[i] < robot_distances[i+1] else i+1

        if proj_index == self.prev_explored[-1]+1:
            self.prev_explored.append(proj_index)

        return projections[proj_index], proj_index

    def get_lookahead_point(self, start_i):
        '''
        Parameters
            start_i: index of the the closest point in self.trajectory.points
        Returns:
            p_goal: the chosen lookahead point along the trajectory
        '''

        # Refer to example: https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm/86428#86428
        Q, r = self.p_robot, self.lookahead
        P1, V = self.seg_start[start_i:,:], self.seg_end[start_i:,:] - self.seg_start[start_i:,:]
        Q_mesh = np.tile(Q, reps=(P1.shape[0], 1))

        # at^2 + bt + c = 0
        a = utils.dot(V, V)
        b = 2 * utils.dot(V, P1 - Q_mesh)
        c = utils.dot(P1, P1) + utils.dot(Q_mesh, Q_mesh) - 2 * utils.dot(P1, Q_mesh) - r**2

        disc = b**2 - 4*a*c
        # is_real = (disc >= 0)

        t1 = (-b + np.sqrt(disc)) / (2*a)
        # t2 = (-b - np.sqrt(disc)) / (2*a)
        t1_mesh = np.tile(t1, reps=(2,1)).T
        # t2_mesh = np.tile(t2, reps=(2,1)).T
        # does_intersect = (t >= 0.0) & (t <= 1.0)

        pos_projections = (P1 + t1_mesh*V)[(disc >= 0) & (t1 >= 0.0) & (t1 <= 1.0), :]
        # neg_projections = (P1 + t2_mesh*V)[(disc >= 0) & (t2 >= 0.0) & (t2 <= 1.0), :]

        if pos_projections.shape[0] == 0:# and neg_projections.shape[0] == 0:
            print "ERROR: 'self.get_lookahead_point' returned no solutions"
            return self.seg_points[start_i, :]

        return pos_projections[0, :]

    def evaluate(self, p_goal):
        ''' Find a steering angle that will approach the transient goal.
        '''
        dp = p_goal - self.p_robot
        d = np.linalg.norm(dp)
        theta = np.arctan2(dp[1], dp[0]) - self.heading
        # Steering angle: delta = arctan( 2 L sin(theta) / d )
        delta = np.arctan2(2*self.wheelbase_length*np.sin(theta), d)
        print "PurePursuit.evaluate\td: %3.3f,\ttheta: %.3f" % (d, theta)
        return delta * 0.05

    # ==========================================================================
    # HELPER METHODS
    # ==========================================================================

    def adapt_lookahead(self, v_dep = True):
        '''
        Sets an adaptive lookahead with respect to speed and a maximum.
        Parameters:
            v_dep: True if lookahead should depend on adaptive speed, else False.
        '''
        l_speed = self.speed / self.v_max
        l_speed = utils.wrap(l_speed, lo=0.2, hi=1.0)
        self.lookahead = self.l_max * (l_speed if v_dep else 1.0)

    def adapt_speed(self, d_dep = True, t_dep = True):
        '''
        Sets an adaptive speed with respect to goal distance, turning angle, and a maximum.
        Parameters:
            d_dep: True if speed should depend on goal distance, else False.
            t_dep: True if speed should depend on turning angle, else False.
        '''
        v_dist = (np.linalg.norm(self.seg_points[-1,:] - self.p_robot) - self.wheelbase_length) / self.l_max
        v_dist = utils.wrap(v_dist)
        v_turn = 1.0/np.cosh((np.pi/2.0 * np.absolute(self.steering_angle))**0.2)
        v_turn = utils.wrap(v_turn)
        self.speed = self.v_max * (v_dist if d_dep else 1.0) * (v_turn if t_dep else 1.0)

    def publish_drive(self):
        msg = AckermannDriveStamped()
        msg.header = self.trajectory.make_header("/map")
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle

        self.drive_pub.publish(msg)

    def viz_closest(self, p):
        closest = Marker()
        closest.header = self.trajectory.make_header("/map")
        closest.ns = self.trajectory.viz_namespace + "/trajectory"
        closest.id = 5
        closest.type = 1 # cube
        closest.lifetime = rospy.Duration.from_sec(0.0)
        closest.action = 0
        closest.pose = utils.msg_from_pose(p)
        closest.scale.x, closest.scale.y, closest.scale.z = 0.5, 0.5, 0.5
        closest.color.r, closest.color.g, closest.color.b = 1.0, 1.0, 0.0
        closest.color.a = 1.0
        self.closest_pub.publish(closest)

    def viz_lookahead(self, r, p):
        look_rad = Marker()
        look_rad.header = self.trajectory.make_header("/map")
        look_rad.ns = self.trajectory.viz_namespace + "/trajectory"
        look_rad.id = 3
        look_rad.type = 3 # cylinder
        look_rad.lifetime = rospy.Duration.from_sec(0.0)
        look_rad.action = 0
        look_rad.pose = utils.msg_from_pose(self.p_robot)
        look_rad.scale.x, look_rad.scale.y, look_rad.scale.z = 2*r, 2*r, 0.1
        look_rad.color.r, look_rad.color.g, look_rad.color.b = 1.0, 1.0, 0.0
        look_rad.color.a = 0.5
        self.look_rad_pub.publish(look_rad)

        look_pos = Marker()
        look_pos.header = self.trajectory.make_header("/map")
        look_pos.ns = self.trajectory.viz_namespace + "/trajectory"
        look_pos.id = 4
        look_pos.type = 3 # cylinder
        look_rad.lifetime = rospy.Duration.from_sec(0.0)
        look_pos.action = 0
        look_pos.pose = utils.msg_from_pose(p)
        look_pos.scale.x, look_pos.scale.y, look_pos.scale.z = 0.5, 0.5, 2.0
        look_pos.color.r, look_pos.color.g, look_pos.color.b = 1.0, 1.0, 0.0
        look_pos.color.a = 1.0
        self.look_pos_pub.publish(look_pos)




if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
