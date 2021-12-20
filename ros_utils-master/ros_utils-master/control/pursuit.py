#!/usr/bin/env python2

import rospy
import numpy as np

class PurePursuit:
    def __init__(self, l, v_max, v_norm = None):
        """
        Initialize a Pure Pursuit controller for a robot with an Ackermann (i.e.
            bycicle) drivetrain.
        Parameters:
            l - L, the characteristic length [m] of the drivetrain
                (for Ackermann drive, this is the distance between
                the front and rear axels).
            v_max - V, the maximum desired velocity [m/s].
            v_norm - a function with arguments (d, delta) which is the
                normalized velocity curve on the range (0.0, 1.0).
                - d is the distance to goal, and delta is the steering angle.
                - By default argument returns a unity lambda ('return 1.0').
        """

        self.l = l
        self.v_max = v_max
        self.v_norm = v_norm if v_norm is not None else PurePursuit.v_constant

        # Input Signals:
        self.x, self.y = (0.0, 0.0)     # Target position (x, y)
        self.d, self.theta = (0.0, 0.0) # Polar target (d, theta)

        # Control Signals: Ackermann drive (delta, v)
        self.u = (0.0, 0.0)

    def update_control(self, x, y, log = False):
        """
        Sets the control signals for steering angle (delta) and velocity (v).
        Parameters:
            x - position of target on x-axis
            y - position of target on y-axis
            log - flag for logging polar (d,theta) using 'rospy.loginfo'
                (default False)
        """

        self.d = (x**2.0 + y**2.0) ** 0.5
        self.theta = np.arctan2(y, x)

        if log:
            print "PurePursuit.input\td: %.3f,\ttheta: %.3f", d, theta
            rospy.loginfo("PurePursuit.input\td: %.3f,\ttheta: %.3f", d, theta)

        # Steering angle: delta = arctan( 2 L sin(theta) / d )
        delta = np.arctan2(2*self.l*np.sin(self.theta), self.d)
        # Velocity: V = V_max * f(d, delta)
        v = self.v_max * self.v_norm(self.d, delta)

        self.u = (delta, v)

    def get_control(self, log = False):
        """
        Parameters:
            log - flag for logging control (delta,v) using 'rospy.loginfo'
                (default False)
        Returns:
            delta - the Ackermann steering angle [rad]
            v - the Ackermann velocity [m/s]
        """
        if log:
            delta, v = self.u
            rospy.loginfo("PurePursuit.output\tdelta: %.3f,\tv: %.3f", delta, v)
        return self.u

    @staticmethod
    def v_constant(d, delta):
        """
        A constant (unity) velocity curve.
        """
        return 1.0

    @staticmethod
    def v_bangbang(d, delta):
        """
        A bang-bang velocity curve: returns 1 if robot is outside of parking
        distance 1, else 0.
        Note: To use as argument 'v_curve'
        """
        return 1.0 if d > 1 else 0.0
