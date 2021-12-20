#!/usr/bin/env python2

import numpy as np
import rospy

class MotionModel:

    # NOISE: Confirmed-working SD values for odometry noise (epsilon)
    # epsilon   MISSES      SUCCESS
    # 0.1       46/150      N
    # 0.01      1/150       N
    # 0.005     passed      Y
    # 0.0075    1/150       N
    # 0.007     2/150       N

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        ####################################

        self.deterministic = rospy.get_param("~deterministic")

    def get_noise(self, N, epsilon = 0.01):
        return np.random.normal(0.0, epsilon, size=(N, 3))
        # return np.asarray(np.random.normal(0.0, epsilon, size=(2, N)), np.random.normal(0.0, 0.05, size = N)).T

        ####################################

    def rotate_r2W(self, theta_r2W):
        return np.array([
            [np.cos(theta_r2W), -np.sin(theta_r2W), 0.0],
            [np.sin(theta_r2W), np.cos(theta_r2W),  0.0],
            [0.0,               0.0,                1.0]
        ])

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        N, dof = particles.shape

        new_particles = np.zeros((N, dof))
        noise = self.get_noise(N)

        for i in range(N):
            p = particles[i]
            x, y, theta = p

            R_r2W = self.rotate_r2W(theta)

            odom_r = odometry + (noise[i, :] if (not self.deterministic) else 0.0)
            odom_W = np.matmul(R_r2W, odom_r)

            new_particles[i] = p + odom_W

        return new_particles

        ####################################
