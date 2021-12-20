

#!/usr/bin/env python2

import numpy as np

import rospy
# from rospy.numpy_msg import numpy_msg

class PID:
    """
    A signal controller that provides proportional (P), integral (I), and/or
    differential (D) error correction to any measurement, given a reference
    target.
    """

    class Error:
        """
        Keeps track of timestamped errors, in terms of a presenterror, previous
        error, and total accumulation of error.
        """
        def __init__(self):
            self.pres = (0.0, 0.0) # Most recent logged error
            self.prev = (self.pres, 0.0) # Previously logged error

            self.total = [0.0] * 100 # Accumulated error Sum(e[i] * (t[i] - t[i-1]))

        def e_inc(self):
            return self.pres[0] - self.prev[0]

        def t_inc(self):
            return self.pres[1] - self.prev[1]

        def log(self, e):
            self.prev = self.pres
            self.pres = (e, rospy.Time.now().to_nsec())
            self.total = e * self.t_inc()

    def __init__(self, p, i, d):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.reset_error()

    def reset_error(self):
        self.error = PID.Error()

    def eval(self, y, reference = 0.0):
        self.error.log(e = reference - y)

        dt = self.error.t_inc()
        de = self.error.e_inc()
        e = self.error.pres[0]
        total  = self.error.total

        try:
            return self.Kp * e + self.Ki * total + self.Kd * de/dt
        except:
            return self.Kp * e + self.Ki * total
