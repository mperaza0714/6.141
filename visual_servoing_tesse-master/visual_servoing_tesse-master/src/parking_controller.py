#!/usr/bin/env python
import rospy
from visual_servoing_tesse.msg import cone_location, parking_error
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

import ros_utils

class ParkingController():
    DRIVE_TOPIC = "/tesse/drive"
    CONE_LOCATION = "/relative_cone"
    PARKING_ERROR_TOPIC = "/parking_error"

    L = rospy.get_param("pursuit/l")
    V = rospy.get_param("pursuit/v")
    D_LIM = rospy.get_param("pursuit/park/d_lim")
    DETLA_LIM = rospy.get_param("pursuit/park/delta_lim")

    X_BUMPER = 1.5

    def __init__(self):
        self.cone_sub = rospy.Subscriber(self.CONE_LOCATION, cone_location, self.relative_cone_callback)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 5)
        self.error_pub = rospy.Publisher(self.PARKING_ERROR_TOPIC, parking_error, queue_size = 5)

        self.controller = PurePursuit(self.L, self.V, self.v_park)

    def v_park(d, delta):
        """
        A velocity curve which increases linearly with parking distance, and
        decreases on a bell curve (sech) with steering angle.
        """
        d_0 = 0.6096 # Desired Parking distance (less than 2 ft from front bumper)
        v_dist = min(1/self.D_LIM * (d-self.X_BUMPER), 1.0) if d > d_0 else 0.0
        v_steer = np.sech( 2*delta / self.DELTA_LIM )
        return v_dist * v_steer

    def init_header(self, header, frame = "base_link_gt"):
        header.stamp = rospy.Time.now()
        header.frame_id = frame

    def relative_cone_callback(self, cone_msg):
        # Update pure pursuit controller
        x, y = cone_msg.x_pos, cone_msg.y_pos
        self.controller.update_control(x, y)

        # Publish Ackermann drive instructions
        drive = AckermannDriveStamped()
        self.init_header(drive.header)

        drive.drive.steering_angle, drive.drive.velocity =\
            self.controller.get_control(log = False)

        self.drive_pub.publish(drive)

        # Publish parking error
        error = parking_error()
        error.x = self.controller.x
        error.y = self.controller.y
        error.distance_error = self.controller.d
        self.error_pub.publish(error)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
