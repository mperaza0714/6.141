#!/usr/bin/env python

import rospy
import numpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ros_exercises.msg import OpenSpace

subscriber_topic = rospy.get_param('subscriber_topic', '/fake_scan')
publisher_topic = rospy.get_param('publisher_topic', '/fake_scan')

def open_space(scan):
    pub_distance = rospy.Publisher(publisher_topic + '/distance', Float32, queue_size=10)
    pub_angle = rospy.Publisher(publisher_topic + '/angle', Float32, queue_size=10)
    pub = rospy.Publisher(publisher_topic, OpenSpace, queue_size=10)
    ranges = scan.ranges
    max_range = max(ranges)
    angle = scan.angle_min + ranges.index(max_range)*scan.angle_increment

    msg = OpenSpace()
    msg.angle = angle
    msg.distance = max_range
    
    pub_angle.publish(angle)
    pub_distance.publish(max_range)
    pub.publish(msg)
    rospy.loginfo(max_range)
    rospy.loginfo(angle)
    rospy.loginfo(msg)


def open_space_publisher():
    rospy.init_node('open_space_publisher', anonymous=True)
    sub = rospy.Subscriber(subscriber_topic, LaserScan, open_space)
    rospy.spin()

if __name__ == '__main__':
    open_space_publisher()
