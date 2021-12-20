#!/usr/bin/env python

import rospy
import random
import numpy
from sensor_msgs.msg import LaserScan

pi = numpy.pi
rospy.init_node('fake_scan_publisher')

publish_topic = rospy.get_param('publish_topic', '/fake_scan')
publish_rate = rospy.get_param('publish_rate', 20)
angle_min = rospy.get_param('angle_min', (-2.0/3.0)*pi)
angle_max = rospy.get_param('angle_max', (2.0/3.0)*pi)
range_min = rospy.get_param('range_min', 1.0)
range_max = rospy.get_param('range_max', 10.0)
angle_increment = rospy.get_param('angle_increment', (1.0/300.0)*pi)

scan_pub = rospy.Publisher(publish_topic, LaserScan, queue_size=50)

rate = rospy.Rate(publish_rate)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()
    scan.header.stamp = current_time
    scan.header.frame_id = 'base_link'
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.range_min = range_min
    scan.range_max = range_max

    scan.ranges = []
    for i in range(401):
        scan.ranges.append(random.uniform(scan.range_min, scan.range_max))

    scan_pub.publish(scan)
    rate.sleep()
