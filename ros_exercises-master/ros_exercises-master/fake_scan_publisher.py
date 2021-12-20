import rospy
import random
import numpy
from sensor_msgs.msg import LaserScan

rospy.init_node('fake_scan_publisher')

scan_pub = rospy.Publisher('/fake_scan', LaserScan, queue_size=50)

pi = numpy.pi
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()
    scan.header.stamp = current_time
    scan.header.frame_id = 'base_link'
    scan.angle_min = (-2.0/3.0)*pi
    scan.angle_max = (2.0/3.0)*pi
    scan.angle_increment = (1.0/300.0)*pi
    scan.range_min = 1.0
    scan.range_max = 10.0

    scan.ranges = []
    for i in range(400):
        scan.ranges.append(random.uniform(scan.range_min, scan.range_max))

    scan_pub.publish(scan)
    rate.sleep()
