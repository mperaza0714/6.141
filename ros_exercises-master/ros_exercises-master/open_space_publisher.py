import rospy
import numpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def open_space(scan):
    pub_distance = rospy.Publisher('/open_space/distance', Float32, queue_size=10)
    pub_angle = rospy.Publisher('/open_space/angle', Float32, queue_size=10)
    ranges = scan.ranges
    max_range = max(ranges)
    angle = scan.angle_min + ranges.index(max_range)*scan.angle_increment
    
    pub_angle.publish(angle)
    pub_distance.publish(max_range)
    rospy.loginfo(max_range)
    rospy.loginfo(angle)


def open_space_publisher():
    rospy.init_node('open_space_publisher', anonymous=True)
    sub = rospy.Subscriber('/fake_scan', LaserScan, open_space)
    rospy.spin()

if __name__ == '__main__':
    open_space_publisher()
