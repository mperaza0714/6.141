import rospy
from std_msgs.msg import Float32
from numpy import log

def random_float_log(data):
    pub = rospy.Publisher('/random_float_log', Float32, queue_size=10)
    ln = log(data.data)
    rospy.loginfo(ln)
    pub.publish(ln)



def simple_subscriber():
    rospy.init_node('simple_subscriber', anonymous=True)
    sub = rospy.Subscriber('/my_random_float', Float32, random_float_log)
    rospy.spin()

if __name__ == '__main__':
    simple_subscriber()
