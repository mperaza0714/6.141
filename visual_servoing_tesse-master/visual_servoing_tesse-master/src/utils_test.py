#!usr/bin/env python2

import ros_utils

print "import ros_utils: Success!"

if __name__ == "__main__":
    absolute_pid = ros_utils.control.pid.PID(1.0, 0.0, 0.5)
    print "Absolute Path: ros_utils.control.pid.PID"
    relative_pid = ros_utils.PID(1.0, 0.0, 0.5)
    print "Relative Path: ros_utils.PID"

    absolute_pursuit = ros_utils.control.pursuit.PurePursuit(3.0, 4.0)
    print "Absolute Path: ros_utils.control.pursuit.PurePursuit"
    relative_pursuit = ros_utils.PurePursuit(3.0, 4.0)
    print "Relative Path: ros_utils.PurePursuit"

    print "All systems are go!"
