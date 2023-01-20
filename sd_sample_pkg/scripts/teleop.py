#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import sys, select, termios, tty

MIN_VEL_X = -1
MAX_VEL_X = 1
MIN_ANG_Z = -1
MAX_ANG_Z = 1



usage = """
usage:
publish linear/angular velocity
1 2
    w
a   s   d
    x
w...go straight
x...go back
a...rotate 
d...rotate
s...stop
1...Left_Mode
2...Right_Mode
q...quit
"""


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    print(usage)
    rospy.init_node('teleop_key')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub_cmd = rospy.Publisher('/cmd_LR', String, queue_size=1)

    while not rospy.is_shutdown():
        key = getKey(settings)
        # print("key: {}".format(key))

        if key == "q":
            break
        
        vel = 0.
        omega = 0.
        if key == "w":
            vel = MAX_VEL_X
        elif key == "x":
            vel = MIN_VEL_X
        elif key == "a":
            omega = MAX_ANG_Z
        elif key == "d":
            omega = MIN_ANG_Z
        elif key == "s":
            vel = 0.
            omega = 0.
        elif key == "1":
            cmd = String()
            cmd.data = "Left"
            pub_cmd.publish(cmd)
        elif key == "2":
            cmd = String()
            cmd.data = "Right"
            pub_cmd.publish(cmd)


        
        twist = Twist()
        twist.linear.x = vel; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = omega
        pub.publish(twist)


        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()