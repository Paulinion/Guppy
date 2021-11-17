#!/usr/bin/env python


from __future__ import print_function

import threading
from guppy.msg import config
import roslib;


import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

"""msg = 
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

"""Control= {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),"
}




    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")"""


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

print("START")
if __name__ == "__main__":
    rospy.init_node('controller_keyboard')
    print("START")
    config_publisher = rospy.Publisher('config', config, queue_size=1)
    settings = termios.tcgetattr(sys.stdin)
    rate = rospy.Rate(10)


    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    config_msg = config()
    x_goal = 0
    y_goal = 0
    depth_holding = False

    while not rospy.is_shutdown():
        key = getKey(key_timeout)
        if key == 'c':
            if depth_holding == False:
                depth_holding = True
            else:
                depth_holding = False
            print("depth_holding: ", depth_holding)
        elif (key == '\x03'):
            break

        else:
            print("skip")

        config_msg.x_goal = x_goal
        config_msg.y_goal = y_goal
        config_msg.depth_holding = depth_holding
        config_publisher.publish(config_msg)
        rate.sleep()

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
