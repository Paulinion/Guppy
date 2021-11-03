#!/usr/bin/python

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Int32MultiArray


def effort_to_pwm(e):
    return 32500 + 20000 / 2 * max(-1, min(1, e))

def thruster_callback(msg, thruster):
    global last_msg_time
    last_msg_time = rospy.Time.now()
    pwm_msg.data[thruster] = effort_to_pwm(msg.data)

def check_timer(_):
    if (rospy.Time.now() - last_msg_time).secs > 5:
        send_zeros()

def send_zeros():
    pwm_msg.data = [effort_to_pwm(0)] * 16
    pwm_publisher.publish(pwm_msg)

if __name__ == '__main__':
    pwm_msg = Int32MultiArray()
    pwm_msg.data = [effort_to_pwm(0)] * 16

    rospy.init_node('thrusters', anonymous='false')

    rate = rospy.Rate(10)
    last_msg_time = rospy.Time.now()


    for i in range(16):
        rospy.Subscriber('thrusters/%s' % i, Float64, thruster_callback, i)
    pwm_publisher = rospy.Publisher('pwm', Int32MultiArray, queue_size=1)

    timer = rospy.Timer(rospy.Duration(0.5), check_timer)
    rospy.on_shutdown(send_zeros)

    while not rospy.is_shutdown():
        pwm_publisher.publish(pwm_msg)
        rate.sleep()

