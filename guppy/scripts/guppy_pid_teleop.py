#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_srvs.srv import SetBool

from joy_models import *

class Teleop():
    def __init__(self):
        self.joy = DUALSHOCK3()
        self.max_effort_ = 0.4
        self.depth_ = 0.0
        self.depth_step_ = 0.2
        self.depth_setpoint_ = 0.0
        self.start_pressed_ = rospy.Time.now()


        self.autopilot_en_ = False
        autopilot_service_name = 'pid/enable'
        rospy.loginfo('Waiting for service: %s' % autopilot_service_name)
        rospy.wait_for_service(autopilot_service_name)
        self.autopilot_service_ = rospy.ServiceProxy(autopilot_service_name, SetBool)

        self.pitch_pid_en_ = False
        pitch_pid_service_name = 'pid/pitch_pid/enable'
        rospy.loginfo('Waiting for service: %s' % pitch_pid_service_name)
        rospy.wait_for_service(pitch_pid_service_name)
        self.pitch_pid_service_ = rospy.ServiceProxy(pitch_pid_service_name, SetBool)

        self.depth_pid_en_ = False
        depth_pid_service_name = 'pid/depth_pid/enable'
        rospy.loginfo('Waiting for service: %s' % depth_pid_service_name)
        rospy.wait_for_service(depth_pid_service_name)
        self.depth_pid_service_ = rospy.ServiceProxy(depth_pid_service_name, SetBool)

        self.cmd_publisher_ = rospy.Publisher('twist_command', Twist, queue_size=1)
        self.depth_sp_publisher_ = rospy.Publisher('/depth_pid/setpoint', Float64, queue_size=1)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.Subscriber('depth', Float64, self.depth_callback)

        rospy.loginfo('Teleoperation started')

    def depth_callback(self, msg):
        self.depth_ = msg.data

    def joy_callback(self, msg):
        if (msg.buttons[self.joy.BUTTONS.START] and
            (rospy.Time.now() - self.start_pressed_).secs > 0.2):
            self.start_pressed_ = rospy.Time.now()

            try:
                self.autopilot_en_ = not self.autopilot_en_
                resp = self.autopilot_service_(self.autopilot_en_)
                rospy.logwarn(resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("Service did not process request: " + str(e))

            try:
                self.pitch_pid_en_ = not self.pitch_pid_en_
                resp = self.pitch_pid_service_(self.pitch_pid_en_)
                rospy.logwarn(resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("Service did not process request: " + str(e))

            try:
                self.depth_pid_en_ = not self.depth_pid_en_
                resp = self.depth_pid_service_(self.depth_pid_en_)
                rospy.logwarn(resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("Service did not process request: " + str(e))
        else:
            self.publish_twist(msg)

    def publish_twist(self, joy_msg):
        twist_msg = Twist()

        twist_msg.linear.x = joy_msg.axes[self.joy.AXIS.LEFT_STICK_UD] * self.max_effort_
        twist_msg.linear.y = joy_msg.axes[self.joy.AXIS.LEFT_STICK_LR] * self.max_effort_
        twist_msg.linear.z = joy_msg.axes[self.joy.AXIS.RIGHT_STICK_UD] * self.max_effort_
        twist_msg.angular.z = joy_msg.axes[self.joy.AXIS.RIGHT_STICK_LR] * self.max_effort_ * 0.5

        if joy_msg.buttons[self.joy.BUTTONS.UP]:
            self.depth_setpoint_ = max(self.depth_setpoint_ - self.depth_step_, 0)
            self.depth_sp_publisher_.publish(self.depth_setpoint_)
        elif joy_msg.buttons[self.joy.BUTTONS.DOWN]:
            self.depth_setpoint_ = self.depth_setpoint_ + self.depth_step_
            self.depth_sp_publisher_.publish(self.depth_setpoint_)

        if joy_msg.buttons[self.joy.BUTTONS.CIRCLE]:
            self.depth_sp_publisher_.publish(self.depth_)
        # twist_msg.angular.z = self.process_trigger(joy_msg)
        # if joy_msg.buttons[self.joy.BUTTONS.LEFT_SHIFT]:
        #     twist_msg.angular.z = self.max_effort_
        # elif joy_msg.buttons[self.joy.BUTTONS.RIGHT_SHIFT]:
        #     twist_msg.angular.z = -self.max_effort_

        self.cmd_publisher_.publish(twist_msg)

    def process_trigger(self, joy_msg):
        if joy_msg.buttons[self.joy.BUTTONS.RIGHT_TRIGGER]:
            right = 1 - joy_msg.axes[self.joy.AXIS.RIGHT_TRIGGER]
        else: right = 0
        if joy_msg.buttons[self.joy.BUTTONS.LEFT_TRIGGER]:
            left = 1 - joy_msg.axes[self.joy.AXIS.LEFT_TRIGGER]
        else: left = 0
        return (left - right) / 2.0


if __name__ == '__main__':
    rospy.init_node('joy_teleop')
    teleop = Teleop()
    rospy.spin()
