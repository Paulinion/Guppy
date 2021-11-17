#!/usr/bin/env python

from math import pi, copysign, atan2, sin, cos, radians, degrees

import rospy

import smach
from smach import State

import smach_ros
from guppy.msg import config

########### CONSTS ###############

WORK_DEPTH = 0.5
SURGE_EFFORT = 0.0

EPS_DEPTH = 0.1
EPS_HEADING = radians(5)
STARTING_HEADING = radians(-10)

##################################


"""class Submerge(smach.State):
    def __init__(self, c, depth):
        smach.State.__init__(self, outcomes=['depth_approached'])
        self.cntr = c
        self.depth = depth

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.switch_depth_pid(True)
        self.cntr.switch_pitch_pid(True)
        self.cntr.submerge(self.depth)
        while not rospy.is_shutdown():
            if self.cntr.depth_approached(self.depth):
                return 'depth_approached'
            else:
                self.cntr.rate.sleep()

class Emerge(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['aborted'])
        self.cntr = c

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.emerge()
        while not rospy.is_shutdown():
            if self.cntr.depth_approached(0) or ((rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(10)):
                self.cntr.switch_autopilot(False)
                self.cntr.switch_depth_pid(False)
                self.cntr.switch_pitch_pid(False)
                return 'aborted'
            else:
                self.cntr.rate.sleep()

class ApproachHeadingImu(smach.State):
    def __init__(self, c, desired_heading):
        smach.State.__init__(self, outcomes=['heading_approached'])
        self.cntr = c
        self.desired_heading = desired_heading

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.set_heading(self.desired_heading)
        while not rospy.is_shutdown():
            if self.cntr.heading_approached(self.desired_heading):
                return 'heading_approached'
            else:
                self.cntr.rate.sleep()


class MoveStraight(smach.State):
    def __init__(self, c, time):
        smach.State.__init__(self, outcomes=['timeout'])
        self.cntr = c
        self.time = time

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.move(SURGE_EFFORT, 0.0)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(self.time):
                self.cntr.move(0.0, 0.0)
                return 'timeout'
            else:
                self.cntr.rate.sleep()"""


class Teleop(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['Switch_to_holding_depth'])
        print(" StartTeleop")

        self.cntr = c

    def execute(self, userdata):

        # self.cntr.teleop() можно сделать перехват телеуправления
        while not rospy.is_shutdown():
            if self.cntr.config_msg.depth_holding == True:
                # self.cntr.switch_autopilot(False)
                # self.cntr.switch_depth_pid(False)
                # self.cntr.switch_pitch_pid(False)

                return 'Switch_to_holding_depth'
            else:

                self.cntr.rate.sleep()


class DepthHolding(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['Switch_to_full_teleop'])
        print("StartDepth")

        self.cntr = c

    def execute(self, userdata):
        # self.cntr.switch_depth_pid(True)
        while not rospy.is_shutdown():
            if self.cntr.config_msg.depth_holding == False:

                return 'Switch_to_full_teleop'

            else:

                self.cntr.rate.sleep()


class Controller():
    def __init__(self):
        self.config_msg = config()
        if self.config_msg.depth_holding != self.config_msg.depth_holding:
            self.config_msg.x_goal = 0
            self.config_msg.y_goal = 0
            self.config_msg.depth_holding = False

        self.rate = rospy.Rate(10)
        self.heading_eps_ = EPS_HEADING
        self.depth_ = 0.0
        self.depth_eps_ = EPS_DEPTH
        self.heading_ = 0.0
        self.working_depth_ = WORK_DEPTH

        # rospy.Subscriber('depth', Float64, self.depth_callback)
        # rospy.Subscriber('heading', Float64, self.heading_callback)
        rospy.Subscriber('config', config, self.config)
        rospy.loginfo('waiting sevices')
        # rospy.wait_for_service('/autopilot/enable')
        # rospy.wait_for_service('/autopilot/depth_pid/enable')
        # rospy.wait_for_service('/autopilot/pitch_pid/enable')
        # self.autopilot_enable_service = rospy.ServiceProxy('/autopilot/enable', SetBool)
        # self.depth_pid_enable_service = rospy.ServiceProxy('/autopilot/depth_pid/enable', SetBool)
        # self.pitch_pid_enable_service = rospy.ServiceProxy('/autopilot/pitch_pid/enable', SetBool)

        # self.command_publisher_ = rospy.Publisher('teleop_command', Twist, queue_size=1)
        # self.pose_publisher_ = rospy.Publisher('pose', Pose, queue_size=1)
        # self.depth_sp_publisher = rospy.Publisher('depth_pid/setpoint', Float64, queue_size=1)
        # self.heading_sp_publisher = rospy.Publisher('heading_setpoint', Float64, queue_size=1)

        self.state_change_time = rospy.Time.now()

    def config(self, config):
        self.config_msg = config

    def set_heading(self, heading):
        self.heading_sp_publisher.publish(heading)

    def submerge(self, depth):
        self.depth_sp_publisher.publish(self.working_depth_)

    def switch_autopilot(self, en):
        resp = self.autopilot_enable_service(en)

    def switch_depth_pid(self, en):
        resp = self.depth_pid_enable_service(en)

    def switch_pitch_pid(self, en):
        resp = self.pitch_pid_enable_service(en)

    def emerge(self):
        self.depth_sp_publisher.publish(0)

    def heading_approached(self, d):
        rospy.loginfo(abs(self.heading_ - d))
        e = radians((degrees(self.depth_ - d) + 180) % 360 - 180)
        if abs(self.heading_ - d) < self.heading_eps_:
            return True
        else:
            return False

    def depth_approached(self, d):
        if abs(d - self.depth_) < self.depth_eps_:
            return True
        else:
            return False

    def heading_callback(self, msg):
        self.heading_ = msg.data

    def depth_callback(self, msg):
        self.depth_ = msg.data

    def move(self, x, y):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        self.command_publisher_.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('guppy_state_maschine')
    print("main")
    c = Controller()
    sm = smach.StateMachine(outcomes=['aborted'])
    with sm:
        # smach.StateMachine.add('SUBMERGE', Submerge(c, WORK_DEPTH),
        # transitions={'depth_approached': 'WAIT'})

        # smach.StateMachine.add('HEADING1', ApproachHeadingImu(c, STARTING_HEADING),
        #                        transitions={'heading_approached': 'MOVE1'})

        # smach.StateMachine.add('WAIT', MoveStraight(c, 10),
        # transitions={'timeout': 'EMERGE'})

        # smach.StateMachine.add('EMERGE', Emerge(c),
        # transitions={'aborted': 'aborted'})
        smach.StateMachine.add('TELEOP', Teleop(c),
                               transitions={'Switch_to_holding_depth': 'DEPTH_HOLDING'})
        smach.StateMachine.add('DEPTH_HOLDING', DepthHolding(c),
                               transitions={'Switch_to_full_teleop': 'TELEOP'})

    rospy.sleep(rospy.Duration(1))
    outcome = sm.execute()
