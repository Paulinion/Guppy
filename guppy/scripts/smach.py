#!/usr/bin/env python

import rospy
import smach
from guppy_pid_teleop import Teleop
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_srvs.srv import SetBool

from joy_models import *




class TELEOP(smach.State):
    def __init__(self):
        self.rate = rospy.Rate(10)
        smach.State.__init__(self, outcomes=['outcome1'])
        self.joy = DUALSHOCK3()
        self.teleop = Teleop()

    def execute(self, userdata):
        rospy.loginfo('Teleoperation started')

        if joy_msg.buttons[self.joy.BUTTONS.RIGHT_TRIGGER]:
            return 'outcome1'
        self.rate.sleep()



class DEPTH_HOLD(smach.State):
    def __init__(self):
        self.rate = rospy.Rate(10)
        smach.State.__init__(self, outcomes=['outcome2'])
        self.teleop = Teleop
        self.teleop.pitch_pid_en = True
    def execute(self, userdata):

        rospy.loginfo('Teleoperation+Depth holding started')
        if joy_msg.buttons[self.joy.BUTTONS.RIGHT_TRIGGER]:
            return 'outcome2'
        self.rate.sleep()


# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TELEOP', TELEOP(),
                               transitions={'outcome1': 'DEPTH_HOLD'})
        smach.StateMachine.add('DEPTH_HOLD', DEPTH_HOLD(),
                               transitions={'outcome2': 'TELEOP'})


    outcome = sm.execute()


if __name__ == '__main__':
    main()