#!/usr/bin/env python
# !/usr/bin/env python
import rospy
import config.msg
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion
import math


class Go_to_goal:

    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.cfg_subscriber = rospy.Subscriber("config", config.msg, self.config_update)
        self.pose_subscriber = rospy.Subscriber("/fiducial_pose", geometry_msgs.msg.PoseWithCovarianceStamped,
                                    self.update_pose)
        self.cmd_publisher_ = rospy.Publisher('twist_command', geometry_msgs.msg.Twist, queue_size=1)
        self.mode = 0
        self.pose = geometry_msgs.msg.Pose
        self.rate = rospy.Rate(10)
        self.pose_quaternion = geometry_msgs.quaternion
        self.pose_x = 0
        self.pose_y = 0
        self.theta = 0
        self.max_effort_ = 1.0
        self.sum_error = 0

        self.prev_angle_er = 0.1
        self.Cu = 180
        self.Cp = 0.6 * self.Cu
        self.Ci = 2 * self.Cp / self.Cu
        self.Cd = self.Cp * self.Cu / 8
        self.prev_distance_er = 1
        self.distance_er_summ = 0
        self.Cu_speed = 120
        self.Cp_speed = 0.6 * self.Cu_speed
        self.Ci_speed = 2 * self.Cp_speed / self.Cu_speed
        self.Cd_speed = 2 * self.Cp_speed * self.Cu_speed / 8
    def config_update(self,data):
        self.goal_pose_x=data.x_goal
        self.goal_pose_y = data.y_goal
        self.mode = data.mode

    def update_angle(self, data):
        rot_q = data
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.theta = theta

    def update_pose(self, data):
        self.pose = data.PoseWithCovariance.Pose
        self.pose_x = self.pose.point.x
        self.pose_y = self.pose.point.y
        self.pose_quaternion = self.pose.quaternion
        self.update_angle(self.pose_quaternion)

    def euclidean_distance(self, goal_pose_x, goal_pose_y):
        """Euclidean distance between current pose and the goal."""

        b = sqrt(pow(goal_pose_x - self.pose.x, 2) + pow(goal_pose_y - self.pose.y, 2))

        return b

    def linear_vel(self, goal_pose_x, goal_pose_y):
        self.Et = self.euclidean_distance(goal_pose_x, goal_pose_y)
        self.Pt_speed = self.Cp_speed * self.Et
        self.It_speed = self.distance_er_summ + self.Ci_speed * self.Et
        self.Dt_speed = self.Cd_speed * self.prev_distance_er
        self.prev_distance_er = self.Et
        self.distance_er_summ = self.It_speed
        return int(self.Pt_speed + self.It_speed + self.Dt_speed)

    def steering_angle(self, goal_pose_x, goal_pose_y):
        p = atan2(goal_pose_x - self.pose_x, goal_pose_y - self.pose_y)
        # print(abs(self.theta), abs(p))

        return p

    def angular_vel(self, goal_pose_x, goal_pose_y, ):
        self.et = abs(self.steering_angle(goal_pose_x, goal_pose_y)) - abs(self.theta)
        self.Pt = self.Cp * self.et
        self.It = self.sum_error + self.Ci * self.et
        self.Dt = self.Cd * self.prev_angle_er
        self.sum_error = self.It
        self.prev_angle_er = self.et
        return int(self.Pt + self.It + self.Dt)

    def move2goal(self):
        distance_tolerance = 0.3
        self.goal_pose_x = 0
        self.goal_pose_y = 0



        while self.euclidean_distance(self.goal_pose_x, self.goal_pose_y) >= distance_tolerance:
            twist_msg = geometry_msgs.msg.Twist()
            ### TODO Отладить pid
            twist_msg.linear.x = self.linear_vel(self.goal_pose_x, self.goal_pose_y) * self.max_effort_
            # Рассмотреть возможность движение боком и движение вертикально
            # twist_msg.linear.y =  * self.max_effort_
            # twist_msg.linear.z =  * self.max_effort_
            twist_msg.angular.z = self.angular_vel(self.goal_pose_x, self.goal_pose_y) * self.max_effort_ * 0.5

            self.cmd_publisher_.publish(twist_msg)

            self.rate.sleep()


        rospy.spin()


if __name__ == '__main__':
    try:
        x = Go_to_goal()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
