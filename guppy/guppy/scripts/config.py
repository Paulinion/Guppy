#!/usr/bin/python
import rospy
import config.msg

if __name__ == '__main__':
    rate = rospy.Rate(10)
    config = config.msg
    cfg_publisher = rospy.Publisher('config', config.msg, queue_size=1)

    config.x_goal = 0
    config.y_goal = 0
    config.Depth = 0
    config.Mode = 1


    
    while not rospy.is_shutdown():
        cfg_publisher.publish(config)
        rate.sleep()
