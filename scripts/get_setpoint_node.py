#!/usr/bin/env python2.7
import rospy
from get_setpoints import setpoint_publisher

rospy.init_node("setpoint_publish_node",anonymous=False)
waypoint_publisher = setpoint_publisher()

loop_rate = rospy.Rate(100)

while not rospy.is_shutdown():
    waypoint_publisher.spin()
    loop_rate.sleep()
