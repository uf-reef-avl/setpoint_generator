#!/usr/bin/env python
import os
import rospy
import tf
import numpy as np
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import RCRaw
from reef_msgs.msg import DesiredState


class setpoint_publisher:

    def __init__(self):

        is_sim = rospy.get_param("is_sim",default='false')
        use_mocap = rospy.get_param("use_mocap",default='true')
        self.mocap_sub = 0

        #theses parameters set the VMax, Vmin (minmum and maximum linear velocity) and PhiMax (maximum angular velocity) of the robot
        self.VMax = rospy.get_param("~VMax")
        self.VMin = rospy.get_param("~VMin")
        self.PhiMax = rospy.get_param("~PhiMax")

        self.initial_yaw = 0

        self.waypoint_index = 0
        self.waypoint_list = rospy.get_param("~waypoint_list")

        # initialize first waypoint message using first waypoint
        self.setpoint_msg = self.pose_msg_from_dict(self.waypoint_list[self.waypoint_index])
        self.output_change_of_waypoint()

        num_cycles = rospy.get_param("~number_of_cycles")
        if type(num_cycles) == int and num_cycles > 0:
            self.waypoint_list *= rospy.get_param("~number_of_cycles")
            #print num_cycles

        if rospy.get_param("~return_to_first"):
            self.waypoint_list.append(self.waypoint_list[0])

        self.setpoint_radius_tolerance = rospy.get_param("~setpoint_radius_tolerance")
        self.setpoint_z_tolerance = rospy.get_param("~setpoint_z_tolerance")

        self.current_x = 0
        self.current_y = 0
        self.quad_desired_state_msg = DesiredState()
        self.quad_desired_state_msg.pose.x = 0
        self.quad_desired_state_msg.pose.y = 0
        self.quad_desired_state_msg.pose.yaw = 0
        self.quad_desired_state_msg.velocity_valid = True

        self.setpoint_pub = rospy.Publisher("setpoint", PoseStamped, queue_size=10)

        #the publisher below are used to send commands to the quad
        self.quad_desired_state_pub = rospy.Publisher("desired_state", DesiredState, queue_size=10)

        if use_mocap:
            self.mocap_sub = rospy.Subscriber("pose_stamped", PoseStamped, self.mocap_callback)
            print("IN USE MOCAP ",  use_mocap)

        elif is_sim:
            self.mocap_sub = rospy.Subscriber("sim_mocap", Odometry, self.mocap_sim_callback)
            print("IN USE SIM")


    def spin(self):

        self.setpoint_msg.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.setpoint_msg)

        active = rospy.get_param("~active")
        self.setDesiredState(active)

        self.quad_desired_state_msg.header.stamp = rospy.Time.now()
        self.quad_desired_state_pub.publish(self.quad_desired_state_msg)


    def setDesiredState(self, active):

        if active:
            self.quad_desired_state_msg.velocity.x = self.setpoint_msg.pose.position.x - self.current_x
            if (math.fabs(self.setpoint_msg.pose.position.x - self.current_x)) > self.VMax:
                if (self.setpoint_msg.pose.position.x - self.current_x) > 0:
                    self.quad_desired_state_msg.velocity.x = self.VMax
                else:
                    self.quad_desired_state_msg.velocity.x = -self.VMax


            self.quad_desired_state_msg.velocity.y = self.setpoint_msg.pose.position.y - self.current_y
            if (math.fabs(self.setpoint_msg.pose.position.y - self.current_y)) > self.VMax:
                if (self.setpoint_msg.pose.position.y - self.current_y) > 0:
                    self.quad_desired_state_msg.velocity.y = self.VMax
                else:
                    self.quad_desired_state_msg.velocity.y = -self.VMax

        else:
            self.quad_desired_state_msg.velocity.x = 0 - self.current_x
            if (math.fabs(0 - self.current_x)) > self.VMax:
                if (0 - self.current_x) > 0:
                    self.quad_desired_state_msg.velocity.x = self.VMax
                else:
                    self.quad_desired_state_msg.velocity.x = -self.VMax

            self.quad_desired_state_msg.velocity.y = 0 - self.current_y
            if (math.fabs(0 - self.current_y)) > self.VMax:
                if (0 - self.current_y) > 0:
                    self.quad_desired_state_msg.velocity.y = self.VMax
                else:
                    self.quad_desired_state_msg.velocity.y = -self.VMax

        #self.quad_desired_state_msg.velocity.yaw = self.setvelocity_msg.angular.z
        self.quad_desired_state_msg.pose.z = self.setpoint_msg.pose.position.z


    def pose_msg_from_dict(self,setpoint_dict):

        pose_msg = PoseStamped()
        euler_xyz = [0,0,np.deg2rad(setpoint_dict["yaw_degrees"])]
        q = tf.transformations.quaternion_from_euler(*euler_xyz,axes="rxyz")

        pose_msg.pose.position.x = setpoint_dict["position"][0]
        pose_msg.pose.position.y = setpoint_dict["position"][1]
        pose_msg.pose.position.z = setpoint_dict["position"][2]

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        return pose_msg


    def output_change_of_waypoint(self):

        info_str = "Requesting setpoint %d of %d: position (m): %s, yaw (degrees): %d." %(
            self.waypoint_index+1, len(self.waypoint_list),
            str(self.waypoint_list[self.waypoint_index]["position"]),
            self.waypoint_list[self.waypoint_index]["yaw_degrees"])
        rospy.logwarn(info_str)


    def mocap_callback(self, mocap_pose_msg):
        current_xy, current_z = self.dist_from_setpoint(mocap_pose_msg)

        if current_xy < self.setpoint_radius_tolerance and current_z < self.setpoint_z_tolerance:
            if self.waypoint_index < len(self.waypoint_list) - 1:
                self.waypoint_index += 1
                #time.sleep(2)
                self.output_change_of_waypoint()
                self.setpoint_msg = self.pose_msg_from_dict( self.waypoint_list[self.waypoint_index] )


    def mocap_sim_callback(self,mocap_sim_msg):
        mocap_msg = PoseStamped()
        mocap_msg.pose = mocap_sim_msg.pose.pose
        self.mocap_callback(mocap_msg)


    def dist_from_setpoint(self, current_pose_msg):
        self.current_x = current_pose_msg.pose.position.x
        self.current_y = current_pose_msg.pose.position.y

        current_pose_xy = np.array([ current_pose_msg.pose.position.x,
                                    current_pose_msg.pose.position.y])
        current_pose_z = np.array([ current_pose_msg.pose.position.z])

        waypoint_position_xy = np.array([ self.setpoint_msg.pose.position.x,
                                       self.setpoint_msg.pose.position.y])
        waypoint_position_z = np.array([ self.setpoint_msg.pose.position.z])

        dist_xy = np.linalg.norm(current_pose_xy - waypoint_position_xy)
        dist_z  = np.linalg.norm(current_pose_z  - waypoint_position_z )

        return[dist_xy,dist_z]
