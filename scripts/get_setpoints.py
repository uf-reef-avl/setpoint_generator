#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import RCRaw
from reef_msgs.msg import DesiredState


class setpoint_publisher:
    """
    @class Setpoint publisher This Ros class subscribes to a pose and load a list of waypoint at initialization. Then it will publish the first
    waypoint in the list until the subscribed pose reaches it (depending on a tolerance radius). Then it will publish
    the second waypoint and so one. For quad, it will publish velocity to reach the waypoint instead of the waypoint actual pose.
    """
    def __init__(self):
        """ @brief Constructor of the class which initialize the waypoint list, the ros publisher/subscriber and differents parameters
        """
        #define if the setpoint_generator is used in a simulation environement or in mocap ( it will change the pose subscriber message type)
        is_sim = rospy.get_param("is_sim",default='false')
        use_mocap = rospy.get_param("use_mocap",default='true')
        self.mocap_sub = 0

        #theses parameters set the VMax, Vmin (minmum and maximum linear velocity) and PhiMax (maximum angular velocity) of the robot
        self.VMax = rospy.get_param("~VMax")
        self.VMin = rospy.get_param("~VMin")
        self.PhiMax = rospy.get_param("~PhiMax")

        #initialize the yaw of the quad
        self.initial_yaw = 0
        #initialize the list index
        self.waypoint_index = 0
        self.waypoint_list = rospy.get_param("~waypoint_list")

        # initialize first waypoint message using first waypoint
        self.setpoint_msg = self.pose_msg_from_dict(self.waypoint_list[self.waypoint_index])
        self.output_change_of_waypoint()
        #define how much time the setpoint generator will go through the list
        num_cycles = rospy.get_param("~number_of_cycles")
        if type(num_cycles) == int and num_cycles > 0:
            self.waypoint_list *= rospy.get_param("~number_of_cycles")
        #define if the pose has to go back at the initial waypoint at the end of waypoint list
        if rospy.get_param("~return_to_first"):
            self.waypoint_list.append(self.waypoint_list[0])
        #initialize the waypoint tolerance radius
        self.setpoint_radius_tolerance = rospy.get_param("~setpoint_radius_tolerance")
        self.setpoint_z_tolerance = rospy.get_param("~setpoint_z_tolerance")

        # initialize first desired state message using first waypoint
        self.current_x = 0
        self.current_y = 0
        self.quad_desired_state_msg = DesiredState()
        self.quad_desired_state_msg.pose.x = 0
        self.quad_desired_state_msg.pose.y = 0
        self.quad_desired_state_msg.pose.yaw = 0
        self.quad_desired_state_msg.velocity_valid = True

        # initialize waypoint publisher
        self.setpoint_pub = rospy.Publisher("setpoint", PoseStamped, queue_size=10)

        #the publisher below are used to send commands to the quad
        self.quad_desired_state_pub = rospy.Publisher("desired_state", DesiredState, queue_size=10)

        # initialize pose subscriber
        if use_mocap:
            self.mocap_sub = rospy.Subscriber("pose_stamped", PoseStamped, self.mocap_callback)
            print("IN USE MOCAP ",  use_mocap)

        elif is_sim:
            self.mocap_sub = rospy.Subscriber("sim_mocap", Odometry, self.mocap_sim_callback)
            print("IN USE SIM")


    def spin(self):
        """
         @brief The spin loop which publish the current waypoint to reach or the real time velocity to apply to permit to the quad to reach the waypoint
        """
        self.setpoint_msg.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.setpoint_msg)

        active = rospy.get_param("~active")
        self.setDesiredState(active)

        self.quad_desired_state_msg.header.stamp = rospy.Time.now()
        self.quad_desired_state_pub.publish(self.quad_desired_state_msg)


    def setDesiredState(self, active):
        """
         @brief This function compute the velocity to apply for a quad to reach the current waypoint
         @param[in] active is a boolean who define if the setpoint publisher is ready to publish velocity or not
        """
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
        """
         @brief This function takes a setpoint in a dict format as entry and return it as a PoseStamped msg
         @param[in] setpoint_dict is a setpoint formatted as a python dict
         @return return a setpoint as a PoseStamped message
        """
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
        """
         @brief This function print some information statement when the setpoint_publisher change his current setpoint
        """
        info_str = "Requesting setpoint %d of %d: position (m): %s, yaw (degrees): %d." %(
            self.waypoint_index+1, len(self.waypoint_list),
            str(self.waypoint_list[self.waypoint_index]["position"]),
            self.waypoint_list[self.waypoint_index]["yaw_degrees"])
        rospy.logwarn(info_str)


    def mocap_callback(self, mocap_pose_msg):
        """
         @brief This function permit to retrieve the current pose of the robot thanks to the ros_subscriber in mocap.
         It also update the current setpoint if the pose of the robot is located in the current setpoint radius.
         @param[in] mocap_pose_msg is the ros subscribed pose of the robot
        """
        current_xy, current_z = self.dist_from_setpoint(mocap_pose_msg)

        if current_xy < self.setpoint_radius_tolerance and current_z < self.setpoint_z_tolerance:
            if self.waypoint_index < len(self.waypoint_list) - 1:
                self.waypoint_index += 1
                #time.sleep(2)
                self.output_change_of_waypoint()
                self.setpoint_msg = self.pose_msg_from_dict( self.waypoint_list[self.waypoint_index] )


    def mocap_sim_callback(self,mocap_sim_msg):
        """
         @brief This function permit to retrieve the current pose of the robot thanks to the ros_subscriber in gazebo simulation.
         @param[in] mocap_sim_msg is the ros subscribed pose of the robot
        """
        mocap_msg = PoseStamped()
        mocap_msg.pose = mocap_sim_msg.pose.pose
        self.mocap_callback(mocap_msg)


    def dist_from_setpoint(self, current_pose_msg):
        """
         @brief This function compute the distance between the current position of the robot and the current setpoint in the xy plane and in z axis.
         @param[in] current_pose_msg is the ros subscribed pose of the robot
         @return a tuple composed of the the xy distance between the robot and the current waypoint and the z distance between them
        """
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
