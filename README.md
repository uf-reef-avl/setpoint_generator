# Setpoint generator


[![master Actions Status](https://github.com/uf-reef-avl/setpoint_generator/workflows/master/badge.svg)](https://github.com/uf-reef-avl/setpoint_generator/actions)


This package is used to apply a list of waypoint to a robot. It takes as argument the different waypoints of the trajectory and publish each of them depending on the robot position as PoseStamped msg. Everytime the robot reaches the current waypoint, the setpoint generator will change it to the next one in the list. This package can also publish the velocities to get to the waypoints for the reef quad rotor.


**Table of Contents**
---------------------

1. [Installation](#Installation)

2. [Dependencies](#Dependencies)

3. [ROS Topics and Messages](#ROS_Topics_and_Messages)

4. [Usage](#Usage)


<a name="Installation"/>

**Installation**
----------------

To install it, clone the setpoint_generator remote repository into your catkin workspace.

For example:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/uf-reef-avl/setpoint_generator



<a name="Dependencies"/>

**Dependencies**
----------------

This package depends mainly upon reef_msgs to retrieve the mocap position data from the robot. To install it and build the two packages, you can use:

	cd ~/catkin_ws/src
    git clone https://github.com/uf-reef-avl/reef_msgs
    cd ..
    catkin_make

As this pack can be used either on quad or on turtlebot by remapping the topics, the user can also install the [kobuki packages](http://wiki.ros.org/kobuki/Tutorials/Installation)
if he wants to use it directly on turtlebot. 


<a name="ROS_Topics_and_Messages"/>

** ROS Topics and Messages **
-------------------------

### Subscribed Topics
|Topic Name|Message Type|Description|
|--|--|--|
|sim_mocap|nav_msgs::Odometry|Position of the robot|
|pose_stamped|geometry_msgs::PoseStamped|Position of the robot|


### Published Topics
|Topic Name|Message Type|Description|
|--|--|--|
|setpoint|geometry_msgs::PoseStamped| Current waypoint published|
|desired_state|reef_msgs_msgs::DesiredState|Quad velocity to reach the current waypoint|

<a name="Usage"/>

**Usage**
---------

First of all the topics that are described above have to be remapped in the launch file. One launch file can look like this:

```xml
<launch>
	<arg name="waypoints_file" default="$(find setpoint_generator)/waypoint_files/figure_8.yaml" /> 
	<arg name="param_file" default="$(find setpoint_generator)/params/basic_param.yaml" /> 
	<node name="setpoint_publisher" pkg="setpoint_generator" type="get_setpoint_node.py" clear_params="true" output="screen">
  	<remap from="sim_mocap" to="odom"/>
  	<rosparam command="load" file="$(arg waypoints_file)"/>
  	<rosparam command="load" file="$(arg param_file"/>
	</node>
</launch>
```
In this launch file, the **waypoint file** and the **param_file** are loaded into the node in order to setup the configuration of the setpoint generator. Moreover, the **sim_mocap** topic is remapped to the **odom** topic. Then the setpoint generator will update the current waypoint published on **/setpoin** if the position published on **odom** topic match the current waypoint position. This launch file could be a gazebo scenario.

Let's take a look to an example of a waypoint file which permit to define the list of waypoints:

```xml
# list of setpoint diactionaries
# a setpoint dictionary consists of a setpoint position in meters and
# a yaw angle in degrees
waypoint_list:
  [
    {position: [5, -2.5, -1.0], yaw_degrees: -90},
    {position: [-5, 2.5, -1.0], yaw_degrees: -90}
  ]

# alteratively:
# specify a formatted CSV file
# give a filepath relative to the home directory
# example:
# waypoint_list: "catkin_ws/src/qv_controller/waypoint_files/waypoint_circle_with_yaw.csv"

# how close is "close enough" for the setpoint position
# imagine a sphere around a setpoint with a radius equal to this value
# if the origin of the vehicle's coordinate frame enters this sphere,
# the vehicle is considered to have hit the setpoint
# unit is meters
setpoint_radius_tolerance: 0.4
setpoint_z_tolerance: 0.7

# fly to each setpoint in waypoint_list this many times
# e.g. if this value is set to 2, the vehicle will fly to each setpoint in
# waypoint_list and then do it again
number_of_cycles: 4

# if this value is true, the vehicle will fly to the first setpoint after
# completing all of the other setpoints
return_to_first: false

```

Here is an example of a param file which is mostly used to defined some parameters to compute the velocity for a quad :
```xml
#if we are using mocap right now
use_mocap: true
initial_yaw: 0.0
#Depending on the type of robot, the command can be sent in the NED (quad) or NWU (turtlebot) frame
#set this value to NED or NWU and make sure that the mocap PoseStamped message are set to NED
robot_command_orientation: NED

#if the frame is in NED, write a negative altitude
altitude: -1.0

#Maximum linear velocity possibly applied to the robot
VMax: 1.
#Minimum linear velocity applied to the robot
VMin: 0.1
#Maximum angular velocity possibly applied
PhiMax: 0.8

#Gains of the controller on x
Kx: 1.
#Gains of the controlller on y
Ky: 1.
#Gains of the controller on  theta
Ktheta: 1.

#Flag for begining a run
active: false

```
