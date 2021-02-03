# Setpoint generator

This package is used to apply a list of waypoint to a robot. It takes as argument the different waypoints of the trajectory and publish each of them depending on the robot position as PoseStamped msg. Everytime the robot reaches the current waypoint, the setpoint generator will change it to the next one in the list. This package can also publish the velocities to get to the waypoints for the reef quad rotor.


[![master Actions Status](https://github.com/uf-reef-avl/setpoint_generator/workflows/master/badge.svg)](https://github.com/uf-reef-avl/setpoint_generator/actions)


**Table of Contents**
---------------------

1. [Installation](#Installation)

2. [Dependencies](#Dependencies)

3. [Usage](#Usage)
