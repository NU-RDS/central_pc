#!/usr/bin/env bash
# command the robot to go to the requested joint angles in radians
#
# For example, to command the robot to set all the joints to
# 45 degrees, or pi/4 radians, run:
# bash set_joints.sh 0.78 0.78 0.78 0.78 0.78
#
ros2 service call /set_joints edward_interfaces/srv/SetJoints "joint1: $1
joint2: $2
joint3: $3
joint4: $4
joint5: $5
"
