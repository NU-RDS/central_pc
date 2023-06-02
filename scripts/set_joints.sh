#!/usr/bin/env bash
# command the robot to go to the requested joint angles
ros2 service call /set_joints edward_interfaces/srv/SetJoints "joint1: $1
joint2: $2
joint3: $3
joint4: $4
joint5: $5
"
