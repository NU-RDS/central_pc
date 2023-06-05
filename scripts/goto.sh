#!/usr/bin/env bash
ros2 service call /goto edward_interfaces/srv/GoTo "x: $1
y: $2
z: $3
roll: $4
pitch: $5
yaw: $6
"
