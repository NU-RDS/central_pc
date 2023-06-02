#!/usr/bin/env bash
# Commands the robot to return to its home state
# or in other words, sets all joints to zero
ros2 service call home std_srvs/srv/Empty
