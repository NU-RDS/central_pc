#!/usr/bin/env bash
# commands the robot to move to the joint angles specified
# in each line of the provided CSV file
ros2 service call csv_traj edward_interfaces/srv/CSVTraj "csv_path: $1
"
