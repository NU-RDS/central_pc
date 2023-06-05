#!/usr/bin/env bash
# Commands the robot to move to the joint angles specified
# in each line of the provided CSV file.
#
# Each line of the CSV file should contain exactly 5
# comma separated joint angles.
# Example:
# file.csv:
# 0.0,0.0,0.0,0.0,0.0
# 0.1,0.1,0.1,0.1,0.1
# 0.2,0.2,0.2,0.2,0.2
#
# Then, simply run "bash follow_csv.sh file.csv"


ros2 service call csv_traj edward_interfaces/srv/CSVTraj "csv_path: $1
"
