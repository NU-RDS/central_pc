#!/usr/bin/env bash

set_color() {
    if [ $1 -ne 0 ]; then
        # Failed command, set color to red
        echo -e "\033[0;31m" # Red color
    else
        # Successful command, set color to normal
        echo -e "\033[0;0m"
    fi
}


# Run each command, setting output to red if error
# and normal if success. Exit the program if there is an error
cmd1() {
    ifconfig can0 down
}

cmd2() {
    ip link set can0 type can bitrate 1000000
}

cmd3() {
    ifconfig can0 txqueuelen 100000
}

cmd4() {
    ifconfig can0 up
}

output=$(cmd1 2>&1)
exit_status=$?
color=$(set_color $exit_status)
if [ $exit_status -ne 0 ]; then
    echo "${color}$output${NORMAL}"
    exit
fi

output=$(cmd2 2>&1)
exit_status=$?
color=$(set_color $exit_status)
if [ $exit_status -ne 0 ]; then
    echo "${color}$output${NORMAL}"
    exit
fi

output=$(cmd3 2>&1)
exit_status=$?
color=$(set_color $exit_status)
if [ $exit_status -ne 0 ]; then
    echo "${color}$output${NORMAL}"
    exit
fi

output=$(cmd4 2>&1)
exit_status=$?
color=$(set_color $exit_status)
if [ $exit_status -ne 0 ]; then
    echo "${color}$output${NORMAL}"
    exit
else 
    echo -e "\033[32mCAN initialization successful\033[0m"
fi
