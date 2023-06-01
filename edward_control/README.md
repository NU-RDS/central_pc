# Nodes

## edward_control
The `edward_control` node is responsible for solving inverse kinematics
to determine the joint angles and torques to command to the motors 
so the robot arm can track the VR controller. Furthermore, this node
defines several services for controlling the robot in very basic ways
without a VR controller.

### Subscribes:
- `/joint_states`
- `/joy`

### Publishes
- `/cmd_state`

### Services
- `/csv_traj` (edward_interfaces/srv/CSVTraj)
  - command the robot to follow the joint trajectories defined in a CSV file
- `/set_joints` (edward_interfaces/srv/SetJoints)
  - enables the user to command the joint angles directly
- `/home` (std_srvs/srv/Empty)
  - commands the joint angles to go to their home positions

## hardware_interface
The `hardware_interface` node is responsible for communication with the 
actual robot. This node sends the commanded states, from the `/cmd_state`
topic, on the CAN bus using the USB2CAN module connected to the computer.
The hand state field of the `CmdState` message is sent over serial.


### Subscribes
- `/cmd_state`: commanded joint angles, torques, and hand state

### Publishes:
- `/joint_states`: measured joint angles and torques


# Launchfile
The `edward_bringup.launch.py` launchfile will start all the necessary nodes.

## Optional Arguments

To view the optional arguments to the launchfile, run:
```
ros2 launch edward_control edward_bringup.launch.py --show-args 
```

For example, to run the edward_ros2 interface on the real robot and using the VR
controller, run the following command:
```
ros2 launch edward_control edward_bringup.launch.py
```

Note that because of the default values, the above is equivalent to the following:
```
ros2 launch edward_control edward_bringup.launch.py robot:=real use_vr:=true
```
