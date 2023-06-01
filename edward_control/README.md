# Nodes

## edward_control

### Subscriptions:
- `/joint_states`
- `/joy`

### Publishers
- `/cmd_state`

### Services
- `/csv_traj` (edward_interfaces/srv/CSVTraj)
- `/set_joints` (edward_interfaces/srv/SetJoints)
- `/home` (std_srvs/srv/Empty)


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
ros2 launch edward_control edward_bringup.launch.py robot:=real use_vr:=true
```

Note that because of the default values, the above is equivalent to the following:
```
ros2 launch edward_control edward_bringup.launch.py robot:=real use_vr:=true
```
