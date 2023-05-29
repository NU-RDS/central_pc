# EdWARD Central PC - ROS2 Interface
*EdWARD's Brain*

A ROS2 interface for controlling the EdWARD 5-DoF robot arm

## Quickstart
1. Clone the repository into the source space of your ROS2 workspace
```
git clone https://github.com/NU-RDS/central_pc.git
cd central_pc
cp -r edward_control/ edward_interfaces/ simple_vr_driver/ ~/ros2_ws/src/
```
2. Build and source the packages
```
colcon build
source install/setup.bash
```
4. Run the launchfile to start the required nodes
```
ros2 launch edward_control edward_bringup.launch.py
```


## TODO:
- start Toby's `simple_vr_driver VR_publisher` node from the `edward_control` launchfile
- add button toggle code for hand
- delete old/ and usb2can_test/ after checking
- test USB2CAN code in the edward_control node


