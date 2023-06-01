# EdWARD Central PC - ROS2 Interface
A ROS2 interface for controlling the EdWARD 5-DoF robot arm

<p align="center">
  <img width="300" src="https://github.com/NU-RDS/central_pc/assets/45540813/28931dba-c25b-4e03-a1a6-eee9c942c9c7">
</p>

## Quickstart
1. Clone the repository into the source space of your ROS2 workspace
```
cd ~/ros2_ws/src/ # or wherever you want
git clone https://github.com/NU-RDS/central_pc.git
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
- test button toggle code for hand
- delete old/ and usb2can_test/ after checking
- test USB2CAN code in the edward_control node
- update this README to explain launchfile and node parameters etc.


