# EdWARD - ROS2 Interface
A ROS2 interface for teleoperated control of the EdWARD 5-DoF robot arm
using the Valve Index virtual reality controller.

<p align="center">
  <img width="300" src="https://github.com/NU-RDS/central_pc/assets/45540813/28931dba-c25b-4e03-a1a6-eee9c942c9c7">
</p>


## Dependencies
- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)
- [modern_robotcs](https://github.com/NxRLab/ModernRobotics)
- `numpy`
- `scipy`
- `cantools`


## Packages
A brief description of the packages in this repository is provided
below. For more details, see the README's in each package.

**edward_control**
- Commands joint angles and torques by publishing
`CmdState` messages on the `/cmd_state` topic. 
- Listens to transform data to obtain the 3D pose of the VR
controller, and does forward kinematics to enable teleoperation of the robot

**edward_interfaces**
- Defines custom message and service types

**simple_vr_driver**
- Publishes transform data on `/tf` of the VR controller
- Publishes controller button state information on the `/joy` topic


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


