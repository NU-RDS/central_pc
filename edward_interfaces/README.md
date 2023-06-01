# edward_interfaces
Custom message and service definitions for the EdWARD ROS2 interface

## Messages

**CmdState.msg**
```
float64[] angles
float64[] torques
bool hand
```

## Services

**SetJoints.srv**
```
float64 joint1
float64 joint2
float64 joint3
float64 joint4
float64 joint5
---
```

**CSVTraj.srv**
```
string csv_path
---
```


