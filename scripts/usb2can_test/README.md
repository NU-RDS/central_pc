# USB2CAN
These scripts demonstrate how to command 
joint angles and torques on the CAN bus directly
without using ROS.

**Note:** if you see "Transmit buffer overflow" when attempting
to use this code, most likely your CAN bus is not connected
properly and you should check the wiring.

Example usage:
```
# initialize CAN0
sudo bash can_bringup.sh 

# Command angles to 45,0,0,0,0
python3 can_send_test.py 45,0,0,0,0 
```
