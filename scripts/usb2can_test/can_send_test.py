#!/usr/bin/env python3
import os
import cantools
import can
import time
import numpy as np
import sys

try:
    db = cantools.database.load_file('./full_bus.dbc')
    main_angle_cmd_msg = db.get_message_by_name("Main_Angle_Command")
    main_torque_offset_msg = db.get_message_by_name("Main_Torque_Offset")
    can1 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
except:
    print("Can't connect to CAN bus")
    exit()

# Get commanded angles from comma separated cmd line args
angles_rad = [float(i)*np.pi/180.0 for i in sys.argv[1].split(",")]

# Send angles as specified by command line arguments
main_angle_cmd_data = main_angle_cmd_msg.encode({
    "Angle_Command_0": angles_rad[0],
    "Angle_Command_1": angles_rad[1],
    "Angle_Command_2": angles_rad[2],
    "Angle_Command_3": angles_rad[3],
    "Angle_Command_4": angles_rad[4],
    "User_Command": "Go"
})

# send all zero torques
main_torque_msg_data = main_torque_offset_msg.encode({
    "Main_Torque_Offset_CMD_0" : 0.0,
    "Main_Torque_Offset_CMD_1" : 0.0,
    "Main_Torque_Offset_CMD_2" : 0.0,
    "Main_Torque_Offset_CMD_3" : 0.0,
    "Main_Torque_Offset_CMD_4" : 0.0
})


c = 0
try:
    while True:
        angle_msg = can.Message(arbitration_id=0x600, data=main_angle_cmd_data)
        torque_msg = can.Message(arbitration_id=0x601, data=main_torque_msg_data)
        can1.send(angle_msg)
        can1.send(torque_msg)
        time.sleep(0.1)
        print(f"Sent {c}")
        c += 1
except KeyboardInterrupt:
    can1.shutdown()
