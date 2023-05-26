import os
import cantools
import can
import time
from matplotlib import pyplot as plt
import numpy as np
import sys


x = np.linspace(0,2*np.pi,100)
y = np.pi/4 * np.sin(x) + np.pi/4

db = cantools.database.load_file('./full_bus_signed_angles.dbc')
main_angle_cmd_msg = db.get_message_by_name("Main_Angle_Command")

can1 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
send_count = 0

#  main_angle_cmd_data = main_angle_cmd_msg.encode({
    #  "Angle_Command_0": float(sys.argv[1]),
    #  "Angle_Command_1": 0.0,
    #  "Angle_Command_2": 0.0,
    #  "Angle_Command_3": 0.0,
    #  "Angle_Command_4": 0.0
#  })
#  msg = can.Message(arbitration_id=0x600, data=main_angle_cmd_data)
#  can1.send(msg)
#  can1.shutdown()

# sends a sinusoidle angle between -45 and 45
while True:
    for yi in y:
        main_angle_cmd_data = main_angle_cmd_msg.encode({
            "Angle_Command_0": yi,
            "Angle_Command_1": 0.0,
            "Angle_Command_2": 0.0,
            "Angle_Command_3": 0.0,
            "Angle_Command_4": 0.0
        })
        msg = can.Message(arbitration_id=0x600, data=main_angle_cmd_data)
        can1.send(msg)
        send_count = send_count + 1
        print(yi)
        time.sleep(0.1)
