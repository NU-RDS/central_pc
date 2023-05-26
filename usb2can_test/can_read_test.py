import os
import cantools
import can
import time
from matplotlib import pyplot as plt
import numpy as np

db = cantools.database.load_file('./full_bus_signed_angles.dbc')
can_bus = can.interface.Bus(channel='can0',bustype='socketcan')

# sends a sinusoidle angle between -45 and 45
while True:
    msg = can_bus.recv()
    decoded_msg = db.decode_message(msg.arbitration_id, msg.data)
    print(decoded_msg)
    time.sleep(0.5)
