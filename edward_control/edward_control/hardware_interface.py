'''
The Hardware Interface Node handles the communication
over the CAN bus for commanding and reading joint states
from the EdWARD robot arm. It also communicates with the
hand over serial.

Subscribes:
    /cmd_state: commanded joint angles, torques, and hand state
Publishes:
    /joint_states: measured joint angles and torques
'''

import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory
import can
import atexit
import cantools
import os
import serial
from sensor_msgs.msg import JointState

from edward_interfaces.msg import CmdState

class HardwareInterface(Node):
    def __init__(self):
        super().__init__("hardware_interface")

        # declare and get parameters
        self.declare_parameter("rate",100)
        self.declare_parameter("serial_port_name","/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 9600)
        freq = self.get_parameter("rate").value
        serial_port_name = self.get_parameter("serial_port_name").value
        baud = self.get_parameter("serial_baud").value

        # load the DBC file
        dbc_file_path = os.path.join(
            get_package_share_directory("edward_control"),
            "config/full_bus.dbc"
        )
        self.db = cantools.database.load_file(dbc_file_path)

        # instantiate the CAN bus
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Connect to serial port
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(serial_port_name, baud)
            atexit.register(self.close_serial_port) # be sure to close port at exit
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {str(e)}\nHand disabled")

        # Create timer
        _timer = self.create_timer(1/freq, self.timer_callback)

        # Publishers
        self.joint_pub = self.create_publisher(JointState,"joint_states",10)

        # Subscribers
        _cmd_state_sub = self.create_subscription(
            CmdState,"/cmd_state", self.cmd_state_callback, 10
        )

        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0]


    def close_serial_port(self):
        # Close serial port
        if self.serial_port.is_open:
            self.serial_port.close()

        # shutdown CAN bus
        self.can_bus.shutdown()


    def cmd_state_callback(self, cmd_state_msg):
        '''
        Sends commanded angles and torques on the CAN bus.
        Commanded hand state is sent over serial
        '''

        angles = cmd_state_msg.angles.tolist()
        torques = cmd_state_msg.torques.tolist()
        hand_state = cmd_state_msg.hand

        main_angle_cmd_msg = self.db.get_message_by_name("Main_Angle_Command")
        main_torque_offset_msg = self.db.get_message_by_name("Main_Torque_Offset")

        main_angle_cmd_data = main_angle_cmd_msg.encode({
            "Angle_Command_0": angles[0],
            "Angle_Command_1": angles[1],
            "Angle_Command_2": angles[2],
            "Angle_Command_3": angles[3],
            "Angle_Command_4": angles[4],
            "User_Command": "Go"
        })

        main_torque_offset_data = main_torque_offset_msg.encode({
            "Main_Torque_Offset_CMD_0" : torques[0],
            "Main_Torque_Offset_CMD_1" : torques[1],
            "Main_Torque_Offset_CMD_2" : torques[2],
            "Main_Torque_Offset_CMD_3" : torques[3],
            "Main_Torque_Offset_CMD_4" : torques[4]
        })

        # construct the messages and send on the CAN bus
        angle_msg = can.Message(arbitration_id=0x600, data=main_angle_cmd_data)
        torque_msg = can.Message(arbitration_id=0x601, data=main_torque_offset_data)
        #  self.can_bus.send(angle_msg)
        #  self.can_bus.send(torque_msg)

        # if serial port is connected, send command to hand accordingly
        if self.serial_port is not None:
            if hand_state:
                self.serial_port.write('a'.encode())

    def read_joint_states(self):
        '''
        Reads joint angles and torques over CAN bus
        and publishes them on as a JointState msg on
        the /joint_states topic
        '''
        #  self.joint_angles, self.joint_torques = read_can()
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ["joint1","joint2","joint3","joint4","joint5"]
        js_msg.position = self.joint_angles
        self.joint_pub.publish(js_msg)


    def timer_callback(self):

        # reads joint states over CAN, publishes on /joint_states
        self.read_joint_states()


def main(args=None):
    '''
    main function/node entry point
    '''
    rclpy.init(args=args)
    node = HardwareInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


