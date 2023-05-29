import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Joy

class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')
        self.previous_state = False

        # Set up the subscription to the controller
        _joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, joy_msg):
        # Read the second button state from the message to open and close the gripper
        # control.py has the zeroing button functionality
        current_state = joy_msg.buttons[1]
        if current_state != self.previous_state:
            if current_state:
                # Button pressed
                self.print_to_serial('a')
        self.previous_state = current_state

    def print_to_serial(self, message):
        serial_port = '/dev/ttyUSB0'  # Replace with the proper serial port if necessary
        baud_rate = 9600

        try:
            with serial.Serial(serial_port, baud_rate) as ser:
                ser.write(message.encode())
        except serial.SerialException as e: # Error handling if the serial communication fails
            self.get_logger().error('Serial communication error: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    button_node = ButtonNode()
    rclpy.spin(button_node)
    button_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()