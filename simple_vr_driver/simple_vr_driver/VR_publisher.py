#!/usr/bin/env python3
import math
import simple_vr_driver.triad_openvr as triad_openvr
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy

import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R




class VRPublisher(Node):

    def __init__(self):
        super().__init__('VR_publisher')

        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)

        # timer
        self.timer = self.create_timer(0.01, self.core)


    def core(self):
        # get new info
        for device_name in self.v.devices:
            device = self.v.devices[device_name]
            try:
                out = device.get_pose_quaternion() # vector
                x,y,z,r_w,r_x,r_y,r_z = out
                v = [x, y, z]
                q_vec = [r_x, r_y, r_z, r_w]
            except:
                continue

            state_dct = device.get_controller_inputs()

            # Rotation 
            # 90 deg about X
            r = R.from_euler('x', 90, degrees=True)
            vf = r.apply(v)
            q = R.from_quat(q_vec)
            qf = r * q
            r_x, r_y, r_z, r_w = qf.as_quat()
            x, y, z = vf

            # ROS stuff
            t = TransformStamped()

            # Read message content and assign it to corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = device_name

            t.transform.translation.x = x; t.transform.translation.y = y; t.transform.translation.z = z
            t.transform.rotation.x = r_x; t.transform.rotation.y = r_y; t.transform.rotation.z = r_z; t.transform.rotation.w = r_w

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            # Joystick state
            j = Joy()
            j.axes.append(state_dct['trigger'])
            j.axes.append(state_dct['trackpad_x'])
            j.axes.append(state_dct['trackpad_y'])
            j.buttons.append(state_dct['menu_button'])
            j.buttons.append(state_dct['trackpad_pressed'])
            j.buttons.append(state_dct['trackpad_touched'])
            j.buttons.append(state_dct['grip_button'])

            self.publisher_.publish(j)



def main(args=None):
    rclpy.init(args=args)
    node = VRPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
