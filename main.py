from inverse_dyn import InverseDynamics
from inverse_kin import IKinSpace
import mr_helpers as mr
import numpy as np
import time

from bot_parameters import *
from helpers import *

#-----------INNOMAKER USB2CAN DEVICE---------------# 

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import (
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
)
#below macro is for gs_usb 0.3.0 and above
from gs_usb.gs_usb import (
    GS_CAN_MODE_NORMAL,
    GS_CAN_MODE_LISTEN_ONLY,
    GS_CAN_MODE_LOOP_BACK,
)

# gs_usb general also can import from gs_usb_structures.py
GS_USB_ECHO_ID = 0
GS_USB_NONE_ECHO_ID = 0xFFFFFFFF

#number of CAN frames to expect to arrive at central PC in one cycle
NUM_FRAMES = 10
   
#----------------------ROS------------------------# 

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix #the ROS TF, not Tensorflow


def callback(data):
    global world_coords, world_rotmatrix
    world_coords = data.pose.position
    quaternion = (data.pose.orientation.x, data.pose.orientation.y,
                  data.pose.orientation.z, data.pose.orientation.w)
    world_rotmatrix = quaternion_matrix(quaternion)[:3, :3]

#note: ChatGPT says to run the following to gt geometry_msgs on the Jetson
#sudo apt-get install ros-<distro>-geometry-msgs
#distro == "noetic", e.g.

#-------------------------------------------------# 


def main():

    #find the Innomaker USB2CAN device
    devs = GsUsb.scan()
    if len(devs) == 0:
        raise Exception("Can not find gs_usb device")
        
    #default usb2can device is devs[0] with gs_usb 0.2.9 ,and devs[1] with gs_usb 0.3.0
    dev = devs[1]
    print(dev)   
    
    #start listening to CAN bus 
    BAUD = 1000000
    if not dev.set_bitrate(BAUD):
        raise Exception("Can not set bitrate for gs_usb")
        
    dev.start(GS_CAN_MODE_NORMAL)

    #initialize CAN frames. We expect 2 from each joint; one for the encoder count, one for the
    #ADC counts for the torque sensor
    can_frames = [None] * NUM_FRAMES
    for ii in range(NUM_FRAMES):
        can_frames.append(GsUsbFrame())

    success_arr = np.zeros(NUM_FRAMES)
    joint_angles_prev = np.zeros(5) #previous commanded torques for inverse kinematics

    '''NOTE: This ROS part is ChatGPT-generated. It may be nonsense/need to be checked by a human.
    '''

    # Initialize the ROS node and subscriber
    rospy.init_node('vive_listener')
    sub = rospy.Subscriber('/vive/left_hand_pose', PoseStamped, callback)
    
    # Spin the ROS node to keep it listening for messages
    world_coords = None
    world_rotmatrix = None
    rospy.spin()

    while (1):
        #read raw frames from each of the joint teensies. this will be the order in which they're
        #sent (enforced by each joint listening for the previous joint's message ID)
        for ii in range(NUM_FRAMES):
            success_arr[ii] = dev.read(num_frames[ii], 1) #returns a boolean as to whether it was received

        #convert data in the CAN frames to usable integers (encoder counts + ADC counts of torque sensor)
        frame_data = convert_can_data(can_frames)
        encoder_ct_arr = frame_data[0::2]
        torque_adc_ct_arr = frame_data[1::2]
        torque_val_arr = adc_ct_to_joint_torques(torque_adc_ct_arr)
        joint_angle_arr = convert_encoder_cts_to_angles(encoder_ct_arr)


        #Get ROS data and transform it. These two variables get modified in the callback function.
        Tsb = mr.RpToTrans(world_coords, world_rotmatrix) 

        #run inverse kinematics based on Vive position; inverse dynamics based on current joint angles
        id_torques = InverseDynamics(joint_angle_arr, g, Force, Mlist, Glist, Slist)
        joint_angles_next = IKinSpace(Slist, M, Tsb, joint_angles_prev, eomg, ev)

        #send the ID torques and next joint angles back out over the CAN bus
        #TODO
        

    

if __name__ == '__main__':
    main()