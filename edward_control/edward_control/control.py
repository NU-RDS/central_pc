import numpy as np
import modern_robotics as mr
import time
from scipy.spatial.transform import Rotation
import can
import cantools
import os

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory

from edward_interfaces.msg import CmdState
from sensor_msgs.msg import JointState, Joy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, PoseStamped

from std_srvs.srv import Empty
from edward_interfaces.srv import GoTo, SetJoints, CSVTraj

from .params import Slist, Blist, M, total_length


LOG = False     # TODO: make this a parameter to the node
FREQ: int = 100 # TODO: make this a parameter to the node


class EdwardControl(Node):
    def __init__(self):
        super().__init__("edward_control")

        # declare and get parameters
        self.declare_parameter("robot", "real")
        self.declare_parameter("use_vr", True)
        self.USE_VR = self.get_parameter("use_vr").value
        self.ROBOT = self.get_parameter("robot").value

        # Create timer
        _timer = self.create_timer(1/FREQ, self.timer_callback)

        # Publishers
        if self.ROBOT == "sim":
            self.joint_pub = self.create_publisher(JointState,"/joint_states",10)
        self.cmd_state_pub = self.create_publisher(CmdState, "/cmd_state", 10)

        # Subscribers
        _joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        if self.ROBOT == "real":
            _joint_states_sub = self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        # Services
        _csv_traj_srv = self.create_service(CSVTraj, "csv_traj", self.csv_callback)
        _set_joints_srv = self.create_service(SetJoints, "set_joints", self.set_joints_callback)
        _home_srv = self.create_service(Empty, "home", self.home_callback)

        # tf listener and buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # tf broadcaster
        self.broadcaster = TransformBroadcaster(self)

        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0] # measured angles
        self.joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0] # measured torques
        self.cmd_angles = [0.0, 0.0, 0.0, 0.0, 0.0] # commanded angles
        self.cmd_torques = [0.0, 0.0, 0.0, 0.0, 0.0] # commanded torques
        self.hand_state = False # commanded hand state
        self.prev_hand_state = 0

        self.i = 0 # index for joint trajectories
        self.pose_recieved = False # flag for if a goal pose was recieved
        self.joint_traj = None # (N,5) ndarray of joint states for each point in trajectory
        self.Tse = None # EE in the S frame, from FK at each step

        # True when the trigger is pressed, thus enabling VR tracking
        self.enable_tracking = False

        # stores the zero position of the VR controller
        self.p0 = np.array([0.0, 0.0, 0.0])
        self.q0 = np.array([0.0, 0.0, 0.0, 1.0])


    def joint_states_callback(self, js_msg):
        '''
        Get the measured joint angles and torques
        '''
        # TODO: check!
        self.joint_angles = js_msg.position.tolist()
        self.joint_torques = js_msg.effort.tolist()


    def joy_callback(self, joy_msg):
        '''
        listens for button presses on the controller and acts accordingly
        '''
        # enable tracking of VR controller if trigger is pressed
        self.enable_tracking = True if joy_msg.axes[0] else False

        # zero the axes if B button is pressed
        self.zero = True if joy_msg.buttons[0] else False

        # set the hand state
        current_state = joy_msg.buttons[3]
        avg = (self.prev_hand_state + current_state) / 2.0
        self.prev_hand_state = avg
        if abs(avg) > 0.05:
            self.hand_state = True
        else:
            self.hand_state = False


    def home_callback(self, request, response):
        '''
        Brings the robot back to its home configuration
        '''
        self.cmd_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        return response


    def set_joints_callback(self, request, response):
        '''
        Commands joints to the requested positions
        '''

        self.cmd_angles = [
            request.joint1,
            request.joint2,
            request.joint3,
            request.joint4,
            request.joint5
        ]

        return response


    def run_IK(self, T_vr, eomg=0.1, ev=0.1):
        '''
        Runs inverse kinematics from the current measured EE state
        self.Tse to the goal state T_vr and sets the joint_states
        equal to the output of inverse kinematics

        Args:
            T_vr (ndarray): 4x4 homogeneous transformation matrix
            of the VR controller in the base link frame
            eomg: error tolerance on orientation
            ev: error tolerance on position
        '''

        # Extract rotation matrix and position vector from tf matrix
        R_mr, pvec = mr.TransToRp(T_vr)
        R = Rotation.from_matrix(R_mr).as_euler("xyz")

        # Solve IK and set the joint angles
        if self.enable_tracking:
            result = mr.IKinSpace(Slist, M, T_vr, self.joint_angles, eomg=eomg, ev=ev)
            if result[1]:
                self.cmd_angles = list(result[0])
                self.cmd_angles[1] = -self.cmd_angles[1] # TODO: why??
            else:
                self.get_logger().warn(f"IK failed to converge")


    def csv_callback(self, request, response):
        '''
        Gets trajectory of joint states from a CSV file.
        '''

        self.pose_recieved = True
        csv_path = request.csv_path
        self.joint_traj = np.loadtxt(csv_path,delimiter=",")
        return response


    def timer_callback(self):

        if self.ROBOT == "sim":
            # if just simulating, set the sensed joint states equal to the commanded ones
            self.joint_angles = self.cmd_angles

        if self.USE_VR:
            try:
                # lookup the transform between world and controller_1
                t = self.tf_buffer.lookup_transform("world", "controller_1",rclpy.time.Time())

                # Get the current rotation(quaternion) and position vector
                q = np.array([
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ])
                p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])

                # set the zero to the current tf if the button is clicked
                if self.zero:

                    #TODO probably just do this:
                    #  self.q0 = q.copy()
                    #  self.p0 = p.copy()

                    self.q0 = np.array([
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w
                    ])
                    self.p0 = np.array([
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z
                    ])

                # Compute difference of current pose and zero pose
                p_diff = p - self.p0
                q_euler = Rotation.from_quat(q).as_euler("xyz")
                q0_euler = Rotation.from_quat(self.q0).as_euler("xyz")
                if not np.allclose(q_euler,q0_euler): # maybe can remove
                    q_diff_euler = q_euler - q0_euler
                    q_diff = Rotation.from_euler("xyz", q_diff_euler).as_quat()
                else:
                    q_diff = self.q0

                # Get transformation from base_link to VR controller
                T_BL_VR = mr.RpToTrans(Rotation.from_quat(q_diff).as_matrix(), p_diff)

                # broadcast a TF between the base_link and the zero'd "controller_delta" frame
                self.current_time = self.get_clock().now()
                BL_VR_tf_msg = TransformStamped()
                BL_VR_tf_msg.transform.translation.x = p_diff[0]
                BL_VR_tf_msg.transform.translation.y = p_diff[1]
                BL_VR_tf_msg.transform.translation.z = p_diff[2]
                BL_VR_tf_msg.transform.rotation.x = q_diff[0]
                BL_VR_tf_msg.transform.rotation.y = q_diff[1]
                BL_VR_tf_msg.transform.rotation.z = q_diff[2]
                BL_VR_tf_msg.transform.rotation.w = q_diff[3]
                BL_VR_tf_msg.header.stamp = self.current_time.to_msg()
                BL_VR_tf_msg.header.frame_id = "base_link"
                BL_VR_tf_msg.child_frame_id = "controller_delta"
                self.broadcaster.sendTransform(BL_VR_tf_msg)

                # Solve IK so BL->EE matches BL->VR
                self.run_IK(T_BL_VR)

            except TransformException as ex:
                self.get_logger().warn(f'Could not transform "world" to "controller_1": {ex}')

        #  update joint angles at each step if pose recieved from CSVTraj services
        if self.pose_recieved:
            self.cmd_angles = [
                self.joint_traj[self.i,0],
                self.joint_traj[self.i,1],
                self.joint_traj[self.i,2],
                self.joint_traj[self.i,3],
                self.joint_traj[self.i,4],
            ]
            self.i += 1

            if self.i == np.shape(self.joint_traj)[0] - 1:
                self.i = 0
                self.pose_recieved = False


        # construct and publish a JointState message if just simulating
        if self.ROBOT == "sim":
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = ["joint1","joint2","joint3","joint4","joint5"]
            js_msg.position = self.joint_angles
            self.joint_pub.publish(js_msg)

        # publish the commanded anlges and torques on /cmd_state
        cmd_state_msg = CmdState()
        cmd_state_msg.angles = self.cmd_angles
        cmd_state_msg.torques =self.cmd_torques
        cmd_state_msg.hand = bool(self.hand_state)
        self.cmd_state_pub.publish(cmd_state_msg)

        # Get the current end effector pose
        self.Tse = mr.FKinSpace(M, Slist, self.joint_angles)





def main(args=None):
    '''
    main function/node entry point
    '''
    rclpy.init(args=args)
    node = EdwardControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


