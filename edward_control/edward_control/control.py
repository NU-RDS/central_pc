import numpy as np
import time
import modern_robotics as mr
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from visualization_msgs.msg import Marker
from edward_interfaces.srv import GoTo, SetJoints, CSVTraj
from std_srvs.srv import Empty

from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import can
import cantools
import os

from .params import Slist, Blist, M, total_length


LOG = False     # TODO: make this a parameter to the node
FREQ: int = 100 # TODO: make this a parameter to the node


def send_can_cmd(angles, torques):
    '''
    Sends requested angles and torques on the CAN bus

    Args:
        angles: length 5 array of commanded angles
        torques: length 5 array of commanded torque offsets
    '''

    #TODO this has only been tested outside of the ROS node

    main_angle_cmd_msg = db.get_message_by_name("Main_Angle_Command")
    main_torque_offset_msg = db.get_message_by_name("Main_Torque_Offset")

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
    can_bus.send(angle_msg)
    can_bus.send(torque_msg)


class EdwardControl(Node):
    def __init__(self):
        super().__init__("edward_control")

        # declare and get parameters
        self.declare_parameter("use_can", False)
        self.USE_CAN = self.get_parameter("use_can").value

        # initialize CAN if requested
        if self.USE_CAN:
            self.get_logger().info("Using CAN!")
            dbc_file_path = os.path.join(
                get_package_share_directory("edward_control"),
                "config/full_bus.dbc"
            )
            db = cantools.database.load_file(dbc_file_path)

            # instantiate the CAN bus
            self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Create timer
        _timer = self.create_timer(1/FREQ, self.timer_callback)

        # Publishers
        self.joint_pub = self.create_publisher(JointState,"joint_states",10)

        # Subscribers
        _joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Services
        _csv_traj_srv = self.create_service(CSVTraj, "csv_traj", self.csv_callback)
        _set_joints_srv = self.create_service(SetJoints, "set_joints", self.set_joints_callback)
        _goto_srv = self.create_service(GoTo, "goto", self.goto_callback)
        _home_srv = self.create_service(Empty, "home", self.home_callback)

        # tf listener and buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # tf broadcaster
        self.broadcaster = TransformBroadcaster(self)

        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0] # commanded angles
        self.joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0] # commanded torques

        self.i = 0 # index for joint trajectories
        self.pose_recieved = False # flag for if a goal pose was recieved
        self.joint_traj = None # (N,5) ndarray of joint states for each point in trajectory
        self.Tse = None # EE in the S frame, from FK at each step

        # True when the trigger is pressed, thus enabling VR tracking
        self.enable_tracking = False

        # stores the zero position of the VR controller
        self.p0 = np.array([0.0, 0.0, 0.0])
        self.q0 = np.array([0.0, 0.0, 0.0, 1.0])


    def joy_callback(self, joy_msg):
        '''
        listens for button presses on the controller and acts accordingly
        '''
        # enable tracking of VR controller if trigger is pressed
        self.enable_tracking = True if joy_msg.axes[0] else False

        # zero the axes if B button is pressed
        self.zero = True if joy_msg.buttons[0] else False


    def home_callback(self, request, response):
        '''
        Brings the robot back to its home configuration
        '''

        #TODO: is this ok to do? Should a trajectory be made?

        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
        return response


    def set_joints_callback(self, request, response):
        '''
        Set joints to the requested positions
        '''

        self.joint_states = [
            request.joint1,
            request.joint2,
            request.joint3,
            request.joint4,
            request.joint5
        ]

        return response


    def goto_callback(self,request,response):
        '''
        This service probably does not need to be used.
        Since we are almost never planning a long distance, IK
        probably will reliably converge to each goal pose from the
        VR controller which should theoretically be very close to the
        previous goal pose.
        '''

        self.pose_recieved = True

        #  Get a transformation matrix for the goal pose in the space frame
        R = Rotation.from_euler(
            "xyz", [request.roll,request.pitch,request.yaw], degrees=True
        ).as_matrix()

        p = np.array([request.x,request.y,request.z])
        goal = mr.RpToTrans(R, p)

        # if Tse is defined, set it as the starting configuration
        # otherwise use M as starting configuration in reference trajectory
        start = self.Tse if self.Tse is not None else M

        total_time = 1 # TODO: change/parmaterize this
        N = int(total_time/(1/FREQ)) # TODO: what should this be?
        self.joint_traj = np.zeros((N,5))

        # generate a reference trajectory from start to goal over time total_time with N points
        ref_traj = mr.CartesianTrajectory(
            Xstart=start,
            Xend=goal,
            Tf=total_time,
            N=N,
            method=5
        )

        # for each point along trajectory, compute IK and store it in the joint_traj array
        for i in range(len(ref_traj)):
            res = mr.IKinSpace(Slist, M, ref_traj[i], self.joint_states, eomg=0.1, ev=0.1)
            self.joint_traj[i,:] = res[0]
            if res[1] is False:
                self.get_logger().warn("IK solver failed to converge")
                response.status = False
                self.get_logger().info(f"{i}, {type(i)}")
                self.joint_traj = self.joint_traj[:i,:] # remove subsequent rows
                break
        else:
            # only true if never broken (converged successfully)
            response.status = True

        # save joint trajectory to a csv file
        if LOG:
            np.savetxt("joint_traj_log.csv", self.joint_traj,delimiter=",", fmt="%.4f")

        return response


    def run_IK(self, T_vr, eomg=0.1, ev=0.1):
        '''
        Runs inverse kinematics from the current measured EE state
        self.Tse to the goal state T_vr and sets the joint_states
        equal to the output of inverse kinematics

        Args:
            T_vr (ndarray): 4x4 homogeneous transformation matrix
            of the VR controller in the base link frame
        '''

        # Extract rotation matrix and position vector from tf matrix
        R_mr, pvec = mr.TransToRp(T_vr)
        R = Rotation.from_matrix(R_mr).as_euler("xyz")

        # Solve IK and set the joint angles
        if self.enable_tracking:
            #  t0 = time.monotonic_ns()
            result = mr.IKinSpace(Slist, M, T_vr, self.joint_states, eomg=eomg, ev=ev)
            #  t_elapsed = time.monotonic_ns() - t0
            #  self.get_logger().info(f"{res[1]}, {round(t_elapsed*1e-9,4)} sec")
            if result[1]:
                self.joint_states = list(result[0])
                self.joint_states[1] = -self.joint_states[1] # TODO: why
            else:
                # TODO: try planning only position?
                self.get_logger().warn(f"IK failed to converge")


    def csv_callback(self, request, response):
        '''
        Gets trajectories of joint states from a CSV file.
        Useful for testing
        '''

        self.pose_recieved = True
        csv_path = request.csv_path
        self.joint_traj = np.loadtxt(csv_path,delimiter=",")
        return response


    def timer_callback(self):

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
                # TODO: probably just do this:
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

        #  update joint angles at each step if pose recieved from GoTo or CSVTraj services
        if self.pose_recieved:
            self.joint_states = [
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


        # construct and publish a JointState message
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ["joint1","joint2","joint3","joint4","joint5"]
        js_msg.position = self.joint_states
        self.joint_pub.publish(js_msg)

        # TODO: send joint states on CAN bus
        #  send_can_cmd(self.joint_states, self.joint_torques)

        # Get the current end effector pose
        # TODO: use sensed joint states, this assumes IK always achieves
        # exactly the requested EE pose 
        # self.Tse = read_joint_angles()
        self.Tse = mr.FKinSpace(M, Slist, self.joint_states)





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


