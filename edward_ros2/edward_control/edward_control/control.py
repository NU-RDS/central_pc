import numpy as np
import modern_robotics as mr
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from edward_interfaces.srv import GoTo, SetJoints, CSVTraj
from std_srvs.srv import Empty

from ament_index_python.packages import get_package_share_directory
import can
import cantools
import os

from .params import Slist, Blist, M, total_length


LOG = False     # TODO: make this a parameter to the node
FREQ: int = 100 # TODO: make this a parameter to the node

# load the dbc file
dbc_file_path = os.path.join(
    get_package_share_directory("edward_control"),
    "config/full_bus.dbc"
)
db = cantools.database.load_file(dbc_file_path)

# instantiate the CAN bus
can_bus = can.interface.Bus(channel='can0', bustype='socketcan')


def send_angle_cmd(joint_states):
    '''
    sends commanded joint states over CAN
    '''
    #TODO TEST 

    main_angle_cmd_msg = db.get_message_by_name("Main_Angle_Command")

    main_angle_cmd_data = main_angle_cmd_msg.encode({
        "Angle_Command_0": joint_states[0],
        "Angle_Command_1": joint_states[1],
        "Angle_Command_2": joint_states[2],
        "Angle_Command_3": joint_states[3],
        "Angle_Command_4": joint_states[4]
    })

    msg = can.Message(arbitration_id=0x600, data=main_angle_cmd_data)
    can_bus.send(msg)


def angles_reached(goal_state) -> bool:
    '''
    reads the measured angles and returns
    a boolean if the 
    '''
    #TODO: TEST
    msg = can_bus.recv()
    decoded_msg = db.decode_message(msg.arbitration_id, msg.data)

    #  if decoded_msg.values == goal_state: # within threshold
        #  return True
    #  else:
        #  return False


class EdwardControl(Node):
    def __init__(self):
        super().__init__("edward_control")
    
        self.declare_parameter("use_jsp_gui", False)
        self.USE_JSP_GUI = self.get_parameter("use_jsp_gui").value

        # Create timer
        _timer = self.create_timer(1/FREQ, self.timer_callback)

        # Publishers
        self.joint_pub = self.create_publisher(JointState,"joint_states",10)

        # Subscribers
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.js_sub_cbk, 10)

        # Services
        _csv_traj_srv = self.create_service(CSVTraj, "csv_traj", self.csv_callback)
        _set_joints_srv = self.create_service(SetJoints, "set_joints", self.set_joints_callback)
        _goto_srv = self.create_service(GoTo, "goto", self.goto_callback)
        _home_srv = self.create_service(Empty, "home", self.home_callback)
        _reset_srv = self.create_service(Empty, "reset", self.reset_callback)


        self.i = 0 # index for joint trajectories

        self.pose_recieved = False # flag for if a goal pose was recieved
        self.joint_traj = None # (N,5) ndarray of joint states for each point in trajectory

        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0] # current joint states

        self.Tse = None # EE in the S frame, from FK at each step


    def js_sub_cbk(self, js_msg):
        '''
        this is just for testing so you can use JSP gui
        and get the joint states from that.
        '''

        if self.USE_JSP_GUI:
            self.joint_states = list(js_msg.position)


    def home_callback(self, request, response):
        '''
        Brings the robot back to its home configuration
        '''

        #TODO: this should be a trajectory of joint states
        # computed from IK, not just setting all joints to 0
        # maybe just call the goto service

        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
        return response


    def reset_callback(self, request, response):
        '''
        Manually sets all the joints to 0 immediately
        '''
        # TODO: Don't do this 
        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
        return response


    def set_joints_callback(self, request, response):
        '''
        Set joints to the requested positions 
        '''
        #TODO: the simulation in RVIZ should have this happen over some time,
        # not instantaneously

        self.joint_states = [
            request.joint1,
            request.joint2,
            request.joint3,
            request.joint4,
            request.joint5
        ]
        
        #  # Command joints to move to these anlges over CAN
        #  send_angle_cmd(self.joint_states)
        #
        #  # wait until the angle is reached before proceeding
        #  while not angles_reached(self.joint_states):
            #  pass # do nothing until reached
        #  else:
            #  self.get_logger().info(f"{self.joint_states} reached")


        return response


    def goto_callback(self,request,response):
        '''
        generates joint trajectories needed to get to
        the requested 3D pose.
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

        # update joint angles at each step if pose recieved
        if not self.USE_JSP_GUI:
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

        # Get the end effector coordinates based on FK
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


