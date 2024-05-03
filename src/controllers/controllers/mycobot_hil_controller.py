# Import packages
from time import sleep
import rclpy  # ROS 2 Python client library
from rclpy.node import Node
from std_msgs.msg import UInt8  # ROS 2 standard message type
from rclpy.action import ActionClient
from interfaces.action import MoveHand
from interfaces.msg import SpaceState
from std_msgs.msg import Bool

OFFLINE = "Offline"
STARTSPACE = "/spinningfactory/startspace_state"
WORKSPACE = "/spinningfactory/workspace_state"
HAND_SUFFIX = "_hand"
ERROR_TRESHOLD = 3
class MyCobotHiLController(Node):
    

    def __init__(self):
        super().__init__('mycobot_hil_controller')
        
        self.mycobot_states = {
            0 : "IDLE",
            1 : "HANDLE STARTSPACE",
            2 : "HANDLE WORKSPACE",
            3 : "BACKING"
        }

        self.space_states = {
            0: "EMPTY",
            1: "BLUE PLACED",
            2: "RED PLACED",
            3: "BLUE AND RED PLACED",
            4: "ERROR"
        }
    
        self.hand_states = {
            False: "NO HUMAN",
            True: "HUMAN INTERACTION"
        }

        # Controlled object trackers
        self.startspace_control = {
            "state" : self.space_states[0],
            "cnt" : 0,
            "error_treshold": 0,
            "hil_state" : self.hand_states[False],
        }

        self.workspace_control = {
            "state" : self.space_states[0],
            "cnt" : 0,
            "error_treshold": 0,
            "hil_state" : self.hand_states[False],
        }

        self.mycobot_control = {
            "state" : self.mycobot_states[0],
            "cnt" : 0,
            "error_treshold" : 0
        }

        self.rhand_subscription = self.create_subscription(
            UInt8,
            '/spinningfactory/mycobotstate',
            self.robotstate_on_receive,
            10
        )

        self.startspace_subscription = self.create_subscription(
            SpaceState,
            STARTSPACE,
            self.startspace_on_receive,
            10
        )

        self.workspace_subscription = self.create_subscription(
            SpaceState,
            WORKSPACE,
            self.workspace_on_receive,
            10
        )

        self.startspace_hand_subscription = self.create_subscription(
            Bool,
            STARTSPACE + HAND_SUFFIX,
            self.startspace_hand_on_receive,
            10
        )

        self.workspace_hand_subscription = self.create_subscription(
            Bool,
            WORKSPACE + HAND_SUFFIX,
            self.workspace_hand_on_receive,
            10
        )

        # Actions
        self.startspace_client = ActionClient(self, MoveHand, 'handleStartSpace')
        self.workspace_client = ActionClient(self, MoveHand, 'handleWorkSpace')
        self.backoff_client = ActionClient(self, MoveHand, 'handleReturnToInit')

    def robotstate_on_receive(self, msg):
        if self.mycobot_control.state == self.mycobot_states[msg.data]:
            self.mycobot_control.cnt += 1
        else:
            self.mycobot_control.error_treshold += 1
        
        if self.mycobot_control.error_treshold >= ERROR_TRESHOLD:
            self.mycobot_control.error_treshold = 0
            self.mycobot_control.cnt = 1
            self.mycobot_control.state = self.mycobot_states[msg.data]

    def startspace_on_receive(self, msg):
        if self.startspace_control.state == self.space_states[msg.state]\
        and self.mycobot_control.state != self.mycobot_states[1]:
            self.startspace_control.cnt += 1
        else:
            self.startspace_control.error_treshold += 1
        
        if self.startspace_control.error_treshold >= ERROR_TRESHOLD:
            self.startspace_control.error_treshold = 0
            self.startspace_control.state = self.space_states[msg.state]
            self.startspace_control.cnt = 1


    def workspace_on_receive(self, msg):
        if self.workspace_control.state == self.space_states[msg.state]\
            and self.mycobot_control.state != self.mycobot_states[2]:
            self.workspace_control.cnt += 1
        else:
            self.workspace_control.error_treshold += 1
        
        if self.workspace_control.error_treshold >= ERROR_TRESHOLD:
            self.workspace_control.error_treshold = 0
            self.workspace_control.state = self.space_states[msg.state]
            self.workspace_control.cnt = 1
       

    def startspace_hand_on_receive(self, msg):
        self.startspace_control.hil_state = msg.data

    def workspace_hand_on_receive(self, msg):
        self.workspace_control.hil_state = msg.data    

    def trigger_startspace_action(self):
        self.startspace_client.wait_for_server()
        goal_msg = MoveHand.Goal()
        # Send goal to handleWorkSpace action server
        future = self.startspace_client.send_goal_async(goal_msg)
    
    def trigger_workspace_action(self):
        self.workspace_client.wait_for_server()
        goal_msg = MoveHand.Goal()
        # Send goal to handleWorkSpace action server
        future = self.workspace_client.send_goal_async(goal_msg)
    
    def trigger_backoff_action(self):
        self.backoff_client.wait_for_server()
        goal_msg = MoveHand.Goal()
        # Send goal to handleWorkSpace action server
        future = self.backoff_client.send_goal_async(goal_msg)
        
    def actuate_mycobot(self):
        if self.startspace_control.state == self.space_states[2]\
        and self.startspace_control.cnt >= 3\
        and not self.startspace_control.hil_state:
            self.trigger_startspace_action()
        if self.workspace_control.state == self.space_states[1]\
        and self.workspace_control.cnt >= 3\
        and not self.workspace_control.hil_state:
            self.trigger_workspace_action()
        
        # Red on startspace & No hand -> move to W
        # Green on workspace & No hand -> move to F
        # Green on startspace & No hand -> move to F
        # Red on workspace -> HIL 
    





# Resources: 1. robot 2. human
# if robot available -> use robot
# if robot not available -> try human
# if none available -> wait until one gets available


### if robot task is scheduled and human does it -> remove that task.
#   workspace green disappears
#   startspace red/green disappears but not because of a task