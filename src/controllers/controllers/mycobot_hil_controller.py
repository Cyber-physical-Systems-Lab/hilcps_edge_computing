# Import packages
from time import sleep
import rclpy  # ROS 2 Python client library
from rclpy.node import Node
from std_msgs.msg import UInt8  # ROS 2 standard message type
from rclpy.action import ActionClient
from interfaces.action import MoveHand
from interfaces.msg import SpaceState
from std_msgs.msg import Bool
import json


OFFLINE = "Offline"
STARTSPACE = "/spinningfactory/startspace_state"
WORKSPACE = "/spinningfactory/workspace_state"
HAND_SUFFIX = "_hand"
UPPER_TRESHOLD = 5
class MyCobotHiLController(Node):
    

    def __init__(self):
        super().__init__('mycobot_hil_controller')
        self.timer_ = self.create_timer(0.5, self.log_states)
        self.mycobot_states = {
             0 : "IDLE",
            1 : "HANDLE STARTSPACE",
            2 : "HANDLE START TO END",
            3 : "HANDLE WORKSPACE",
            4 : "BACKING"
        }

        self.hil_states = {
            0 : "IDLE",
            1 : "HANDLE STARTSPACE",
            2 : "HANDLE START TO END",
            3 : "HANDLE WORKSPACE",
            4 : "ASSEMBLE"
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
            "cnt" : UPPER_TRESHOLD,
            "hil_state" : self.hand_states[False],
        }

        self.workspace_control = {
            "state" : self.space_states[0],
            "cnt" : UPPER_TRESHOLD,
            "hil_state" : self.hand_states[False],
        }

        self.actuator_control = {
            "mycobot" : self.mycobot_states[0],
        }

        self.rhand_subscription = self.create_subscription(
            UInt8,
            '/spinningfactory/mycobotstate',
            self.robotstate_on_receive,
            10
        )

        self.hil_subscription = self.create_subscription(
            UInt8,
            '/spinningfactory/hilstate',
            self.hilstate_on_receive,
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

    def log_states(self):
        startspace_control_str = json.dumps(self.startspace_control)
        workspace_control_str = json.dumps(self.workspace_control)
        rhand_control_str = json.dumps(self.actuator_control)

        self.get_logger().info("startspace: " + startspace_control_str)
        #self.get_logger().info("workspace: " + workspace_control_str)
        #self.get_logger().info("rhand: " + rhand_control_str)


    def startspace_on_receive(self, msg):
        # if robot or human interaction is happening at the space, don't update
        if self.actuator_control["mycobot"] != self.mycobot_states[1]\
        or self.startspace_control["hil_state"]:
            return
        
        # increase until capped
        if self.startspace_control["state"] == self.space_states[msg.state]\
         and self.startspace_control["cnt"] < UPPER_TRESHOLD:
            self.startspace_control["cnt"] += 1
            return
        
        # decrease or change state
        if self.startspace_control["state"] != self.space_states[msg.state]:
            self.startspace_control["cnt"] -=1
            if self.startspace_control["cnt"] < 0:
                self.startspace_control["cnt"] = UPPER_TRESHOLD
                self.startspace_control["state"] = self.space_states[msg.state]
            
    def workspace_on_receive(self, msg):
        # if robot or human interaction is happening at the space, don't update
        if self.actuator_control["mycobot"] != self.mycobot_states[1]\
        or self.workspace_control["hil_state"]:
            return
        
        # increase until capped
        if self.workspace_control["state"] == self.space_states[msg.state]\
         and self.workspace_control["cnt"] < UPPER_TRESHOLD:
            self.workspace_control["cnt"] += 1
            return
        
        # decrease or change state
        if self.workspace_control["state"] != self.space_states[msg.state]:
            self.workspace_control["cnt"] -=1
            if self.workspace_control["cnt"] < 0:
                self.workspace_control["cnt"] = UPPER_TRESHOLD
                self.workspace_control["state"] = self.space_states[msg.state]
        



    """def workspace_on_receive(self, msg):
        if self.workspace_control["state"] == self.space_states[msg.state]\
            and self.mycobot_control["state"] != self.mycobot_states[2]:
            self.workspace_control["cnt"] += 1
        else:
            self.workspace_control["error_treshold"] += 1
        
        if self.workspace_control["error_treshold"] >= UPPER_TRESHOLD:
            self.workspace_control["error_treshold"] = 0
            self.workspace_control["state"] = self.space_states[msg.state]
            self.workspace_control["cnt"] = 1
    """   

    def robotstate_on_receive(self, msg):
        self.actuator_control["mycobot"] = self.mycobot_states[msg.data]

    def hilstate_on_receive(self, msg):
        self.actuator_control["hil"] = self.hil_states[msg.data]


    def startspace_hand_on_receive(self, msg):
        self.startspace_control["hil_state"] = msg.data

    def workspace_hand_on_receive(self, msg):
        self.workspace_control["hil_state"] = msg.data    

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
        
    
    # kettőből ha a robot elérhető: robot exec
    # ha a robot nem elérhető de az ember igen ÉS a taszk nem ugyanaz: ember exec
    # ha egyik sem elérhető várj
    def actuate(self):
        if self.startspace_control["state"] == self.space_states[2]\
        and self.startspace_control["cnt"] >= 3\
        and not self.startspace_control["hil_state"]:
            self.trigger_startspace_action()
        if self.workspace_control["state"] == self.space_states[1]\
        and self.workspace_control["cnt"] >= 3\
        and not self.workspace_control["hil_state"]:
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