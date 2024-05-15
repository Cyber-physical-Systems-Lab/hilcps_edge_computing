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
import time
import os


OFFLINE = "Offline"
STARTSPACE = "/spinningfactory/startspace_state"
WORKSPACE = "/spinningfactory/workspace_state"
HAND_SUFFIX = "_hand"
UPPER_TRESHOLD = 5
class MyCobotHiLController(Node):
    

    def __init__(self):
        super().__init__('mycobot_hil_controller')
        self.STARTSPACE_UID = "-1"
        self.WORKSPACE_UID = "-1"
        self.timer_ = self.create_timer(0.5, self.actuate)
        self.mycobot_driver_states = {
            0 : "IDLE",
            1 : "HANDLE STARTSPACE",
            2 : "HANDLE START TO END",
            3 : "HANDLE WORKSPACE",
            4 : "BACKING"
        }

        self.hil_driver_states = {
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
    
        self.human_interaction_states = {
            False: "NO HUMAN",
            True: "HUMAN INTERACTION"
        }

        # Controlled object trackers
        self.startspace_control = {
            "state" : self.space_states[0],
            "cnt" : UPPER_TRESHOLD,
            "hil_state" : self.human_interaction_states[False],
        }

        self.workspace_control = {
            "state" : self.space_states[0],
            "cnt" : UPPER_TRESHOLD,
            "hil_state" : self.human_interaction_states[False],
        }

        self.actuator_control = {
            "mycobot" : self.mycobot_driver_states[4],
            "human" : self.hil_driver_states[0]
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
            1
        )

        self.workspace_subscription = self.create_subscription(
            SpaceState,
            WORKSPACE,
            self.workspace_on_receive,
            1
        )

        self.startspace_hand_subscription = self.create_subscription(
            Bool,
            STARTSPACE + HAND_SUFFIX,
            self.startspace_hand_on_receive,
            1
        )

        self.workspace_hand_subscription = self.create_subscription(
            Bool,
            WORKSPACE + HAND_SUFFIX,
            self.workspace_hand_on_receive,
            1
        )

        # Actions
        self.startspace_action = {
            "mycobot" : ActionClient(self, MoveHand, 'handleStartSpace'),
            "human" : ActionClient(self, MoveHand, 'handleStartSpaceHiL')
        }

        self.start_to_end_action = {
            "mycobot" : ActionClient(self, MoveHand, 'handleStartToEndSpace'),
            "human" : ActionClient(self, MoveHand, 'handleStartToEndSpaceHiL')
        }

        self.workspace_action = {
            "mycobot" : ActionClient(self, MoveHand, 'handleWorkSpace'),
            "human" : ActionClient(self, MoveHand, 'handleWorkSpaceHiL')
        }

        # Specific to arm / human
        self.mycobot_backoff_action = ActionClient(self, MoveHand, 'handleReturnToInit')
        self.hil_assemble_action = ActionClient(self, MoveHand, 'handleAssembleHiL')


    """def log_states(self):
        startspace_control_str = json.dumps(self.startspace_control)
        workspace_control_str = json.dumps(self.workspace_control)
        rhand_control_str = json.dumps(self.actuator_control)

        # self.get_logger().info("startspace: " + startspace_control_str)
        #self.get_logger().info("workspace: " + workspace_control_str)
        #self.get_logger().info("rhand: " + rhand_control_str)
    """

    def startspace_on_receive(self, msg):
        #self.get_logger().info(f"STARTSPACE_START\tPID {os.getpid() }\tUID {msg.unique_id}\tTIMESTAMP {time.time_ns()}")
        #self.STARTSPACE_UID = msg.unique_id
        # if robot or human interaction is happening at the space, don't update
        if self.actuator_control["mycobot"] == self.mycobot_driver_states[1]\
        or self.startspace_control["hil_state"]:
            pass
            #return
        
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
        self.get_logger().info(f"WORKSPACE_REC_START\tPID {os.getpid() }\tUID {msg.unique_id}\tTIMESTAMP {time.time_ns()}")
        # if robot or human interaction is happening at the space, don't update
        if self.actuator_control["mycobot"] == self.mycobot_driver_states[1]\
        or self.workspace_control["hil_state"]:
            self.get_logger().info(f"WORKSPACE_REC_END\tPID {os.getpid() }\tUID {msg.unique_id}\tTIMESTAMP {time.time_ns()}")
            return
        
        # increase until capped
        if self.workspace_control["state"] == self.space_states[msg.state]\
         and self.workspace_control["cnt"] < UPPER_TRESHOLD:
            self.workspace_control["cnt"] += 1

        
        # decrease or change state
        if self.workspace_control["state"] != self.space_states[msg.state]:
            self.workspace_control["cnt"] -=1
            if self.workspace_control["cnt"] < 0:
                self.workspace_control["cnt"] = UPPER_TRESHOLD
                self.workspace_control["state"] = self.space_states[msg.state]
        
        if self.workspace_control["cnt"] == UPPER_TRESHOLD:
            self.WORKSPACE_UID = msg.unique_id
        self.get_logger().info(f"WORKSPACE_REC_END\tPID {os.getpid() }\tUID {msg.unique_id}\tTIMESTAMP {time.time_ns()}")
        


    def robotstate_on_receive(self, msg):
        self.actuator_control["mycobot"] = self.mycobot_driver_states[4]#msg.data]

    def hilstate_on_receive(self, msg):
        self.actuator_control["human"] = self.hil_driver_states[msg.data]


    def startspace_hand_on_receive(self, msg):
        #uid = msg.unique_id
        #self.get_logger().info(f"STARTSPACEHAND\tPID {os.getpid() }\tTUID {uid}\tIMESTAMP {time.time_ns()}")
        self.startspace_control["hil_state"] = msg.data

    def workspace_hand_on_receive(self, msg):
        #uid = msg.unique_id
        #self.get_logger().info(f"WORKSPACEHAND\tPID {os.getpid() }\tUID {uid}\tTIMESTAMP {time.time_ns()}")
        self.workspace_control["hil_state"] = msg.data    

    def _select_actuator(self):
        # Prefer mycobot over human in any case

        if self.actuator_control["mycobot"] == self.mycobot_driver_states[0]:
            return "mycobot"
        elif self.actuator_control["human"] == self.hil_driver_states[0]:
            return "human"
        else:
            return None
        
    def trigger_movement_action(self, movement):
        actuator = self._select_actuator()
        # Wait if there is no actuator!
        if not actuator:
            return
        
        action_client = movement[actuator]
        goal_msg = MoveHand.Goal()
        goal_msg.unique_id = self.WORKSPACE_UID
        future = action_client.send_goal_async(goal_msg)
    
    def trigger_hil_assemble(self):
        # don't request if human is not available
        if self.actuator_control["human"] != self.hil_driver_states[0]:
            return
        goal_msg = MoveHand.Goal()
        goal_msg.unique_id = self.WORKSPACE_UID
        future = self.hil_assemble_action.send_goal_async(goal_msg)
    
    def trigger_mycobot_backoff(self):
        if self.actuator_control["mycobot"] == self.mycobot_driver_states[0]:
            return
        goal_msg = MoveHand.Goal()
        future = self.mycobot_backoff_action.send_goal_async(goal_msg)
    
    def _is_blue_on_space(self, space_control):
        return space_control["state"] == self.space_states[1]\
            or space_control["state"] == self.space_states[3]
    
    def _is_red_on_space(self, space_control):
        return space_control["state"] == self.space_states[2]\
            or space_control["state"] == self.space_states[3]

    def actuate(self):
        # precedence: blue items move first to flush assembled items as fast as possible

        # ha kék vagy kékpiros Start -> actuate startend
        self.get_logger().info(f"ACTUATE_START\tPID {os.getpid() }\tUID {self.WORKSPACE_UID}\tTIMESTAMP {time.time_ns()}")
        if self._is_blue_on_space(self.startspace_control):
            self.trigger_movement_action(self.start_to_end_action)
            self.startspace_control["cnt"] = 1
        
        # ha kék work -> actuate work
        if self._is_blue_on_space(self.workspace_control):
            self.trigger_movement_action(self.workspace_action)
            self.workspace_control["cnt"] = 1
        
        # ha piros vagy kékpiros Start -> actuate start
        if self._is_red_on_space(self.startspace_control):
            self.trigger_movement_action(self.startspace_action)
            self.startspace_control["cnt"] = 1

        # ha piros work -> actuate human
        if self._is_red_on_space(self.workspace_control):
            self.trigger_hil_assemble()
            self.workspace_control["cnt"] = 1
        self.get_logger().info(f"ACTUATE_END\tPID {os.getpid() }\tUID {self.WORKSPACE_UID}\tTIMESTAMP {time.time_ns()}")
        
