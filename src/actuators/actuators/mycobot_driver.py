from interfaces.action import MoveHand
from actuators.abstract_mycobot_driver import AbstractMyCobotDriver, MyCobotState
import threading

class MyCobotDriver(AbstractMyCobotDriver):

    

    def __init__(self, mc):
        super().__init__("mycobot_driver", mc=mc)
        self.rhand_lock = threading.Lock()

    def execute_handle_startspace(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_startspace(goal_handle)
            result = MoveHand.Result()
            # Grab the new item
            self.mc.sync_send_coords([195.9, -180.9, 195.8, -170, -5.72, -110.19], 20, 0)
            self.mc.sync_send_coords([206.9, -200.3, 70.6, -170, -2.31, -112.41], 20, 0)
            self.mc.set_gripper_state(1, 30)

            # Move up
            self.mc.sync_send_coords([184.9, 8.1, 298.3, -162.45, 12.85, -93.93], 20, 0)
            
            # Move to workspace
            self.mc.sync_send_coords([212.1, 197.2, 84.4, -177.05, 2.11, -19.57], 20, 0)
            self.mc.set_gripper_state(0, 30)

            # Return
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 20)
            

            result.finished = True
            goal_handle.succeed()
            self.current_task = MyCobotState.IDLE
            return result

    def execute_handle_workspace(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_workspace(goal_handle)
            goal_handle.succeed()
            result = MoveHand.Result()
            # Movements here!
            result.finished = True
            self.current_task = MyCobotState.IDLE
            return result
        

    def execute_handle_return(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_return(goal_handle)
            goal_handle.succeed()
            result = MoveHand.Result()
            # Movements here!
            result.finished = True
            self.current_task = MyCobotState.IDLE
            return result
