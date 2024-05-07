import time
from interfaces.action import MoveHand
from actuators.abstract_mycobot_driver import AbstractMyCobotDriver, MyCobotState
import threading

class MockMyCobotDriver(AbstractMyCobotDriver):

    

    def __init__(self):
        super().__init__("mock_mycobot_driver", mc= None)
        self.TASK_TIME_IN_SEC = 3
        self.rhand_lock = threading.Lock()

    def execute_handle_startspace(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_startspace(goal_handle)
            result = MoveHand.Result()
            result.finished = True
            goal_handle.succeed()
            time.sleep(self.TASK_TIME_IN_SEC)
            self.current_task = MyCobotState.IDLE
            return result

    def execute_handle_start_to_endspace(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_start_to_endspace(goal_handle)
            result = MoveHand.Result()
            result.finished = True
            goal_handle.succeed()
            time.sleep(self.TASK_TIME_IN_SEC)
            self.current_task = MyCobotState.IDLE
            return result

    
    def execute_handle_workspace(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_workspace(goal_handle)
            goal_handle.succeed()
            result = MoveHand.Result()
            result.finished = True
            time.sleep(self.TASK_TIME_IN_SEC)
            self.current_task = MyCobotState.IDLE
            return result
        

    def execute_handle_return(self, goal_handle):
        with self.rhand_lock:
            super().execute_handle_return(goal_handle)
            goal_handle.succeed()
            result = MoveHand.Result()
            result.finished = True
            time.sleep(self.TASK_TIME_IN_SEC)
            self.current_task = MyCobotState.IDLE
            return result
