from configuration import STARTSPACE_TOPIC
import rclpy
from rclpy.node import Node
from interfaces.msg import SpaceState
import random

class StartSpaceCameraMock(Node):
    def __init__(self):
        super().__init__("startspace_camera")
        self.current_state = SpaceState.EMPTY
        self.cmd_vel_pub_ = self.create_publisher(SpaceState, STARTSPACE_TOPIC, 10) #Give type and topic name and queue size
        self.timer = self.create_timer(0.5, self.send_startspace_state)
        self.state_changer = self.create_timer(2, self.change_state_randomly)
        self.get_logger().info("StartSpace camera mock started transmitting")
        

    def change_state_randomly(self):
        states = [SpaceState.EMPTY, SpaceState.ITEMPLACED, SpaceState.ERROR]
        self.current_state = random.choice(states)

    def send_startspace_state(self):
        msg = SpaceState()
        msg.state = self.current_state

        self.cmd_vel_pub_.publish(msg)


def main(args = None):
    rclpy.init(args=args)

    camera_node = StartSpaceCameraMock()
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__=="__main__":
    main()