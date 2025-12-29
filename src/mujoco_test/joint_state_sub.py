import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/joint_states/vel',
            self.callback,
            10
        )

    def callback(self, msg):
        joint_pos = list(msg.data)
        self.get_logger().info(f"{msg}")


def main():
    rclpy.init()
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
