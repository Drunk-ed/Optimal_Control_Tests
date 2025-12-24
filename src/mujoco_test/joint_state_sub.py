import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.sub = self.create_subscription(
            String,
            '/string/msgs',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f"Joint states: {msg.data}")

def main():
    rclpy.init()
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
