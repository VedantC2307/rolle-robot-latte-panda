#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Import your agent and message types
from rolle_langchain_agent.single_shot_agent import UnifiedNavAgent
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped

def euler_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw (in radians) about Z into a quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )

class TestNavAgent(Node):
    def __init__(self):
        super().__init__('test_nav_agent')

        # Publishers to drive the /odom and /voice_command topics
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_pub = self.create_publisher(String, '/voice_command', 10)

        # Subscribe to the agent’s goal output
        self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Launch your real agent in the same process
        self.agent_node = UnifiedNavAgent()

        # Timer to fire once and send our test messages
        self.test_timer = self.create_timer(1.0, self.publish_test)

    def publish_test(self):
        # --- publish fake odometry ---
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = 50.0
        odom.pose.pose.position.y = 122.3
        # zero yaw
        odom.pose.pose.orientation = euler_to_quaternion(0.0)
        self.odom_pub.publish(odom)

        # --- publish fake voice command ---
        cmd = String()
        # Test different types of commands to ensure robustness
        # cmd.data = 'Take me to the kitchen'  # Navigation command
        # cmd.data = 'Move forward 2 meters'   # Movement command
        # cmd.data = 'Go to the bedroom'       # Navigation command
        cmd.data = 'How are you doing today?' # Conversational command (the one that failed)
        self.cmd_pub.publish(cmd)

        self.get_logger().info('▶️  Published test /odom & /voice_command')

        # stop firing again
        self.destroy_timer(self.test_timer)

    def goal_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'✅  Agent published goal: x={x:.2f}, y={y:.2f}')
        # Clean shutdown
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    tester = TestNavAgent()
    # Set up a MultiThreadedExecutor so both nodes can spin
    executor = MultiThreadedExecutor()
    executor.add_node(tester)
    executor.add_node(tester.agent_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tester.agent_node.destroy_node()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
