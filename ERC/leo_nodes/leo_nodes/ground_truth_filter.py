import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GroundTruthFilter(Node):

    def __init__(self, x, y):
        super().__init__('ground_truth_filter')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal_pose)
        self.x = x
        self.y = y

    def publish_goal_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x - 17.0038069
        pose.pose.position.y = 7.617856 - self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.publisher_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print("Usage: ros2 run leo_nodes ground_truth_filter <x_value> <y_value>")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    ground_truth_filter = GroundTruthFilter(x, y)

    try:
        rclpy.spin(ground_truth_filter)
    except KeyboardInterrupt:
        pass
    finally:
        ground_truth_filter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
