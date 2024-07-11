#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Float32MultiArray

class SimpleTransformNode(Node):
    def __init__(self):
        super().__init__('simple_transform_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.xy_pub = self.create_publisher(Float32MultiArray, 'xy_values', 10)

        # Subscribe to tf topic
        self.create_subscription(
            TransformStamped,
            '/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        try:
            # Extract the position from the transform message
            pose = msg.transform.translation

            # Apply the given transformations
            x = pose.x + 17.0038069
            y = 7.617856 - pose.y

            # Publish the results
            self.publish_xy(x, y)
            self.get_logger().info(f'Published X: {x}, Y: {y}')
        except Exception as e:
            self.get_logger().error(f"Error in TF callback: {str(e)}")

    def publish_xy(self, x, y):
        msg = Float32MultiArray()
        msg.data = [x, y]
        self.xy_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

