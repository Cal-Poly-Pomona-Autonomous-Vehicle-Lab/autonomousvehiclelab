#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage  # useful if you ever forward tf

class GenericRelayNode(Node):
    def __init__(self):
        super().__init__('generic_relay_node')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('message_type', '')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        message_type = self.get_parameter('message_type').get_parameter_value().string_value

        if not input_topic or not output_topic or not message_type:
            self.get_logger().error("You must set 'input_topic', 'output_topic', and 'message_type'")
            return

        msg_class = get_message(message_type)

        self.publisher = self.create_publisher(msg_class, output_topic, 10)
        self.subscription = self.create_subscription(msg_class, input_topic, self.relay_callback, 10)

        self.get_logger().info(f"Relaying [{message_type}] from '{input_topic}' → '{output_topic}'")

    def relay_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GenericRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
