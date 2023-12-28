import rclpy
import rclpy.node
import std_msgs.msg


class ExampleNode(rclpy.node.Node):
    """ An example node that contains a bit of everything. """

    def __init__(self):
        super().__init__('example_node')

        # Parameters
        self.declare_parameter('message_content', 'Hello World!')
        self.declare_parameter('publish_count', 1)
        self.declare_parameter('publish_interval', 1.0)

        # Publisher
        self.publisher = self.create_publisher(std_msgs.msg.String, 'example_pub_topic', 10)

        # Subscriber
        self.subscription = self.create_subscription(std_msgs.msg.String, 'example_sub_topic',
                                                     self.subscriber_callback, 10)

        # Timer
        self.execution_count = 0
        self.timer = self.create_timer(
            self.get_parameter('publish_interval').get_parameter_value().double_value,
            self.timer_callback)

    def subscriber_callback(self, msg):
        """ Print the received message. """
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        """ Publish a new message. """
        self.execution_count += 1

        message_content = self.get_parameter('message_content').get_parameter_value().string_value
        msg = std_msgs.msg.String()
        msg.data = message_content
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        if self.execution_count == self.get_parameter(
                'publish_count').get_parameter_value().integer_value:
            raise SystemExit


def main(args=None):
    """ Run the node. """
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('Quitting').info('Done')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
