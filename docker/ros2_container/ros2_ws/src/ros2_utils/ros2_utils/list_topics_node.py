import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class minimalNode(Node):
    
    def list_topics(self):
        self.get_logger().info(' Sourcing topics...')
        topic_list = self.get_topic_names_and_types()
        for info in topic_list:
            self.get_logger().info(f'topic: {info[0]}')

    def __init__(self):
        super().__init__('list_topics_node')
        self.create_timer(5.0, self.list_topics)

def main(args=None):
    rclpy.init(args=args)

    min_node = minimalNode()

    # Run the node continously
    try:
        rclpy.spin(min_node)
    except KeyboardInterrupt:
        print('Shutting down minimal_node...')
        min_node.destroy_node()
        rclpy.shutdown()
        return

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    min_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
