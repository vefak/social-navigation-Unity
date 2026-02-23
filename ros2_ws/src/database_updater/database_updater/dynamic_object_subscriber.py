import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Example message type, replace as needed
from geometry_msgs.msg import TransformStamped 
# Subscriber callback function
def object_callback(msg, topic_name):
    node.get_logger().info(f"Received message on topic {topic_name}: {msg.data}")

class DynamicObjectSubscriber(Node):
    def __init__(self):
        super().__init__('dynamic_object_subscriber')
        self.subscribers = []  # Keep track of subscribers

        # Call the method to get and subscribe to object topics
        self.subscribe_to_object_topics()

    def subscribe_to_object_topics(self):
        # Get the list of all active topics with their types
        topic_list = self.get_topic_names_and_types()

        for topic_name, topic_types in topic_list:
            # Check if "object" is in the topic name
            if "dynamic" in topic_name:
                # Assuming the message type is std_msgs/String for simplicity
                if 'geometry_msgs/msg/TransformStamped' in topic_types:
                    # Subscribe to the topic
                    subscriber = self.create_subscription(
                        String,
                        topic_name,
                        lambda msg, t=topic_name: object_callback(msg, t),
                        10
                    )
                    self.subscribers.append(subscriber)
                    self.get_logger().info(f"Subscribed to topic: {topic_name}")
                else:
                    self.get_logger().warn(f"Unsupported type for topic {topic_name}: {topic_types}")

def main(args=None):
    rclpy.init(args=args)

    # Create the node and run
    global node
    node = DynamicObjectSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
