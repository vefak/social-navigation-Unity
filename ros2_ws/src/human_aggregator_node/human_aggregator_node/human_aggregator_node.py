import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from unitycustommsg.msg import Human, HumanArray, TwistTransformUnity


class HumanAggregatorNode(Node):
    def __init__(self):
        super().__init__("human_aggregator")
        self.publisher_ = self.create_publisher(HumanArray, "/all_humans", 10)
        self.human_data = {}

        # Scan for human topics
        all_topics = self.get_topic_names_and_types()
        for name, types in all_topics:
            if (
                name.startswith("/dynamic_")
                and "unitycustommsg/msg/TwistTransformUnity" in types
            ):
                self.create_subscription(
                    TwistTransformUnity,
                    name,
                    lambda msg, t=name: self.human_callback(msg, t),
                    10,
                )

        self.create_timer(0.1, self.publish_human_array)

    def human_callback(self, msg, topic):
        self.human_data[topic] = msg

    def publish_human_array(self):
        array_msg = HumanArray()
        for topic, msg in self.human_data.items():
            human = Human()

            # Modify the transform
            fixed_transform = TransformStamped()
            fixed_transform.header = msg.transform.header
            fixed_transform.header.stamp = self.get_clock().now().to_msg()
            fixed_transform.child_frame_id = msg.transform.child_frame_id
            fixed_transform.transform.translation.x = (
                msg.transform.transform.translation.x + 36.0
            )
            fixed_transform.transform.translation.y = (
                msg.transform.transform.translation.y
            )
            fixed_transform.transform.translation.z = (
                msg.transform.transform.translation.z + 13.5
            )
            fixed_transform.transform.rotation = msg.transform.transform.rotation

            # Fill human message
            human.transform = fixed_transform
            human.twist = msg.twist
            human.id = topic
            array_msg.humans.append(human)

        self.publisher_.publish(array_msg)


def main():
    rclpy.init()
    rclpy.spin(HumanAggregatorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
