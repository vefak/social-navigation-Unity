# json_publisher.py
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class JsonPublisher(Node):
    def __init__(self):
        super().__init__("json_publisher")
        self.declare_parameter("file_path", "path_eval_results.json")
        self.declare_parameter("topic", "/path_eval_results")
        self.declare_parameter("publish_rate_hz", 1.0)  # set 0 to publish once
        self.declare_parameter("reload_on_change", True)  # reload when file changes

        file_path = self.get_parameter("file_path").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.reload_on_change = bool(self.get_parameter("reload_on_change").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # “latched” behavior
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(String, topic, qos)

        self.file_path = file_path
        self.last_mtime = 0
        self.cached_payload = None

        # publish once immediately
        self._load_and_publish()

        if rate > 0:
            self.timer = self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        self._load_and_publish()

    def _load_and_publish(self):
        try:
            mtime = os.path.getmtime(self.file_path)
            if (self.cached_payload is None) or (
                self.reload_on_change and mtime != self.last_mtime
            ):
                with open(self.file_path, "r") as f:
                    data = json.load(f)  # validates JSON
                self.cached_payload = json.dumps(data, separators=(",", ":"))
                self.last_mtime = mtime

            msg = String(data=self.cached_payload)
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish JSON: {e}")


def main():
    rclpy.init()
    node = JsonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
