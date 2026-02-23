import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Path

class PathEvaluationNode(Node):
    def __init__(self):
        super().__init__('path_evaluation_node')

        # Subscribers
        self.create_subscription(Pose, '/robot_position', self.robot_callback, 10)
        self.create_subscription(PoseArray, '/dynamic_obstacles', self.dynamic_obstacles_callback, 10)
        self.create_subscription(PoseArray, '/static_obstacles', self.static_obstacles_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)

        # Metrics variables
        self.robot_position = None
        #self.dynamic_obstacles = []
        #self.static_obstacles = []
        self.path = None
        self.path_start_time = None

    def robot_callback(self, msg):
        self.robot_position = msg

    #def dynamic_obstacles_callback(self, msg):
    #    self.dynamic_obstacles = msg.poses

    #def static_obstacles_callback(self, msg):
    #    self.static_obstacles = msg.poses

    def path_callback(self, msg):
        self.path = msg.poses
        self.path_start_time = time.time()
        self.evaluate_path()

    def evaluate_path(self):
        if self.path is None or len(self.path) == 0:
            self.get_logger().warn('No path received to evaluate.')
            return

        # Metric calculations
        completion_time = time.time() - self.path_start_time
        path_length = self.calculate_path_length()
        path_efficiency = self.calculate_path_efficiency()
        min_static_dist, min_dynamic_dist = self.calculate_minimum_distances()
        personal_space_violations = self.calculate_personal_space_violations()

        # Log metrics
        self.get_logger().info(f'Completion Time: {completion_time:.2f} s')
        self.get_logger().info(f'Path Length: {path_length:.2f} m')
        self.get_logger().info(f'Path Length Efficiency: {path_efficiency:.2f}')
        #self.get_logger().info(f'Minimum Distance to Static Obstacles: {min_static_dist:.2f} m')
        #self.get_logger().info(f'Minimum Distance to Dynamic Obstacles: {min_dynamic_dist:.2f} m')
        self.get_logger().info(f'Personal Space Violations: {personal_space_violations}')

    def calculate_path_length(self):
        length = 0.0
        for i in range(1, len(self.path)):
            prev = self.path[i - 1].pose.position
            curr = self.path[i].pose.position
            length += math.sqrt((curr.x - prev.x)**2 + (curr.y - prev.y)**2)
        return length

    def calculate_path_efficiency(self):
        if len(self.path) < 2:
            return 0.0
        start = self.path[0].pose.position
        end = self.path[-1].pose.position
        straight_line_distance = math.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)
        return straight_line_distance / self.calculate_path_length()

    # def calculate_minimum_distances(self):
    #     min_static_dist = float('inf')
    #     min_dynamic_dist = float('inf')

    #     for pose in self.path:
    #         curr = pose.pose.position

    #         # Static obstacles
    #         for obs in self.static_obstacles:
    #             obs_pos = obs.position
    #             dist = math.sqrt((curr.x - obs_pos.x)**2 + (curr.y - obs_pos.y)**2)
    #             min_static_dist = min(min_static_dist, dist)

    #         # Dynamic obstacles
    #         for obs in self.dynamic_obstacles:
    #             obs_pos = obs.position
    #             dist = math.sqrt((curr.x - obs_pos.x)**2 + (curr.y - obs_pos.y)**2)
    #             min_dynamic_dist = min(min_dynamic_dist, dist)

    #     return min_static_dist, min_dynamic_dist

    # def calculate_personal_space_violations(self):
    #     violations = 0
    #     personal_space_radius = 1.0  # Define personal space radius

    #     for pose in self.path:
    #         curr = pose.pose.position

    #         for obs in self.dynamic_obstacles:
    #             obs_pos = obs.position
    #             dist = math.sqrt((curr.x - obs_pos.x)**2 + (curr.y - obs_pos.y)**2)
    #             if dist < personal_space_radius:
    #                 violations += 1

    #     return violations

def main(args=None):
    rclpy.init(args=args)
    node = PathEvaluationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
