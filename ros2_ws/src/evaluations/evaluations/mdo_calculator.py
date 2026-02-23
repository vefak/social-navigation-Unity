import math

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from visualization_msgs.msg import Marker


class ObstacleDistanceCalculator(Node):
    def __init__(self):
        super().__init__("obstacle_distance_calculator")

        # ---- Parameters ----
        self.declare_parameter("occupancy_threshold", 65)  # > this ⇒ occupied
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("timer_period_sec", 0.5)
        self.declare_parameter("debug_visualization", True)
        self.declare_parameter("save_debug_images_once", False)

        # ---- Parameters (add this if not present) ----
        self.declare_parameter("publish_marker", True)
        self.publish_marker = bool(self.get_parameter("publish_marker").value)

        # ---- Publishers ----
        self.distance_pub = self.create_publisher(
            Float32, "/nearest_obstacle_distance", 10
        )
        self.marker_pub = self.create_publisher(Marker, "/nearest_obstacle_marker", 10)

        self.occ_thresh = int(self.get_parameter("occupancy_threshold").value)
        self.unknown_as_obs = bool(
            self.get_parameter("treat_unknown_as_obstacle").value
        )
        self.debug_vis = bool(self.get_parameter("debug_visualization").value)
        self.save_imgs = bool(self.get_parameter("save_debug_images_once").value)
        period = float(self.get_parameter("timer_period_sec").value)

        # ---- Subscriptions ----
        self.create_subscription(
            OccupancyGrid, "/occupancy_grid_map", self.map_callback, 10
        )

        # ---- TF2 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- State ----
        self.map_meta = None
        self.grid_img = None  # flipped 8-bit image (255 free, 128 unknown, 0 occupied)
        self.dist_transform = None  # cached float32 pixels
        self.saved_images = False

        # ---- Timer ----
        self.create_timer(period, self.timer_callback)

        # ---- Optional viz ----
        if self.debug_vis:
            import matplotlib.pyplot as plt

            plt.ion()
            self.plt = plt
            self.fig, self.ax = plt.subplots()
            self.image_plot = None

    def map_callback(self, msg: OccupancyGrid):
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin

        raw = np.asarray(msg.data, dtype=np.int16).reshape(
            (height, width)
        )  # use wider type for comparisons

        # Build 8-bit display mask: 255 free, 128 unknown, 0 occupied
        grid = np.zeros((height, width), dtype=np.uint8)
        # Free
        grid[raw == 0] = 255
        # Occupied by threshold
        grid[raw > self.occ_thresh] = 0
        # Unknown
        if self.unknown_as_obs:
            grid[raw == -1] = 0  # unknown treated as obstacle
        else:
            grid[raw == -1] = (
                128  # unknown visualized differently but considered free below
            )

        # Flip vertically for imshow-style indexing
        grid = cv2.flip(grid, 0)

        # Binary for distanceTransform: non-zero = free cells
        binary = np.where(grid == 255, 255, 0).astype(np.uint8)

        # Compute and cache distance transform (pixels)
        # Note: distanceTransform gives distance (in px) from each free cell to nearest zero cell (obstacle/unknown)
        dist = cv2.distanceTransform(binary, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

        self.grid_img = grid
        self.dist_transform = dist
        self.map_meta = {
            "resolution": res,
            "origin": origin,
            "width": width,
            "height": height,
        }

        # self.get_logger().info(f"Map received: {width}x{height} @ {res:.3f} m/px")

        # Optional one-time debug images
        if self.save_imgs and not self.saved_images:
            norm = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX)
            dist_u8 = norm.astype(np.uint8)
            cv2.imwrite("/tmp/distance_transform.png", dist_u8)
            color = cv2.applyColorMap(dist_u8, cv2.COLORMAP_JET)
            cv2.imwrite("/tmp/distance_transform_colored.png", color)
            self.saved_images = True
            self.get_logger().info("Saved /tmp/distance_transform*.png")

        # Warn if origin rotation is non-zero
        ori = origin.orientation
        yaw = self._yaw_from_quat(ori.x, ori.y, ori.z, ori.w)
        if abs(yaw) > math.radians(1.0):
            self.get_logger().warn(
                f"Map origin has yaw {math.degrees(yaw):.2f}°. Coordinate mapping assumes axis-aligned map."
            )

    def timer_callback(self):
        if self.map_meta is None or self.dist_transform is None:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame="map",
                source_frame="base_link",
                time=rclpy.time.Time(),  # latest
                timeout=Duration(seconds=0.5),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        distance_m, mx, my = self._sample_distance_at_xy(x, y)
        if distance_m is None:
            self.get_logger().warn("Robot is outside map bounds!")
            return

        # self.get_logger().info(
        #    f"Robot at x={x:.3f}, y={y:.3f} ⇒ nearest obstacle: {distance_m:.2f} m"
        # )
        # ---- Publish distance ----
        msg = Float32()
        msg.data = distance_m
        self.distance_pub.publish(msg)
        if self.publish_marker:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "nearest_obstacle"
            m.id = 0
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            # Centered on the robot; cylinder radius = distance
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = 2.0 * distance_m  # diameter in X
            m.scale.y = 2.0 * distance_m  # diameter in Y
            m.scale.z = 0.02  # 2 cm tall disc
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.35
            # Keep marker alive briefly so it updates smoothly
            m.lifetime = DurationMsg(sec=0, nanosec=int(0.6 * 1e9))
            self.marker_pub.publish(m)

        if self.debug_vis:
            self._visualize(mx, my)

    # ---------- helpers ----------

    def _sample_distance_at_xy(self, x, y):
        res = self.map_meta["resolution"]
        origin = self.map_meta["origin"]
        W = self.map_meta["width"]
        H = self.map_meta["height"]

        # map pixel coords accounting for vertical flip applied to the image
        mx = int((x - origin.position.x) / res)
        my = H - int((y - origin.position.y) / res)

        if not (0 <= mx < W and 0 <= my < H):
            return None, mx, my

        dist_px = self.dist_transform[my, mx]  # float32 pixels
        return float(dist_px * res), mx, my

    def _visualize(self, mx, my):
        debug = cv2.cvtColor(self.grid_img, cv2.COLOR_GRAY2BGR)
        if 0 <= mx < debug.shape[1] and 0 <= my < debug.shape[0]:
            cv2.circle(debug, (mx, my), 4, (0, 0, 255), -1)

        if self.image_plot is None:
            self.image_plot = self.ax.imshow(debug)
            self.ax.set_title("Robot Position on Map")
            self.ax.axis("off")
        else:
            self.image_plot.set_data(debug)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    @staticmethod
    def _yaw_from_quat(x, y, z, w):
        # yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistanceCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
