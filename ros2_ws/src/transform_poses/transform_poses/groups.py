import math
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from unitycustommsg.msg import Human, HumanArray, HumanGroup, HumanGroupArray


def _circle_from_2(a, b):
    cx = (a[0] + b[0]) / 2.0
    cy = (a[1] + b[1]) / 2.0
    r = math.hypot(a[0] - b[0], a[1] - b[1]) / 2.0
    return (cx, cy, r)


def _circle_from_3(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        circles = [_circle_from_2(a, b), _circle_from_2(a, c), _circle_from_2(b, c)]
        return max(circles, key=lambda cir: cir[2])
    ux = (
        (ax * ax + ay * ay) * (by - cy)
        + (bx * bx + by * by) * (cy - ay)
        + (cx * cx + cy * cy) * (ay - by)
    ) / d
    uy = (
        (ax * ax + ay * ay) * (cx - bx)
        + (bx * bx + by * by) * (ax - cx)
        + (cx * cx + cy * cy) * (bx - ax)
    ) / d
    r = math.hypot(ux - ax, uy - ay)
    return (ux, uy, r)


def _is_in_circle(p, c):
    x, y = p
    cx, cy, r = c
    return math.hypot(x - cx, y - cy) <= r + 1e-9


def minimal_enclosing_circle(points: List[Tuple[float, float]]):
    n = len(points)
    if n == 0:
        return (0.0, 0.0, 0.0)
    if n == 1:
        x, y = points[0]
        return (x, y, 0.0)
    best = None
    for i in range(n):
        for j in range(i + 1, n):
            c = _circle_from_2(points[i], points[j])
            if all(_is_in_circle(p, c) for p in points):
                if best is None or c[2] < best[2]:
                    best = c
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                c = _circle_from_3(points[i], points[j], points[k])
                if all(_is_in_circle(p, c) for p in points):
                    if best is None or c[2] < best[2]:
                        best = c
    if best is None:
        max_d = -1.0
        pa = pb = None
        for i in range(n):
            for j in range(i + 1, n):
                d = math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
                if d > max_d:
                    max_d = d
                    pa, pb = points[i], points[j]
        best = _circle_from_2(pa, pb)
    return best  # (cx, cy, r)


class HumanGroupingNode(Node):
    def __init__(self):
        super().__init__("human_grouping")

        # Params
        self.declare_parameter("distance_threshold", 1.5)
        self.declare_parameter(
            "angle_threshold_raw", 60.0
        )  # raw yaw diff (same units as orientation.y)
        self.declare_parameter("min_group_size", 2)
        self.declare_parameter("publish_tf", False)

        self.dist_thresh = float(self.get_parameter("distance_threshold").value)
        self.ang_thresh_raw = float(self.get_parameter("angle_threshold_raw").value)
        self.min_group_size = int(self.get_parameter("min_group_size").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # Latest humans from /all_humans
        self.humans: List[Human] = []

        # Subscribe to the **aggregated** topic so positions match your plugin
        self.create_subscription(HumanArray, "/all_humans", self._all_humans_cb, 10)

        # Publisher
        self.groups_pub = self.create_publisher(
            HumanGroupArray, "/human_groups/groups", 10
        )

        # Main loop
        self.create_timer(0.1, self._group_and_publish)

    def _all_humans_cb(self, msg: HumanArray):
        self.humans = list(msg.humans)

    def _group_and_publish(self):
        if not self.humans:
            return

        # Use Unity x,z from the aggregator, then map to ROS x,y when publishing
        pts: List[Tuple[float, float]] = []
        yaws_raw: List[float] = []
        for h in self.humans:
            t = h.transform.transform
            x = float(t.translation.x)
            z = float(t.translation.z)
            yaw_raw = float(t.rotation.y)  # raw yaw stored in .y
            pts.append((x, z))
            yaws_raw.append(yaw_raw)

        n = len(pts)
        visited = [False] * n
        adj = [[] for _ in range(n)]

        def angdiff_raw(a: float, b: float) -> float:
            return abs(a - b)

        for i in range(n):
            for j in range(i + 1, n):
                d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
                if (
                    d <= self.dist_thresh
                    and angdiff_raw(yaws_raw[i], yaws_raw[j]) <= self.ang_thresh_raw
                ):
                    adj[i].append(j)
                    adj[j].append(i)

        groups = []
        for i in range(n):
            if visited[i]:
                continue
            q = [i]
            visited[i] = True
            comp = [i]
            while q:
                u = q.pop(0)
                for v in adj[u]:
                    if not visited[v]:
                        visited[v] = True
                        q.append(v)
                        comp.append(v)
            groups.append(comp)

        groups = [g for g in groups if len(g) >= self.min_group_size]

        out = HumanGroupArray()
        # Use the same frame as /all_humans entries (typically "map")
        out.header.frame_id = self.humans[0].transform.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()

        for gi, idxs in enumerate(groups):
            xs = [pts[k][0] for k in idxs]
            zs = [pts[k][1] for k in idxs]
            mean_x = sum(xs) / len(xs)
            mean_z = sum(zs) / len(zs)
            mean_yaw_raw = sum(yaws_raw[k] for k in idxs) / len(idxs)

            _, _, r = minimal_enclosing_circle([(pts[k][0], pts[k][1]) for k in idxs])
            diameter = 2.0 * r

            g = HumanGroup()
            g.header.frame_id = out.header.frame_id
            g.header.stamp = out.header.stamp
            g.name = f"group_{gi}"
            g.child_frame_id = g.name

            g.center = Pose()
            # IMPORTANT: map Unity Z -> ROS Y
            g.center.position.x = float(mean_x)
            g.center.position.y = 0.0
            g.center.position.z = float(mean_z)  # <-- put mean_z into Y

            # Keep the "raw yaw in .y" convention
            g.center.orientation.x = 0.0
            g.center.orientation.y = float(mean_yaw_raw)
            g.center.orientation.z = 0.0
            g.center.orientation.w = 0.0

            g.diameter = float(diameter)
            out.groups.append(g)

        self.groups_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(HumanGroupingNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
