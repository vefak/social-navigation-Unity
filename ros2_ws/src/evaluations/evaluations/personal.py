# evaluations/personal.py

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Header

# --- Your custom messages ---
# HumanArray has NO header in this option.
# We keep imports narrow to avoid dependency surprises.
from unitycustommsg.msg import (  # pragma: assume available
    Human,
    HumanArray,
    TwistTransformUnity,
)


class PersonalSpaceMonitor(Node):
    """
    Monitors distance between the robot and humans.
    Handles HumanArray WITHOUT a header by inferring/parametrizing frame_id.
    """

    def __init__(self) -> None:
        super().__init__("personal_space_monitor")

        # ---------------- Parameters ----------------
        self.declare_parameter("humans_topic", "/all_humans")
        self.declare_parameter("use_xz_plane", True)
        self.declare_parameter("personal_radius", 2.5)
        self.declare_parameter("critical_radius", 1.5)
        self.declare_parameter(
            "humans_frame", "map"
        )  # fallback when no header is present

        self.humans_topic: str = (
            self.get_parameter("humans_topic").get_parameter_value().string_value
        )
        self.use_xz: bool = (
            self.get_parameter("use_xz_plane").get_parameter_value().bool_value
        )
        self.personal_radius: float = (
            self.get_parameter("personal_radius").get_parameter_value().double_value
        )
        self.critical_radius: float = (
            self.get_parameter("critical_radius").get_parameter_value().double_value
        )
        self.humans_frame: str = (
            self.get_parameter("humans_frame").get_parameter_value().string_value
        )

        plane = "XZ" if self.use_xz else "XY"
        # self.get_logger().info(
        #    f"PersonalSpaceMonitor up: critical<{self.critical_radius} m, "
        #    f"personal<{self.personal_radius} m ({plane} plane)"
        # )

        # ---------------- Subscriptions ----------------
        self._robot_pos: Optional[Tuple[float, float, float]] = (
            None  # (x,y,z) in the chosen world
        )
        self._robot_frame: Optional[str] = None

        # Robot pose (Unity combo message)
        self.create_subscription(TwistTransformUnity, "/robot", self._robot_cb, 10)

        # Humans array (NO header in this option)
        self.create_subscription(HumanArray, self.humans_topic, self._humans_cb, 10)

        # ---------------- Publishers ----------------
        self.pub_min_dist = self.create_publisher(
            Float32, "/personal_space/min_distance", 10
        )
        self.pub_personal = self.create_publisher(
            Bool, "/personal_space/personal_breach", 10
        )
        self.pub_critical = self.create_publisher(
            Bool, "/personal_space/critical_breach", 10
        )
        self.pub_nearest = self.create_publisher(
            PointStamped, "/personal_space/nearest_human", 10
        )

    # ---------------- Utility: make a Header ----------------
    def _mk_header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    # ---------------- Robot callback ----------------
    def _robot_cb(self, msg: TwistTransformUnity) -> None:
        """
        Extract robot position from TwistTransformUnity.
        Expected fields (robust):
          - msg.transform.transform.translation.{x,y,z}  (geometry_msgs/TransformStamped)
          - or msg.transform (with nested fields)
        """
        # Frame inference for robot, if available
        frame = None
        try:
            frame = msg.transform.header.frame_id  # type: ignore[attr-defined]
        except Exception:
            frame = None
        self._robot_frame = frame or self._robot_frame  # keep last known if none

        # Extract translation robustly
        x = y = z = None
        try:
            # geometry_msgs/TransformStamped pattern
            t = msg.transform.transform.translation  # type: ignore[attr-defined]
            x, y, z = float(t.x), float(t.y), float(t.z)
        except Exception:
            # Fallbacks (very defensive)
            try:
                t = msg.transform.translation  # type: ignore[attr-defined]
                x, y, z = float(t.x), float(t.y), float(t.z)
            except Exception:
                # Last resort: look for fields directly
                for cand in ("tx", "x"), ("ty", "y"), ("tz", "z"):
                    pass  # not worth overfitting; if not found, leave None

        if x is not None and y is not None and z is not None:
            self._robot_pos = (x, y, z)

        # self.get_logger().info(
        #    f"🤖 Robot position updated: x={x:.2f}, y={y:.2f}, z={z:.2f}"
        # )

    # ---------------- Humans callback ----------------
    def _humans_cb(self, msg: HumanArray) -> None:
        """
        Handle HumanArray with NO header.
        We infer frame_id from first human if possible; otherwise use param `humans_frame`.
        """
        if self._robot_pos is None:
            # We can’t compute distances yet
            return

        # Frame inference (try nested headers; else fallback param)
        frame_id = self._infer_humans_frame(msg) or self.humans_frame

        # Compute nearest human distance
        rx, ry, rz = self._robot_pos
        nearest_d = math.inf
        nearest_point: Optional[Tuple[float, float, float]] = None

        humans = getattr(msg, "humans", [])
        if not humans:
            # Still publish "no breach" and +inf distance for completeness
            self._publish_results(nearest_d, False, False, frame_id, nearest_point)
            return

        for h in humans:
            hp = self._extract_human_position(h)  # (x,y,z) or None
            if hp is None:
                continue
            hx, hy, hz = hp
            hx = hx - 36.0
            hz = hz - 13.5
            if self.use_xz:
                d = math.hypot(hx - rx, hz - rz)
            else:
                d = math.hypot(hx - rx, hy - ry)
            if d < nearest_d:
                nearest_d = d
                nearest_point = (hx, hy, hz)
            # self.get_logger().info(
            #    f"[frame={frame_id}] Human[{h}]: pos=({hx:.2f}, {hy:.2f}, {hz:.2f})  "
            # )

        # Evaluate zones
        personal_breach = nearest_d < self.personal_radius
        critical_breach = nearest_d < self.critical_radius
        # self.get_logger().info(
        #    f"👥 Humans: {len(humans)} | Nearest distance = {nearest_d:.2f} m | "
        #    f"Personal breach: {personal_breach} | Critical: {critical_breach}"
        # )

        # Publish
        self._publish_results(
            nearest_d, personal_breach, critical_breach, frame_id, nearest_point
        )

    # ---------------- Helpers ----------------
    def _infer_humans_frame(self, msg: HumanArray) -> Optional[str]:
        """
        Try to infer a frame_id from the first human’s nested headers if present.
        Patterns tried:
          - human.transform.header.frame_id
          - human.pose.header.frame_id
        """
        humans = getattr(msg, "humans", [])
        if not humans:
            return None
        h0 = humans[0]

        # Try tra.nsform header
        try:
            return h0.transform.header.frame_id  # type: ignore[attr-defined]
        except Exception:
            pass

        # Try pose header
        try:
            return h0.pose.header.frame_id  # type: ignore[attr-defined]
        except Exception:
            pass

        # Nothing found
        return None

    def _extract_human_position(self, h: Human) -> Optional[Tuple[float, float, float]]:
        """
        Extract (x,y,z) from common human layouts:
          - h.transform.transform.translation.{x,y,z}
          - h.transform.translation.{x,y,z}
          - h.pose.pose.position.{x,y,z}
          - h.pose.position.{x,y,z}
          - h.position.{x,y,z}
        Returns None if nothing matches.
        """
        # transform.transform.translation
        try:
            tt = h.transform.transform.translation  # type: ignore[attr-defined]
            return float(tt.x), float(tt.y), float(tt.z)
        except Exception:
            pass
        # transform.translation
        try:
            t = h.transform.translation  # type: ignore[attr-defined]
            return float(t.x), float(t.y), float(t.z)
        except Exception:
            pass
        # pose.pose.position
        try:
            pp = h.pose.pose.position  # type: ignore[attr-defined]
            return float(pp.x), float(pp.y), float(pp.z)
        except Exception:
            pass
        # pose.position
        try:
            p = h.pose.position  # type: ignore[attr-defined]
            return float(p.x), float(p.y), float(p.z)
        except Exception:
            pass
        # position
        try:
            p = h.position  # type: ignore[attr-defined]
            return float(p.x), float(p.y), float(p.z)
        except Exception:
            pass

        return None

    def _publish_results(
        self,
        nearest_d: float,
        personal_breach: bool,
        critical_breach: bool,
        frame_id: str,
        nearest_point: Optional[Tuple[float, float, float]],
    ) -> None:
        # min distance
        dmsg = Float32()
        dmsg.data = float("inf") if math.isinf(nearest_d) else float(nearest_d)
        self.pub_min_dist.publish(dmsg)

        # flags
        pmsg = Bool()
        pmsg.data = bool(personal_breach)
        cmsg = Bool()
        cmsg.data = bool(critical_breach)
        self.pub_personal.publish(pmsg)
        self.pub_critical.publish(cmsg)

        # nearest point (if any), with constructed header
        if nearest_point is not None:
            hx, hy, hz = nearest_point
            pt = PointStamped()
            pt.header = self._mk_header(frame_id)
            pt.point.x, pt.point.y, pt.point.z = float(hx), float(hy), float(hz)
            self.pub_nearest.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = PersonalSpaceMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
