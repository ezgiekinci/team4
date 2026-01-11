#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from apriltag_msgs.msg import AprilTagDetectionArray

class FloorIdFromAprilTag(Node):
    def __init__(self):
        super().__init__('floor_id_from_apriltag')

        # --- params ---
        self.declare_parameter('min_margin', 50.0)
        self.declare_parameter('stable_count', 3)
        self.declare_parameter('tag_to_floor', {1: 1, 11: 2})  # id->floor

        self.min_margin = float(self.get_parameter('min_margin').value)
        self.stable_count = int(self.get_parameter('stable_count').value)

        # rclpy param returns dict as string sometimes; handle robustly
        tag_to_floor = self.get_parameter('tag_to_floor').value
        if isinstance(tag_to_floor, dict):
            self.tag_to_floor = {int(k): int(v) for k, v in tag_to_floor.items()}
        else:
            # fallback: keep defaults
            self.tag_to_floor = {1: 1, 11: 2}

        self.sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.cb, 10
        )
        self.pub = self.create_publisher(Int32, '/floor_id', 10)

        self.current_floor = None
        self.candidate_floor = None
        self.candidate_count = 0

        self.get_logger().info(
            f"Started. min_margin={self.min_margin}, stable_count={self.stable_count}, tag_to_floor={self.tag_to_floor}"
        )

    def cb(self, msg: AprilTagDetectionArray):
        # pick best detection by decision_margin among known tags
        best = None
        best_margin = -1.0

        for det in msg.detections:
            tid = int(det.id)
            if tid not in self.tag_to_floor:
                continue
            margin = float(det.decision_margin)
            if margin < self.min_margin:
                continue
            if margin > best_margin:
                best = tid
                best_margin = margin

        if best is None:
            return  # no usable tag in this frame

        new_floor = int(self.tag_to_floor[best])

        # stabilization (N consecutive frames)
        if self.candidate_floor != new_floor:
            self.candidate_floor = new_floor
            self.candidate_count = 1
        else:
            self.candidate_count += 1

        if self.current_floor != new_floor and self.candidate_count >= self.stable_count:
            self.current_floor = new_floor
            out = Int32()
            out.data = self.current_floor
            self.pub.publish(out)
            self.get_logger().info(f"FLOOR CHANGED -> {self.current_floor} (tag {best}, margin {best_margin:.1f})")


def main():
    rclpy.init()
    node = FloorIdFromAprilTag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

