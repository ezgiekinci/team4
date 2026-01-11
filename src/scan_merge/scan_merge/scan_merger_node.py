#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

def yaw_from_quat(q):
    # q: geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap(a):
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')

        # --- Params ---
        self.declare_parameter('front_topic', '/front_scan')
        self.declare_parameter('rear_topic',  '/rear_scan')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max',  math.pi)
        self.declare_parameter('angle_increment', 0.005775)  # front_scan'in increment'iyle aynı yaptık
        self.declare_parameter('range_min', 0.10)
        self.declare_parameter('range_max', 15.0)

        self.front_topic = self.get_parameter('front_topic').value
        self.rear_topic  = self.get_parameter('rear_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.out_angle_min = float(self.get_parameter('angle_min').value)
        self.out_angle_max = float(self.get_parameter('angle_max').value)
        self.out_inc = float(self.get_parameter('angle_increment').value)
        self.out_rmin = float(self.get_parameter('range_min').value)
        self.out_rmax = float(self.get_parameter('range_max').value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subs/pubs
        self.sub_front = self.create_subscription(LaserScan, self.front_topic, self.cb_front, 10)
        self.sub_rear  = self.create_subscription(LaserScan, self.rear_topic,  self.cb_rear,  10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.latest_front = None
        self.latest_rear = None

        # Precompute bins
        self.n_bins = int(round((self.out_angle_max - self.out_angle_min) / self.out_inc)) + 1

        self.get_logger().info(
            f"Merging {self.front_topic} + {self.rear_topic} -> {self.output_topic} in {self.target_frame}"
        )

    def cb_front(self, msg: LaserScan):
        self.latest_front = msg
        self.try_publish()

    def cb_rear(self, msg: LaserScan):
        self.latest_rear = msg
        self.try_publish()

    def try_publish(self):
        if self.latest_front is None or self.latest_rear is None:
            return

        # --- Simple time sync: only merge scans that are close in time ---
        t_f = self.latest_front.header.stamp.sec + 1e-9 * self.latest_front.header.stamp.nanosec
        t_r = self.latest_rear.header.stamp.sec + 1e-9 * self.latest_rear.header.stamp.nanosec
        if abs(t_f - t_r) > 0.05:  # 50 ms tolerance
            return

        # Output scan init
        out = LaserScan()
        out.header.stamp = self.latest_front.header.stamp
        out.header.frame_id = self.target_frame
        out.angle_min = self.out_angle_min
        out.angle_max = self.out_angle_max
        out.angle_increment = self.out_inc
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = self.out_rmin
        out.range_max = self.out_rmax
        out.ranges = [float('inf')] * self.n_bins
        out.intensities = []

        # Merge both scans
        self._accumulate_scan(self.latest_front, out)
        self._accumulate_scan(self.latest_rear, out)

        # Replace inf with range_max+eps (bazı node'lar inf sever, AMCL genelde OK; istersen dokunma)
        self.pub.publish(out)

    def _accumulate_scan(self, scan: LaserScan, out: LaserScan):
        # Scan points -> target_frame angles
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF missing {self.target_frame} <- {scan.header.frame_id}: {e}")
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)

        a = scan.angle_min
        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                a += scan.angle_increment
                continue
            if r < scan.range_min or r > scan.range_max:
                a += scan.angle_increment
                continue

            # point in scan frame
            px = r * math.cos(a)
            py = r * math.sin(a)

            # transform to target_frame (2D rigid transform)
            x = tx + (math.cos(yaw) * px - math.sin(yaw) * py)
            y = ty + (math.sin(yaw) * px + math.cos(yaw) * py)

            # angle in target_frame
            ang = math.atan2(y, x)
            dist = math.hypot(x, y)

            # bin index
            if ang < out.angle_min or ang > out.angle_max:
                a += scan.angle_increment
                continue
            idx = int(round((ang - out.angle_min) / out.angle_increment))
            if 0 <= idx < self.n_bins:
                # keep closest obstacle for that direction
                if dist < out.ranges[idx]:
                    out.ranges[idx] = dist

            a += scan.angle_increment


def main():
    rclpy.init()
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

