#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        self.declare_parameter('in_topic', '/scan_merged')
        self.declare_parameter('out_topic', '/scan')

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        # BEST_EFFORT QoS (SensorData-style)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(LaserScan, out_topic, qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, qos)

        self.get_logger().info(f"Relaying {in_topic} -> {out_topic} with BEST_EFFORT QoS")

    def cb(self, msg: LaserScan):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

