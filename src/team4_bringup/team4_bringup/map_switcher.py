#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from nav2_msgs.srv import LoadMap
import os

class MapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')
        # AprilTag tespitlerini dinliyoruz [cite: 144]
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        
        # Nav2 harita yükleme servisine istemci oluşturuyoruz [cite: 146, 152]
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        
        self.current_floor = None
        # Haritalarınızın tam yolu [cite: 142]
        self.map_path = "/home/diakamos/ros2_ws/src/team4_multifloor/team4_maps/maps/"
        self.get_logger().info("Map Switcher Başlatıldı ve AprilTag bekleniyor...")

    def listener_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            
            # Tag 1 ise Zemin Kat (Floor 1), Tag 11 ise 1. Kat (Floor 2) 
            if tag_id == 1 and self.current_floor != 1:
                self.change_map("floor1.yaml", 1)
            elif tag_id == 11 and self.current_floor != 2:
                self.change_map("floor2.yaml", 2)

    def change_map(self, map_name, floor_id):
        if not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Nav2 Map Server servisi bulunamadı!')
            return
            
        req = LoadMap.Request()
        req.map_url = os.path.join(self.map_path, map_name)
        
        self.get_logger().info(f"Kat {floor_id} algılandı. Harita yükleniyor: {map_name}")
        self.map_client.call_async(req)
        self.current_floor = floor_id

def main(args=None):
    rclpy.init(args=args)
    node = MapSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
