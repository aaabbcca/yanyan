#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import time

class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark_node')
        self.scan_count = 0
        self.cloud_count = 0
        self.start_time = time.time()
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        
        self.timer = self.create_timer(5.0, self.print_stats)
        
    def scan_callback(self, msg):
        self.scan_count += 1
        
    def cloud_callback(self, msg):
        self.cloud_count += 1
        
    def print_stats(self):
        elapsed = time.time() - self.start_time
        scan_hz = self.scan_count / elapsed
        cloud_hz = self.cloud_count / elapsed
        
        self.get_logger().info(f"""
        ⏱️  Time: {elapsed:.1f}s
        📊 LaserScan: {scan_hz:.2f} Hz
        ☁️  PointCloud: {cloud_hz:.2f} Hz
        """)

def main():
    rclpy.init()
    node = BenchmarkNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
