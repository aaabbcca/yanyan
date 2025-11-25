#!/usr/bin/env python3
"""
CLIP VQA + Navigation (WITH DEPTH CAMERA!)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import torch
import math
import numpy as np
from ultralytics import YOLO
from transformers import CLIPProcessor, CLIPModel
from datetime import datetime
import threading
import time


class CLIPVQANode(Node):
    def __init__(self):
        super().__init__('clip_vqa')

        self.object_memory = []
        self.object_id_counter = 1
        self.current_pose = None
        self.current_yaw = 0.0

        # Camera
        self.bridge = CvBridge()
        
        # 🆕 RGB + Depth 카메라!
        self.rgb_sub = self.create_subscription(
            Image, '/camera_center/camera_center/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera/depth_camera/depth/image_raw', self.depth_callback, 10
        )
        
        self.latest_image = None
        self.depth_image = None  # 🆕 Depth 이미지

        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Goal publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # YOLO
        self.yolo = YOLO('yolov8n.pt')
        self.get_logger().info('✅ YOLO Loaded')

        # CLIP
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(self.device)
        self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.get_logger().info(f'✅ CLIP Loaded on {self.device}')

        # Timer for detection
        self.create_timer(3.0, self.detect_and_save)

        self.get_logger().info('🚀 CLIP VQA Node Started')

        # quit flag
        self.shutdown_requested = False

    def rgb_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg):
        """🆕 Depth 이미지 수신"""
        try:
            # Depth 이미지는 32FC1 형식
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
        }
        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def detect_and_save(self):
        if self.latest_image is None or self.current_pose is None:
            return

        results = self.yolo(self.latest_image, verbose=False)

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = self.yolo.names[cls]

            if conf > 0.5:
                crop_img = self.latest_image[y1:y2, x1:x2].copy()
                map_pos = self.calculate_map_position(x1, y1, x2, y2)

                if map_pos and crop_img.size > 0:
                    memory_entry = {
                        'id': self.object_id_counter,
                        'class': label,
                        'image': crop_img,
                        'pose': map_pos,
                        'timestamp': datetime.now().isoformat(),
                    }

                    self.object_memory.append(memory_entry)
                    self.object_id_counter += 1

                    self.get_logger().info(f'💾 Saved {label} #{memory_entry["id"]} at ({map_pos["x"]:.2f}, {map_pos["y"]:.2f})')

    def get_depth_at_pixel(self, x, y):
        """🆕 픽셀 위치에서 실제 거리 가져오기"""
        if self.depth_image is None:
            self.get_logger().warn('⚠️ No depth image, using default distance')
            return 0.8  # fallback: 0.8m
        
        try:
            # Depth 이미지 크기 확인
            height, width = self.depth_image.shape
            
            # 범위 체크
            if 0 <= y < height and 0 <= x < width:
                depth_value = self.depth_image[y, x]
                
                # 유효한 depth인지 확인 (0 < depth < 10m)
                if 0.1 < depth_value < 10.0 and not np.isnan(depth_value):
                    # 🆕 안전 거리로 조정 (실제 거리의 80%)
                    safe_distance = depth_value * 0.7 ##바꿨음 0.8 → 0.9
                    self.get_logger().info(f'📏 Depth: {depth_value:.2f}m → Safe: {safe_distance:.2f}m')
                    return safe_distance
        except Exception as e:
            self.get_logger().warn(f'Depth read error: {e}')
        
        return 0.8  # fallback

    def calculate_map_position(self, x1, y1, x2, y2):
        """카메라 픽셀 → 맵 좌표 (Depth 사용!)"""
        
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        
        # 🆕 실제 거리 가져오기!
        estimated_distance = self.get_depth_at_pixel(center_x, center_y)
        
        # 카메라 파라미터
        image_width = 640
        fx = 320.0
        
        # 좌우 offset 계산
        pixel_offset = center_x - (image_width / 2)
        angle_offset = pixel_offset / fx
        
        # 로봇 좌표계
        x_robot = estimated_distance
        y_robot = estimated_distance * math.tan(angle_offset)
        
        # 맵 좌표계로 변환
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        map_x = self.current_pose['x'] + (x_robot * cos_yaw - y_robot * sin_yaw)
        map_y = self.current_pose['y'] + (x_robot * sin_yaw + y_robot * cos_yaw)
        
        # 유효성 검사
        if abs(map_x) > 50 or abs(map_y) > 50:
            self.get_logger().warn(f"⚠️ Invalid goal: ({map_x:.2f}, {map_y:.2f})")
            return None
        
        return {'x': map_x, 'y': map_y}

    def handle_query(self, query):

        if not self.object_memory:
            return None, "❌ Memory empty"

        images = []
        for obj in self.object_memory:
            img_rgb = cv2.cvtColor(obj['image'], cv2.COLOR_BGR2RGB)
            images.append(img_rgb)

        inputs = self.clip_processor(
            text=[query, f"not {query}"],
            images=images,
            return_tensors="pt",
            padding=True
        ).to(self.device)

        with torch.no_grad():
            outputs = self.clip_model(**inputs)

        probs = outputs.logits_per_image.softmax(dim=1)
        best_idx = probs[:, 0].argmax().item()
        best_score = probs[best_idx, 0].item()

        if best_score > 0.3:
            best_obj = self.object_memory[best_idx]
            pose = best_obj["pose"]

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = pose["x"]
            goal.pose.position.y = pose["y"]
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)
            
            self.get_logger().info(f'🚀 Published goal: ({pose["x"]:.2f}, {pose["y"]:.2f})')

            return pose, f"✅ Found {best_obj['class']} #{best_obj['id']} (score {best_score:.2f}) → Navigating to ({pose['x']:.2f}, {pose['y']:.2f})"

        return None, f"❌ Not found (best score {best_score:.2f})"

    def list_memory(self):
        if not self.object_memory:
            return "❌ Memory empty"

        by_class = {}
        for obj in self.object_memory:
            cls = obj['class']
            if cls not in by_class:
                by_class[cls] = []
            by_class[cls].append(obj)

        result = f"💾 Total: {len(self.object_memory)} objects\n"
        for cls, objs in by_class.items():
            result += f"\n📦 {cls} ({len(objs)}):\n"
            for obj in objs:
                result += f"  #{obj['id']}: ({obj['pose']['x']:.2f}, {obj['pose']['y']:.2f})\n"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CLIPVQANode()

    def input_thread():
        time.sleep(2)
        print("\n" + "="*60)
        print("🧠 CLIP VQA System (WITH DEPTH!)")
        print("="*60)
        print("\nCommands:")
        print("  'list' → Show saved objects")
        print("  'person' → Search for person")
        print("  'quit' → Exit\n")
        
        while rclpy.ok() and not node.shutdown_requested:
            try:
                cmd = input("💬 Query: ").strip()

                if cmd.lower() in ["quit", "exit", "q"]:
                    print("\n🔴 Shutting down...")
                    node.shutdown_requested = True
                    break

                if cmd.lower() == "list":
                    print(node.list_memory())
                    continue

                if cmd:
                    pose, msg = node.handle_query(cmd)
                    print("\n" + msg + "\n")
                    
            except (EOFError, KeyboardInterrupt):
                print("\n🔴 Interrupted, shutting down...")
                node.shutdown_requested = True
                break

    th = threading.Thread(target=input_thread, daemon=True)
    th.start()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print("🔴 Cleaning up...")
    node.destroy_node()
    
    try:
        rclpy.shutdown()
    except:
        pass


if __name__ == '__main__':
    main()