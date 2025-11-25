#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
from ultralytics import YOLO

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        self.get_logger().info("🚀 YOLO Detection Node Starting...")
        
        # GPU 설정
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"📱 Using device: {self.device}")
        
        # YOLO 모델 로드
        self.get_logger().info("📦 Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')  # nano 버전 (가벼움)
        self.model.to(self.device)
        self.get_logger().info("✅ Model loaded successfully!")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 카메라 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera_left/image_raw',
            self.image_callback,
            10
        )
        
        # 퍼블리셔 생성
        self.class_publisher = self.create_publisher(String, '/target_class', 10)
        self.position_publisher = self.create_publisher(Point, '/target_position', 10)
        
        # 타이머 (1초마다 처리)
        self.timer = self.create_timer(1.0, self.process_frame)
        
        self.latest_frame = None
        self.frame_count = 0
        
        # 탐지할 클래스 (원하는 물체로 변경 가능)
        self.target_classes = ['person', 'bottle', 'cup', 'chair', 'book']
        
        self.get_logger().info("🎥 Subscribed to /camera_left/image_raw")
        self.get_logger().info(f"🎯 Target classes: {self.target_classes}")
    
    def image_callback(self, msg):
        """카메라 이미지 수신"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
    
    def process_frame(self):
        """1초마다 YOLO 추론"""
        if self.latest_frame is None:
            self.get_logger().warn("⏳ Waiting for camera image...")
            return
        
        self.get_logger().info(f"🖼️  Processing frame #{self.frame_count}")
        
        try:
            # YOLO 추론
            results = self.model(self.latest_frame, verbose=False)[0]
            
            # 결과 파싱
            detections = []
            for box in results.boxes:
                # 클래스 이름
                class_id = int(box.cls[0])
                class_name = results.names[class_id]
                
                # 신뢰도
                confidence = float(box.conf[0])
                
                # 바운딩 박스 (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # 중심점 계산
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bbox': (x1, y1, x2, y2),
                    'center': (center_x, center_y)
                })
            
            # 결과 출력 및 퍼블리시
            if detections:
                self.get_logger().info(f"✅ Detected {len(detections)} objects:")
                
                for det in detections:
                    self.get_logger().info(
                        f"  📦 {det['class']} "
                        f"(conf: {det['confidence']:.2f}, "
                        f"center: ({det['center'][0]:.0f}, {det['center'][1]:.0f}))"
                    )
                    
                    # target_classes에 있는 물체만 퍼블리시
                    if det['class'] in self.target_classes:
                        # 클래스 이름 퍼블리시
                        class_msg = String()
                        class_msg.data = det['class']
                        self.class_publisher.publish(class_msg)
                        
                        # 위치 퍼블리시
                        pos_msg = Point()
                        pos_msg.x = det['center'][0]
                        pos_msg.y = det['center'][1]
                        pos_msg.z = det['confidence']  # z에 신뢰도 저장
                        self.position_publisher.publish(pos_msg)
            else:
                self.get_logger().info("❌ No objects detected")
            
            # 시각화
            self.visualize_results(self.latest_frame, detections)
            
        except Exception as e:
            self.get_logger().error(f"❌ Detection error: {e}")
    
    def visualize_results(self, image, detections):
        """바운딩 박스 시각화"""
        display_img = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            
            # 바운딩 박스
            color = (0, 255, 0) if det['class'] in self.target_classes else (255, 0, 0)
            cv2.rectangle(
                display_img,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                color,
                2
            )
            
            # 레이블 (클래스 + 신뢰도)
            label = f"{det['class']}: {det['confidence']:.2f}"
            cv2.putText(
                display_img,
                label,
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )
            
            # 중심점
            center_x, center_y = det['center']
            cv2.circle(
                display_img,
                (int(center_x), int(center_y)),
                5,
                (0, 0, 255),
                -1
            )
        
        # 화면 표시
        cv2.imshow('YOLO Detection', display_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()