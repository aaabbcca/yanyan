#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from transformers import Blip2ForConditionalGeneration, AutoImageProcessor, AutoTokenizer
from PIL import Image as PILImage
import cv2
import numpy as np

class BLIP2VisionNode(Node):
    def __init__(self):
        super().__init__('blip2_vision_node')
        
        self.get_logger().info("🚀 BLIP-2 Vision Node Starting...")
        
        # GPU 설정
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"📱 Using device: {self.device}")
        
        # BLIP-2 모델 로드
        self.get_logger().info("📦 Loading BLIP-2 model...")
        model_name = "Salesforce/blip2-opt-2.7b"  # 또는 "Salesforce/blip2-flan-t5-xl"
        
        self.image_processor = AutoImageProcessor.from_pretrained(model_name)
        self.tokenizer = AutoTokenizer.from_pretrained(model_name, use_fast=False)
        self.model = Blip2ForConditionalGeneration.from_pretrained(
            model_name,
            torch_dtype=torch.float16 if self.device.type == 'cuda' else torch.float32
        ).to(self.device)
        
        self.get_logger().info("✅ Model loaded successfully!")
        
        # CV Bridge (ROS Image ↔ OpenCV)
        self.bridge = CvBridge()
        
        # 카메라 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera_left/image_raw',  # 토픽 이름
            self.image_callback,
            10
        )
        
        # 최신 프레임 저장
        self.latest_frame = None
        self.frame_count = 0
        
        # 1초마다 처리 (프레임 제어)
        self.timer = self.create_timer(1.0, self.process_frame)
        
        # 자연어 명령 (나중에 토픽으로 받을 수도 있음)
        self.question = "What objects are in this image?"
        
        self.get_logger().info("🎥 Subscribed to /camera_left/image_raw")
        self.get_logger().info(f"❓ Question: {self.question}")
    
    def image_callback(self, msg):
        """카메라 이미지 수신 (매 프레임)"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_frame = cv_image
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
    
    def process_frame(self):
        """1초마다 이미지 처리"""
        if self.latest_frame is None:
            self.get_logger().warn("⏳ Waiting for camera image...")
            return
        
        self.get_logger().info(f"🖼️  Processing frame #{self.frame_count}")
        
        try:
            # OpenCV (BGR) → PIL (RGB)
            rgb_image = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # BLIP-2 입력 준비
            pixel_values = self.image_processor(images=pil_image, return_tensors="pt").pixel_values
            pixel_values = pixel_values.to(self.device, torch.float16 if self.device.type == 'cuda' else torch.float32)
            
            input_ids = self.tokenizer(self.question, return_tensors="pt").input_ids.to(self.device)
            
            # 추론
            with torch.no_grad():
                generated_ids = self.model.generate(pixel_values=pixel_values, max_new_tokens=50)
            
            # 결과 디코딩
            answer = self.tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0].strip()
            
            # 결과 출력
            self.get_logger().info(f"❓ Q: {self.question}")
            self.get_logger().info(f"✅ A: {answer}")
            self.get_logger().info("─" * 50)
            
            # 시각화 (선택사항)
            self.visualize_result(self.latest_frame, answer)
            
        except Exception as e:
            self.get_logger().error(f"❌ Processing error: {e}")
    
    def visualize_result(self, image, text):
        """결과를 이미지에 표시"""
        display_img = image.copy()
        
        # 텍스트 배경
        cv2.rectangle(display_img, (10, 10), (700, 80), (0, 0, 0), -1)
        
        # 텍스트 (답변)
        cv2.putText(
            display_img,
            f"Answer: {text}",
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # 화면 표시
        cv2.imshow('BLIP-2 Vision', display_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BLIP2VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()