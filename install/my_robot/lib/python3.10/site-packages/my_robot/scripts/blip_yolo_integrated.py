#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
from transformers import BlipProcessor, BlipForConditionalGeneration, BlipForQuestionAnswering
from PIL import Image as PILImage
import cv2
import threading
import sys
import time


class BLIP_YOLO_Node(Node):
    def __init__(self):
        super().__init__('blip_yolo_node')
        self.get_logger().info("🚀 BLIP + YOLO Node Starting...")

        # ✅ Device
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"📱 Using device: {self.device}")

        # ✅ BLIP Models (Caption + VQA)
        self.caption_model_name = "Salesforce/blip-image-captioning-large"
        self.vqa_model_name = "Salesforce/blip-vqa-base"

        self.caption_processor = BlipProcessor.from_pretrained(self.caption_model_name)
        self.caption_model = BlipForConditionalGeneration.from_pretrained(
            self.caption_model_name,
            torch_dtype=torch.float16 if self.device.type == 'cuda' else torch.float32
        ).to(self.device)

        self.vqa_processor = BlipProcessor.from_pretrained(self.vqa_model_name)
        self.vqa_model = BlipForQuestionAnswering.from_pretrained(
            self.vqa_model_name,
            torch_dtype=torch.float16 if self.device.type == 'cuda' else torch.float32
        ).to(self.device)

        # ✅ YOLOv8 model
        self.get_logger().info("📦 Loading YOLOv8 model...")
        self.yolo = YOLO("yolov8n.pt")
        self.get_logger().info("✅ YOLO model loaded successfully!")

        # ✅ ROS Image
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera_left/image_raw', self.image_callback, 10
        )

        self.latest_frame = None
        self.running = True

        self.get_logger().info("🎥 Subscribed to /camera_left/image_raw")
        sys.stdout.flush()
        time.sleep(0.2)

        # ✅ Input thread
        self.user_input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.user_input_thread.start()

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def _input_loop(self):
        while self.running:
            try:
                question = input("\n💬 질문 입력 (예: Is there a chair? / 종료: exit): ").strip()
                if question.lower() in ["exit", "quit", "종료"]:
                    self.get_logger().info("🛑 종료합니다.")
                    self.clean_exit()

                if self.latest_frame is None:
                    print("⚠️ 아직 카메라 프레임을 받지 못했습니다.")
                    continue

                rgb_image = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
                pil_image = PILImage.fromarray(rgb_image)

                # YOLO 탐지
                detections = self.run_yolo(self.latest_frame)

                # BLIP 질문/캡션 처리
                if self.is_question_type(question):
                    answer = self.run_vqa(pil_image, question)
                else:
                    answer = self.run_captioning(pil_image)

                # 시각화
                self.visualize_result(self.latest_frame, detections, answer)

            except KeyboardInterrupt:
                self.clean_exit()

    def run_yolo(self, frame):
        """YOLO로 객체 탐지"""
        results = self.yolo(frame, verbose=False)
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            label = self.yolo.names[cls]
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2
            detections.append({
                'label': label,
                'conf': conf,
                'coords': (x_center, y_center)
            })
        return detections

    def run_vqa(self, pil_image, question):
        self.get_logger().info(f"🧠 [VQA] Q: {question}")
        inputs = self.vqa_processor(images=pil_image, text=question, return_tensors="pt").to(self.device)
        with torch.no_grad():
            output = self.vqa_model.generate(**inputs, max_new_tokens=50)
        answer = self.vqa_processor.decode(output[0], skip_special_tokens=True)
        self.get_logger().info(f"✅ A: {answer}")
        return answer

    def run_captioning(self, pil_image):
        self.get_logger().info("🖼️ [Captioning] Generating scene description...")
        inputs = self.caption_processor(images=pil_image, return_tensors="pt").to(self.device)
        with torch.no_grad():
            output = self.caption_model.generate(**inputs, max_new_tokens=50)
        caption = self.caption_processor.decode(output[0], skip_special_tokens=True)
        self.get_logger().info(f"📝 Caption: {caption}")
        return caption

    def visualize_result(self, frame, detections, text):
        """YOLO + BLIP 결과 시각화"""
        display_img = frame.copy()
        # 🔹 YOLO 박스 그리기
        for det in detections:
            label = det['label']
            conf = det['conf']
            (x_center, y_center) = det['coords']
            x1 = int(x_center - 40)
            y1 = int(y_center - 40)
            x2 = int(x_center + 40)
            y2 = int(y_center + 40)
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(display_img, f"{label} ({conf:.2f})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            # 🔹 콘솔에 좌표 표시
            print(f"📍 {label}: ({x_center:.1f}, {y_center:.1f})")

        # 🔹 BLIP 답변 표시
        cv2.rectangle(display_img, (10, 10), (850, 80), (0, 0, 0), -1)
        cv2.putText(display_img, f"{text[:100]}", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('BLIP + YOLO Output', display_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.clean_exit()

    def is_question_type(self, text):
        keywords = ["?", "있", "where", "what", "is there", "are there", "who"]
        return any(k in text.lower() for k in keywords)

    def clean_exit(self):
        """안전 종료"""
        self.get_logger().info("🧹 종료 중...")
        self.running = False
        cv2.destroyAllWindows()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BLIP_YOLO_Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.clean_exit()
    finally:
        cv2.destroyAllWindows()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()