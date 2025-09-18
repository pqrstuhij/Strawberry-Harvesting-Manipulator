#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import random
import math
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO

class PaprikaDetector:
    def __init__(self):
        self.model = YOLO('/home/senes/catkin_ws/src/senes/robot_vision/lib/ARC_models/n.pt')  # 모델 경로
        self.publisher = rospy.Publisher('/detected_objects', Point, queue_size=10)
        self.bridge = CvBridge()
        self.conf_threshold = 0.6  # 신뢰도 임계값

        # 발행 조건 제어 변수
        self.prev_center = None          # 이전 발행 좌표
        self.min_distance = 20           # 이전 좌표와의 최소 거리 (픽셀)
        self.last_publish_time = 0.0     # 마지막 발행 시간
        self.publish_interval = 10.0      # 발행 최소 간격 (초)

    def __call__(self, image):
        results = self.model(image)
        valid_boxes = []

        # 1. 유효한 박스 수집 (신뢰도 ≥ 임계값 + 미숙한 고추 제외)
        for result in results:
            for box in result.boxes:
                if box.conf[0] >= self.conf_threshold:
                    label_name = self.model.names[int(box.cls[0])].strip().lower()
                    if label_name not in ["unripe pepper", "unripe peppers"]:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center = ((x1 + x2) // 2, (y1 + y2) // 2)
                        valid_boxes.append((center, box))

        # 2. 유효한 박스가 있을 경우 랜덤 선택 + 거리 검사
        if valid_boxes:
            selected_center, selected_box = self._select_random_box(valid_boxes)
            self._publish_and_visualize(image, selected_center, selected_box)
        else:
            rospy.loginfo("No valid detections.")

        cv2.imshow("YOLO Results", image)
        cv2.waitKey(1)

    def _select_random_box(self, valid_boxes):
        """랜덤 선택 + 이전 좌표와 거리 검사"""
        max_attempts = 10  # 최대 재시도 횟수
        for _ in range(max_attempts):
            # 랜덤으로 박스 선택
            center, box = random.choice(valid_boxes)
            
            # 이전 좌표와의 거리 계산
            if self.prev_center is None:
                return center, box  # 첫 발행인 경우 바로 반환
            
            distance = math.dist(center, self.prev_center)
            if distance >= self.min_distance:
                return center, box  # 거리 조건 충족 시 선택

        # 최대 시도 횟수 내 조건 미충족 → 첫 번째 박스 선택
        return valid_boxes[0]

    def _publish_and_visualize(self, image, center, box):
        """좌표 발행 + 시각화"""
        current_time = time.time()
        
        # 발행 시간 간격 검사
        if current_time - self.last_publish_time >= self.publish_interval:
            # Point 메시지 발행
            point_msg = Point()
            point_msg.x, point_msg.y = center
            self.publisher.publish(point_msg)
            
            # 상태 업데이트
            self.prev_center = center
            self.last_publish_time = current_time
            rospy.loginfo(f"Published: {center} (Conf: {box.conf[0]:.2f})")

        # 바운딩 박스 및 중심점 시각화
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        label = f"{self.model.names[int(box.cls[0])]}: {box.conf[0]:.2f}"
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

class PaprikaDetectorNode:
    def __init__(self):
        rospy.init_node('paprika_detector')
        self.detector = PaprikaDetector()
        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        try:
            cv_image = self.detector.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detector(cv_image)
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")

if __name__ == '__main__':
    node = PaprikaDetectorNode()
    rospy.spin()