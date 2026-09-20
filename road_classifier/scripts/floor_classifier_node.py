#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
import cv2
import numpy as np
import threading
import os
from datetime import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from ultralytics import YOLO

class IntegratedRoadClassifier:
    def __init__(self):
        rospy.init_node('road_classifier_node')

        # 1. 설정 및 경로
        self.log_file_path = "/home/omrob/road_classifier_test_log.txt"
        model_path = rospy.get_param('~model_path', '/home/omrob/catkin_ws/src/road_classifier/models/omrob_class.pt')
        self.conf_threshold = 0.4
        
        # 2. 모델 및 도구 로드
        rospy.loginfo("Loading YOLO Model...")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        
        # 3. 상태 관리 변수
        self.lock = threading.Lock()
        self.display_img = None
        self.last_amcl_pose = None
        self.last_slip_flag = -1
        self.frame_count = 0
        self.skip_frames = 2

        # 바닥 재질별 슬립 점수 및 색상 설정
        self.slip_score_map = {
            "polished_concrete_hallway": 40.97,
            "polished_granite_bright": 34.23,
            "porcelain_tile_dark": 90.22,
            "general_floor": 26.30
        }
        self.class_colors = {
            'polished_concrete_hallway': (255, 100, 0),
            'polished_granite_bright':   (0, 200, 0),
            'porcelain_tile_dark':       (0, 0, 255),
        }

        # 4. Pub/Sub 설정
        self.pub_road = rospy.Publisher('/road_condition', String, queue_size=10)
        self.pub_initial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        # 카메라 동기화 구독
        sub1 = message_filters.Subscriber('/camera_left/image_raw', Image)
        sub2 = message_filters.Subscriber('/camera_right/image_raw', Image)
        sync = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=5, slop=0.5)
        sync.registerCallback(self.synced_callback)

        rospy.loginfo("System Ready. Logging to: " + self.log_file_path)
        self.main_loop()

    def amcl_pose_callback(self, msg):
        self.last_amcl_pose = msg

    def write_to_log(self, road_type, flag, cov):
        """파일에 즉시 물리적 기록"""
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            with open(self.log_file_path, "a") as f:
                f.write(f"[{timestamp}] ROAD: {road_type:25} | FLAG: {flag} | COV: {cov}\n")
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            rospy.logerr(f"Log Error: {e}")

    def classify(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)
            probs = results[0].probs
            if probs is not None:
                idx = probs.top1
                conf = float(probs.top1conf)
                name = self.model.names[idx]
                if conf >= self.conf_threshold:
                    all_p = {self.model.names[i]: float(probs.data[i]) for i in range(len(self.model.names))}
                    return cv_image, name, conf, all_p
            return cv_image, None, 0.0, {}
        except:
            return None, None, 0.0, {}

    def synced_callback(self, msg1, msg2):
        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0: return

        img1, r1, c1, p1 = self.classify(msg1)
        img2, r2, c2, p2 = self.classify(msg2)

        # 최종 결정 (더 높은 신뢰도 선택)
        final_road = r1 if c1 >= c2 else r2
        
        if final_road:
            score = self.slip_score_map.get(final_road, 0.0)
            
            # 위험도 플래그 및 공분산 결정
            if score < 40:   slip_flag, target_cov = 0, 0.10
            elif score < 70: slip_flag, target_cov = 1, 0.15
            else:            slip_flag, target_cov = 2, 0.20

            # 1. 토픽 발행
            self.pub_road.publish(String(data=final_road))
            
            # 2. 파일 기록
            self.write_to_log(final_road, slip_flag, target_cov)

            # 3. 상태 변화 시 AMCL 공분산 업데이트
            if self.last_slip_flag != slip_flag:
                rospy.loginfo(f"Changed to {final_road} (Flag {slip_flag})")
                if self.last_amcl_pose:
                    new_pose = PoseWithCovarianceStamped()
                    new_pose.header = self.last_amcl_pose.header
                    new_pose.pose.pose = self.last_amcl_pose.pose.pose
                    cov = list(self.last_amcl_pose.pose.covariance)
                    cov[0], cov[7], cov[35] = target_cov, target_cov, target_cov
                    new_pose.pose.covariance = cov
                    self.pub_initial_pose.publish(new_pose)
                self.last_slip_flag = slip_flag

        # 시각화 데이터 합성
        vis1 = self.draw_ui(img1, "LEFT", r1, c1)
        vis2 = self.draw_ui(img2, "RIGHT", r2, c2)
        combined = np.hstack([vis1, vis2])
        with self.lock:
            self.display_img = combined

    def draw_ui(self, img, side, name, conf):
        if img is None: return np.zeros((480, 640, 3), dtype=np.uint8)
        res = img.copy()
        color = self.class_colors.get(name, (0, 255, 0))
        cv2.putText(res, f"{side}: {name} ({conf*100:.1f}%)", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return res

    def main_loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            with self.lock:
                if self.display_img is not None:
                    cv2.imshow("Road Navigation Monitor", self.display_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    IntegratedRoadClassifier()
