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
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO


class IntegratedRoadClassifier:
    def __init__(self):
        rospy.init_node('road_classifier_node')

        # ── 1. 경로 및 설정 ──────────────────────────────────
        self.log_file_path = "/home/omrob/road_classifier_test_log.txt"
        model_path = rospy.get_param(
            '~model_path',
            '/home/omrob/catkin_ws/src/road_classifier/models/omrob_class.pt'
        )
        self.conf_threshold = 0.7

        # ── 2. 모델 로드 ─────────────────────────────────────
        rospy.loginfo("Loading YOLO Model...")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # ── 3. 상태 관리 변수 ────────────────────────────────
        self.lock = threading.Lock()
        self.display_img = None
        self.last_slip_flag = -1
        self.last_road_name = ""
        self.frame_count = 0
        self.process_skip = 2    # 2프레임마다 1번 추론
        self.log_counter = 0
        self.log_skip = 10       # 10번 처리마다 1번 로그

        # ── 3-1. Navigation 시작 후에만 파일 기록하기 위한 변수 ──
        self.navigation_goal_received = False
        self.robot_moving = False
        self.log_session_started = False
        self.cmd_vel_move_threshold = 0.01

        # ── 4. 노면별 설정 ───────────────────────────────────
        # custom_amcl은 /road_condition만 사용한다.
        # /road_condition에 따라 AMCL 내부에서
        #   1) trigger threshold(current_update_min_d/a)
        #   2) initialpose covariance
        # 를 다르게 적용한다.
        self.surface_config = {
            'polished_concrete_hallway': {
                'flag': 0,
                'slip_score': 0.189,
            },
            'polished_granite_bright': {
                'flag': 1,
                'slip_score': 0.619,
            },
            'porcelain_tile_dark': {
                'flag': 2,
                'slip_score': 0.868,
            },
        }

        self.class_colors = {
            'polished_concrete_hallway': (255, 100, 0),
            'polished_granite_bright': (0, 200, 0),
            'porcelain_tile_dark': (0, 0, 255),
        }

        # ── 5. Publisher ─────────────────────────────────────
        # 노면 이름 토픽
        self.pub_road = rospy.Publisher('/road_condition', String, queue_size=10)

        # ── 6. Navigation / cmd_vel 구독 ─────────────────────
        # navigation goal이 지정되고, cmd_vel이 실제로 움직이기 시작한 뒤부터만 txt 기록
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.navigation_goal_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navigation_goal_cb)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)

        # ── 7. 카메라 동기화 구독 ────────────────────────────
        sub1 = message_filters.Subscriber('/camera_left/image_raw', Image)
        sub2 = message_filters.Subscriber('/camera_right/image_raw', Image)

        sync = message_filters.ApproximateTimeSynchronizer(
            [sub1, sub2],
            queue_size=5,
            slop=0.5
        )
        sync.registerCallback(self.synced_callback)

        rospy.loginfo("Road Classifier Ready.")
        rospy.loginfo("Publishing road condition to /road_condition")
        rospy.loginfo("TXT logging will start only after navigation goal + robot movement.")

        self.main_loop()

    # ────────────────────────────────────────────────────────
    # Navigation goal callback
    # ────────────────────────────────────────────────────────
    def navigation_goal_cb(self, msg):
        self.navigation_goal_received = True
        self.log_session_started = False

        rospy.loginfo(
            "[NAVIGATION GOAL RECEIVED] TXT logging will start when robot begins moving."
        )

    # ────────────────────────────────────────────────────────
    # cmd_vel callback
    # ────────────────────────────────────────────────────────
    def cmd_vel_cb(self, msg):
        moving = (
            abs(msg.linear.x) > self.cmd_vel_move_threshold or
            abs(msg.linear.y) > self.cmd_vel_move_threshold or
            abs(msg.angular.z) > self.cmd_vel_move_threshold
        )

        self.robot_moving = moving

    # ────────────────────────────────────────────────────────
    # 파일 기록 가능 여부
    # ────────────────────────────────────────────────────────
    def should_write_log(self):
        return self.navigation_goal_received and self.robot_moving

    # ────────────────────────────────────────────────────────
    # 로그 세션 시작 기록
    # ────────────────────────────────────────────────────────
    def start_log_session_if_needed(self):
        if self.log_session_started:
            return

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        with open(self.log_file_path, "a") as f:
            f.write(f"\n{'#' * 60}\n")
            f.write(f"### NAVIGATION LOG START: {timestamp} ###\n")
            f.write(f"{'#' * 60}\n")
            f.flush()
            os.fsync(f.fileno())

        self.log_session_started = True
        rospy.loginfo("[TXT LOGGING STARTED] navigation goal + robot movement detected.")

    # ────────────────────────────────────────────────────────
    # 분류
    # ────────────────────────────────────────────────────────
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
                    return cv_image, name, conf

            return cv_image, None, 0.0

        except Exception as e:
            rospy.logerr(f"Classification Error: {e}")
            return None, None, 0.0

    # ────────────────────────────────────────────────────────
    # 동기화 콜백
    # ────────────────────────────────────────────────────────
    def synced_callback(self, msg1, msg2):
        self.frame_count += 1

        if self.frame_count % self.process_skip != 0:
            return

        img1, r1, c1 = self.classify(msg1)
        img2, r2, c2 = self.classify(msg2)

        # 신뢰도 높은 카메라 결과 선택
        final_road = r1 if c1 >= c2 else r2

        if final_road:
            self.log_counter += 1
            config = self.surface_config.get(final_road)

            if config:
                slip_flag = config['flag']

                # 항상 발행
                self.pub_road.publish(String(data=final_road))

                # 노면 바뀔 때만 로그 + 터미널 출력
                if self.last_slip_flag != slip_flag:
                    rospy.loginfo(
                        f"[SURFACE CHANGE] "
                        f"{self.last_road_name} → {final_road} | "
                        f"flag={slip_flag} | "
                        f"slip_score={config['slip_score']}"
                    )

                    self.write_to_log(final_road, slip_flag, config)

                    self.last_slip_flag = slip_flag
                    self.last_road_name = final_road

                # 10프레임마다 주기 로그
                elif self.log_counter % self.log_skip == 0:
                    self.write_to_log(final_road, slip_flag, config)

        # 시각화
        vis1 = self.draw_ui(img1, "LEFT", r1, c1)
        vis2 = self.draw_ui(img2, "RIGHT", r2, c2)
        combined = np.hstack([vis1, vis2])

        with self.lock:
            self.display_img = combined

    # ────────────────────────────────────────────────────────
    # 로그 기록
    # ────────────────────────────────────────────────────────
    def write_to_log(self, road_type, flag, config):
        if not self.should_write_log():
            return

        try:
            self.start_log_session_if_needed()

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

            with open(self.log_file_path, "a") as f:
                if self.last_road_name != road_type:
                    f.write(f"\n{'-' * 60}\n")
                    f.write(f"[{timestamp}] CHANGE: {self.last_road_name} → {road_type}\n")
                    f.write(f"  FLAG           : {flag}\n")
                    f.write(f"  SLIP_SCORE     : {config['slip_score']}\n")
                    f.write(f"{'-' * 60}\n")
                else:
                    f.write(
                        f"[{timestamp}] "
                        f"ROAD: {road_type:25} | "
                        f"FLAG: {flag} | "
                        f"SLIP_SCORE: {config['slip_score']}\n"
                    )

                f.flush()
                os.fsync(f.fileno())

        except Exception as e:
            rospy.logerr(f"Log Error: {e}")

    # ────────────────────────────────────────────────────────
    # 시각화
    # ────────────────────────────────────────────────────────
    def draw_ui(self, img, side, name, conf):
        if img is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        res = img.copy()
        color = self.class_colors.get(name, (200, 200, 200))

        label = (
            f"{side}: {name} ({conf * 100:.1f}%)"
            if name else
            f"{side}: UNKNOWN"
        )

        cv2.putText(
            res,
            label,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        # 노면별 flag와 slip score 표시
        if name and name in self.surface_config:
            config = self.surface_config[name]
            road_label = (
                f"flag={config['flag']} "
                f"slip_score={config['slip_score']:.3f}"
            )

            cv2.putText(
                res,
                road_label,
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        return res

    # ────────────────────────────────────────────────────────
    # 메인 루프
    # ────────────────────────────────────────────────────────
    def main_loop(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            with self.lock:
                if self.display_img is not None:
                    cv2.imshow("Road Classifier", self.display_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    IntegratedRoadClassifier()
