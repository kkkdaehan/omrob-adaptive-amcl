#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
from datetime import datetime

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

import math


class ScanMatchEvaluator:
    def __init__(self):
        rospy.init_node('scan_match_evaluator')

        # =========================
        # 바꿀 부분
        # =========================

        # 로그 저장 경로
        self.log_path = "/home/omrob/scan_match_log.txt"

        # ROI 사용 여부
        # True  = 맵의 특정 구간만 평가
        # False = 맵 전체 평가
        self.use_roi = True

        # 평가할 맵 구간 좌표
        # RViz에서 map 좌표 보고 네 실험 구간에 맞게 바꿔야 함
        self.roi_x_min = -0.531
        self.roi_x_max = 7.239
        self.roi_y_min = -2.283
        self.roi_y_max = -0.909

        # scan 점 주변 몇 픽셀 안에서 맵의 벽/장애물을 찾을지
        # map resolution이 0.05m라면 10픽셀 = 약 0.5m
        self.search_radius = 10

        # scan callback이 몇 번 들어올 때마다 기록할지
        # 10이면 scan 10번마다 1번 기록
        self.log_skip = 10

        # 노면 이름과 숫자 flag 매칭
        self.flag_map = {
            'polished_concrete_hallway': 0,
            'polished_granite_bright': 1,
            'porcelain_tile_dark': 2,
        }

        # =========================
        # 2. 내부 상태 변수
        # =========================

        self.map_data = None
        self.map_info = None
        self.amcl_pose = None

        self.current_road = "unknown"
        self.current_flag = -1

        self.log_counter = 0

        # =========================
        # 3. 로그 파일 헤더 작성
        # =========================

        with open(self.log_path, "a") as f:
            f.write(f"\n{'#' * 120}\n")
            f.write(f"### EVAL SESSION START: {datetime.now()} ###\n")
            f.write(f"{'#' * 120}\n")
            f.write(
                f"{'TIME':<25} "
                f"{'ROAD':<30} "
                f"{'FLAG':<6} "
                f"{'MEAN_ERR(m)':<12} "
                f"{'RMSE_ERR(m)':<12} "
                f"{'MAX_ERR(m)':<12} "
                f"{'MATCH_RATIO':<12} "
                f"{'VALID_SCAN':<12} "
                f"{'MATCHED':<10} "
                f"{'COV_X':<10} "
                f"{'COV_Y':<10} "
                f"{'COV_YAW':<10}\n"
            )
            f.write(f"{'-' * 150}\n")

        # =========================
        # 4. ROS Subscriber
        # =========================

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/road_condition', String, self.road_callback)

        rospy.loginfo("Scan Match Evaluator Ready.")
        rospy.spin()

    def map_callback(self, msg):
        # /map 데이터를 numpy 배열로 저장
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(
            msg.info.height,
            msg.info.width
        )

        rospy.loginfo_once("Map received.")

    def amcl_callback(self, msg):
        # AMCL이 추정한 현재 로봇 위치 저장
        self.amcl_pose = msg

    def road_callback(self, msg):
        # 현재 노면 상태 저장
        self.current_road = msg.data
        self.current_flag = self.flag_map.get(msg.data, -1)

    def scan_callback(self, scan_msg):
        # map 또는 pose가 아직 안 들어왔으면 계산 불가
        if self.map_data is None or self.map_info is None or self.amcl_pose is None:
            return

        # 로그를 너무 자주 찍지 않기 위해 일정 횟수마다만 실행
        self.log_counter += 1
        if self.log_counter % self.log_skip != 0:
            return

        # =========================
        # 1. AMCL pose에서 x, y, yaw 추출
        # =========================

        pose = self.amcl_pose.pose.pose

        rx = pose.position.x
        ry = pose.position.y

        q = pose.orientation

        # quaternion -> yaw 변환
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        ryaw = math.atan2(siny, cosy)

        # AMCL covariance 추출
        cov = list(self.amcl_pose.pose.covariance)
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]

        # =========================
        # 2. scan-to-map 오차 계산
        # =========================

        errors = []

        valid_scan_count = 0
        matched_count = 0

        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            current_angle = angle
            angle += scan_msg.angle_increment

            # 유효하지 않은 라이다 값 제거
            if math.isnan(r) or math.isinf(r):
                continue

            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            # scan 점을 map 좌표계로 변환
            px = rx + r * math.cos(ryaw + current_angle)
            py = ry + r * math.sin(ryaw + current_angle)

            # ROI 밖이면 제외
            if self.use_roi:
                if not self.is_inside_roi(px, py):
                    continue

            valid_scan_count += 1

            # map 좌표 -> occupancy grid 셀 좌표
            mx = int((px - self.map_info.origin.position.x) / self.map_info.resolution)
            my = int((py - self.map_info.origin.position.y) / self.map_info.resolution)

            # 맵 범위 밖이면 제외
            if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
                continue

            # scan 점 주변에서 가장 가까운 occupied cell 찾기
            dist_pixel = self.get_nearest_obstacle_dist(mx, my, self.search_radius)

            if dist_pixel is not None:
                dist_meter = dist_pixel * self.map_info.resolution
                errors.append(dist_meter)
                matched_count += 1

        if valid_scan_count == 0:
            rospy.logwarn("[EVAL] No valid scan points in ROI.")
            return

        if len(errors) == 0:
            rospy.logwarn(
                f"[EVAL] No matched obstacle cells. "
                f"ROAD={self.current_road}, VALID_SCAN={valid_scan_count}"
            )
            return

        # =========================
        # 3. 지표 계산
        # =========================

        mean_err = float(np.mean(errors))
        rmse_err = float(math.sqrt(np.mean(np.square(errors))))
        max_err = float(np.max(errors))
        match_ratio = matched_count / valid_scan_count

        # =========================
        # 4. 터미널 출력
        # =========================

        rospy.loginfo(
            f"[EVAL] "
            f"ROAD={self.current_road} | "
            f"FLAG={self.current_flag} | "
            f"MEAN={mean_err:.4f}m | "
            f"RMSE={rmse_err:.4f}m | "
            f"MAX={max_err:.4f}m | "
            f"MATCH={match_ratio:.2f} | "
            f"VALID={valid_scan_count} | "
            f"MATCHED={matched_count} | "
            f"COV_X={cov_x:.4f}"
        )

        # =========================
        # 5. 파일 저장
        # =========================

        self.write_log(
            mean_err,
            rmse_err,
            max_err,
            match_ratio,
            valid_scan_count,
            matched_count,
            cov_x,
            cov_y,
            cov_yaw
        )

    def is_inside_roi(self, px, py):
        # scan 점이 ROI 안에 있는지 확인
        return (
            self.roi_x_min <= px <= self.roi_x_max and
            self.roi_y_min <= py <= self.roi_y_max
        )

    def get_nearest_obstacle_dist(self, mx, my, search_radius=10):
        # scan 점 주변에서 가장 가까운 맵의 occupied cell 찾기
        min_dist = float('inf')

        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                nx = mx + dx
                ny = my + dy

                if nx < 0 or ny < 0 or nx >= self.map_info.width or ny >= self.map_info.height:
                    continue

                # OccupancyGrid에서 50보다 크면 벽/장애물로 판단
                if self.map_data[ny][nx] > 50:
                    dist = math.sqrt(dx * dx + dy * dy)

                    if dist < min_dist:
                        min_dist = dist

        if min_dist == float('inf'):
            return None

        return min_dist

    def write_log(
        self,
        mean_err,
        rmse_err,
        max_err,
        match_ratio,
        valid_scan_count,
        matched_count,
        cov_x,
        cov_y,
        cov_yaw
    ):
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

            with open(self.log_path, "a") as f:
                f.write(
                    f"{timestamp:<25} "
                    f"{self.current_road:<30} "
                    f"{self.current_flag:<6} "
                    f"{mean_err:<12.4f} "
                    f"{rmse_err:<12.4f} "
                    f"{max_err:<12.4f} "
                    f"{match_ratio:<12.4f} "
                    f"{valid_scan_count:<12} "
                    f"{matched_count:<10} "
                    f"{cov_x:<10.4f} "
                    f"{cov_y:<10.4f} "
                    f"{cov_yaw:<10.4f}\n"
                )

                f.flush()
                os.fsync(f.fileno())

        except Exception as e:
            rospy.logerr(f"Log Error: {e}")


if __name__ == '__main__':
    try:
        ScanMatchEvaluator()
    except rospy.ROSInterruptException:
        pass
