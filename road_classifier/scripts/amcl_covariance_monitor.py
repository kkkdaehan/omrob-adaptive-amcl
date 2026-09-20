#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import csv
import os
import numpy as np

from std_msgs.msg import Float32, String, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from tf.transformations import euler_from_quaternion


class AMCLCovarianceMonitor:
    def __init__(self):
        rospy.init_node("amcl_covariance_monitor")

        self.current_material_scale = None
        self.current_road_condition = "UNKNOWN"

        self.last_amcl_cov = None
        self.last_particle_cov = None

        self.log_path = rospy.get_param(
            "~log_path",
            "/home/omrob/amcl_covariance_monitor_log.csv"
        )

        self.pub_debug = rospy.Publisher(
            "/amcl_covariance_debug",
            Float32MultiArray,
            queue_size=10
        )

        rospy.Subscriber("/material_scale", Float32, self.material_scale_cb)
        rospy.Subscriber("/road_condition", String, self.road_condition_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb)
        rospy.Subscriber("/particlecloud", PoseArray, self.particlecloud_cb)

        self.init_csv()

        rospy.loginfo("AMCL Covariance Monitor Started.")
        rospy.loginfo("Subscribing: /material_scale, /road_condition, /amcl_pose, /particlecloud")
        rospy.loginfo("Publishing: /amcl_covariance_debug")
        rospy.loginfo("CSV log: %s", self.log_path)

        rospy.spin()

    def init_csv(self):
        file_exists = os.path.exists(self.log_path)

        if not file_exists:
            with open(self.log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "time",
                    "road_condition",
                    "material_scale",

                    "amcl_var_x",
                    "amcl_var_y",
                    "amcl_var_yaw",
                    "amcl_std_x",
                    "amcl_std_y",
                    "amcl_std_yaw",

                    "particle_count",
                    "particle_var_x",
                    "particle_var_y",
                    "particle_var_yaw",
                    "particle_std_x",
                    "particle_std_y",
                    "particle_std_yaw"
                ])

    def material_scale_cb(self, msg):
        self.current_material_scale = msg.data

    def road_condition_cb(self, msg):
        self.current_road_condition = msg.data

    def amcl_pose_cb(self, msg):
        cov = msg.pose.covariance

        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]

        std_x = math.sqrt(max(var_x, 0.0))
        std_y = math.sqrt(max(var_y, 0.0))
        std_yaw = math.sqrt(max(var_yaw, 0.0))

        self.last_amcl_cov = {
            "var_x": var_x,
            "var_y": var_y,
            "var_yaw": var_yaw,
            "std_x": std_x,
            "std_y": std_y,
            "std_yaw": std_yaw,
        }

        self.publish_and_log()

    def particlecloud_cb(self, msg):
        count = len(msg.poses)

        if count < 2:
            return

        xs = []
        ys = []
        yaws = []

        for pose in msg.poses:
            xs.append(pose.position.x)
            ys.append(pose.position.y)

            q = pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            _, _, yaw = euler_from_quaternion(quat)
            yaws.append(yaw)

        xs = np.array(xs)
        ys = np.array(ys)
        yaws = np.array(yaws)

        var_x = float(np.var(xs))
        var_y = float(np.var(ys))

        # yaw는 각도라서 원형 평균 기반으로 분산 계산
        sin_mean = np.mean(np.sin(yaws))
        cos_mean = np.mean(np.cos(yaws))
        mean_yaw = math.atan2(sin_mean, cos_mean)

        yaw_diffs = np.array([
            math.atan2(math.sin(y - mean_yaw), math.cos(y - mean_yaw))
            for y in yaws
        ])

        var_yaw = float(np.var(yaw_diffs))

        std_x = math.sqrt(max(var_x, 0.0))
        std_y = math.sqrt(max(var_y, 0.0))
        std_yaw = math.sqrt(max(var_yaw, 0.0))

        self.last_particle_cov = {
            "count": count,
            "var_x": var_x,
            "var_y": var_y,
            "var_yaw": var_yaw,
            "std_x": std_x,
            "std_y": std_y,
            "std_yaw": std_yaw,
        }

        self.publish_and_log()

    def publish_and_log(self):
        if self.last_amcl_cov is None:
            return

        particle = self.last_particle_cov

        if particle is None:
            particle_count = 0
            p_var_x = 0.0
            p_var_y = 0.0
            p_var_yaw = 0.0
            p_std_x = 0.0
            p_std_y = 0.0
            p_std_yaw = 0.0
        else:
            particle_count = particle["count"]
            p_var_x = particle["var_x"]
            p_var_y = particle["var_y"]
            p_var_yaw = particle["var_yaw"]
            p_std_x = particle["std_x"]
            p_std_y = particle["std_y"]
            p_std_yaw = particle["std_yaw"]

        material_scale = (
            self.current_material_scale
            if self.current_material_scale is not None
            else -1.0
        )

        a = self.last_amcl_cov

        debug_msg = Float32MultiArray()
        debug_msg.data = [
            float(material_scale),

            float(a["var_x"]),
            float(a["var_y"]),
            float(a["var_yaw"]),
            float(a["std_x"]),
            float(a["std_y"]),
            float(a["std_yaw"]),

            float(particle_count),
            float(p_var_x),
            float(p_var_y),
            float(p_var_yaw),
            float(p_std_x),
            float(p_std_y),
            float(p_std_yaw),
        ]

        self.pub_debug.publish(debug_msg)

        rospy.loginfo_throttle(
            1.0,
            "[AMCL COV] road=%s | scale=%.2f | "
            "amcl_std=(x=%.4f, y=%.4f, yaw=%.4f) | "
            "particle_std=(x=%.4f, y=%.4f, yaw=%.4f) | particles=%d",
            self.current_road_condition,
            material_scale,
            a["std_x"],
            a["std_y"],
            a["std_yaw"],
            p_std_x,
            p_std_y,
            p_std_yaw,
            particle_count
        )

        self.write_csv(
            material_scale,
            a,
            particle_count,
            p_var_x,
            p_var_y,
            p_var_yaw,
            p_std_x,
            p_std_y,
            p_std_yaw
        )

    def write_csv(
        self,
        material_scale,
        amcl,
        particle_count,
        p_var_x,
        p_var_y,
        p_var_yaw,
        p_std_x,
        p_std_y,
        p_std_yaw
    ):
        try:
            with open(self.log_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    rospy.Time.now().to_sec(),
                    self.current_road_condition,
                    material_scale,

                    amcl["var_x"],
                    amcl["var_y"],
                    amcl["var_yaw"],
                    amcl["std_x"],
                    amcl["std_y"],
                    amcl["std_yaw"],

                    particle_count,
                    p_var_x,
                    p_var_y,
                    p_var_yaw,
                    p_std_x,
                    p_std_y,
                    p_std_yaw,
                ])
        except Exception as e:
            rospy.logerr("CSV write error: %s", str(e))


if __name__ == "__main__":
    try:
        AMCLCovarianceMonitor()
    except rospy.ROSInterruptException:
        pass
