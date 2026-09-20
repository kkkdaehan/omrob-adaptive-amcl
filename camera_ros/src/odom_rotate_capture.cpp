#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>


class OdomRotateCapture
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    ros::Subscriber odom_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber accel_sub_;
    ros::Subscriber gyro_sub_;

    ros::Publisher cmd_pub_;


    // =====================================================
    // Launch 파라미터
    // =====================================================

    std::string odom_topic_;
    std::string image_topic_;
    std::string accel_topic_;
    std::string gyro_topic_;
    std::string cmd_topic_;
    std::string save_dir_;

    std::string rotation_direction_;

    double rotation_deg_;
    double capture_interval_deg_;
    double angular_speed_;
    double control_rate_;


    // =====================================================
    // ODOM
    // =====================================================

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    double prev_yaw_ = 0.0;

    ros::Time odom_stamp_;


    // =====================================================
    // 시작 위치
    // =====================================================

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;


    // =====================================================
    // 상대 ODOM
    // =====================================================

    double relative_x_ = 0.0;
    double relative_y_ = 0.0;
    double relative_yaw_ = 0.0;


    // =====================================================
    // 회전량
    // =====================================================

    double rotated_angle_ = 0.0;
    double capture_angle_ = 0.0;


    // =====================================================
    // CAMERA
    // =====================================================

    cv::Mat latest_image_;

    ros::Time image_stamp_;

    int image_count_ = 0;


    // =====================================================
    // ACCEL
    // =====================================================

    double accel_x_ = 0.0;
    double accel_y_ = 0.0;
    double accel_z_ = 0.0;

    ros::Time accel_stamp_;


    // =====================================================
    // GYRO
    // =====================================================

    double gyro_x_ = 0.0;
    double gyro_y_ = 0.0;
    double gyro_z_ = 0.0;

    ros::Time gyro_stamp_;


    // =====================================================
    // SENSOR READY
    // =====================================================

    bool odom_ready_ = false;
    bool image_ready_ = false;
    bool accel_ready_ = false;
    bool gyro_ready_ = false;

    bool running_ = false;


public:

    OdomRotateCapture()
    {
        // =================================================
        // Topic
        // =================================================

        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/odom"
        );

        pnh_.param<std::string>(
            "image_topic",
            image_topic_,
            "/camera/color/image_raw"
        );

        pnh_.param<std::string>(
            "accel_topic",
            accel_topic_,
            "/camera/accel/sample"
        );

        pnh_.param<std::string>(
            "gyro_topic",
            gyro_topic_,
            "/camera/gyro/sample"
        );

        pnh_.param<std::string>(
            "cmd_vel_topic",
            cmd_topic_,
            "/cmd_vel"
        );


        // =================================================
        // 저장 폴더
        // =================================================

        pnh_.param<std::string>(
            "save_dir",
            save_dir_,
            "/home/omrob/catkin_ws/src/camera_ros/src/연구실_회전_day0"
        );


        // =================================================
        // 회전 설정
        // =================================================

        pnh_.param(
            "rotation_deg",
            rotation_deg_,
            360.0
        );

        pnh_.param(
            "capture_interval_deg",
            capture_interval_deg_,
            5.0
        );

        pnh_.param(
            "angular_speed",
            angular_speed_,
            0.1853020188851842
        );

        pnh_.param<std::string>(
            "rotation_direction",
            rotation_direction_,
            "left"
        );

        pnh_.param(
            "control_rate",
            control_rate_,
            20.0
        );


        // =================================================
        // ROS Subscriber
        // =================================================

        odom_sub_ = nh_.subscribe(
            odom_topic_,
            20,
            &OdomRotateCapture::odomCallback,
            this
        );

        image_sub_ = nh_.subscribe(
            image_topic_,
            1,
            &OdomRotateCapture::imageCallback,
            this
        );

        accel_sub_ = nh_.subscribe(
            accel_topic_,
            100,
            &OdomRotateCapture::accelCallback,
            this
        );

        gyro_sub_ = nh_.subscribe(
            gyro_topic_,
            200,
            &OdomRotateCapture::gyroCallback,
            this
        );


        // =================================================
        // cmd_vel Publisher
        // =================================================

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(
            cmd_topic_,
            10
        );


        ROS_INFO("====================================");
        ROS_INFO("ROTATION CAPTURE");
        ROS_INFO("Rotation      : %.1f deg", rotation_deg_);
        ROS_INFO("Capture       : %.1f deg", capture_interval_deg_);
        ROS_INFO("Angular speed : %.6f rad/s", angular_speed_);
        ROS_INFO("Direction     : %s", rotation_direction_.c_str());
        ROS_INFO("Save dir      : %s", save_dir_.c_str());
        ROS_INFO("====================================");
    }


    ~OdomRotateCapture()
    {
        stopRobot();
    }


    // =====================================================
    // Quaternion -> Yaw
    // =====================================================

    double getYaw(
        const geometry_msgs::Quaternion& q
    )
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
    }


    // =====================================================
    // Angle Normalize
    // =====================================================

    double normalizeAngle(
        double angle
    )
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }


    // =====================================================
    // CAMERA CALLBACK
    // =====================================================

    void imageCallback(
        const sensor_msgs::ImageConstPtr& msg
    )
    {
        try
        {
            latest_image_ =
                cv_bridge::toCvShare(
                    msg,
                    "bgr8"
                )->image.clone();

            image_stamp_ =
                msg->header.stamp;

            image_ready_ = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR(
                "cv_bridge: %s",
                e.what()
            );
        }
    }


    // =====================================================
    // ACCEL CALLBACK
    // =====================================================

    void accelCallback(
        const sensor_msgs::Imu::ConstPtr& msg
    )
    {
        accel_x_ =
            msg->linear_acceleration.x;

        accel_y_ =
            msg->linear_acceleration.y;

        accel_z_ =
            msg->linear_acceleration.z;

        accel_stamp_ =
            msg->header.stamp;

        accel_ready_ = true;
    }


    // =====================================================
    // GYRO CALLBACK
    // =====================================================

    void gyroCallback(
        const sensor_msgs::Imu::ConstPtr& msg
    )
    {
        gyro_x_ =
            msg->angular_velocity.x;

        gyro_y_ =
            msg->angular_velocity.y;

        gyro_z_ =
            msg->angular_velocity.z;

        gyro_stamp_ =
            msg->header.stamp;

        gyro_ready_ = true;
    }


    // =====================================================
    // ODOM CALLBACK
    // =====================================================

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {
        x_ =
            msg->pose.pose.position.x;

        y_ =
            msg->pose.pose.position.y;

        yaw_ =
            getYaw(
                msg->pose.pose.orientation
            );

        odom_stamp_ =
            msg->header.stamp;


        // 최초 odom
        if (!odom_ready_)
        {
            prev_yaw_ = yaw_;

            odom_ready_ = true;

            ROS_INFO("ODOM READY");

            return;
        }


        // 회전 시작 전에는 yaw만 갱신
        if (!running_)
        {
            prev_yaw_ = yaw_;

            return;
        }


        // =================================================
        // 현재 회전량 계산
        // =================================================

        double delta_yaw =
            normalizeAngle(
                yaw_ - prev_yaw_
            );

        prev_yaw_ = yaw_;


        double abs_delta =
            std::fabs(
                delta_yaw
            );


        rotated_angle_ +=
            abs_delta;

        capture_angle_ +=
            abs_delta;


        // =================================================
        // 시작점 기준 상대 위치
        // =================================================

        double dx =
            x_ - start_x_;

        double dy =
            y_ - start_y_;


        relative_x_ =
            std::cos(start_yaw_) * dx +
            std::sin(start_yaw_) * dy;

        relative_y_ =
            -std::sin(start_yaw_) * dx +
             std::cos(start_yaw_) * dy;


        relative_yaw_ =
            normalizeAngle(
                yaw_ - start_yaw_
            );


        // =================================================
        // 일정 각도마다 저장
        // =================================================

        double interval_rad =
            capture_interval_deg_
            * M_PI
            / 180.0;


        if (
            capture_angle_ >=
            interval_rad
        )
        {
            saveData();

            capture_angle_ -=
                interval_rad;
        }
    }


    // =====================================================
    // 기준점 RESET
    // =====================================================

    void resetOdomReference()
    {
        start_x_ = x_;
        start_y_ = y_;
        start_yaw_ = yaw_;

        prev_yaw_ = yaw_;

        relative_x_ = 0.0;
        relative_y_ = 0.0;
        relative_yaw_ = 0.0;

        rotated_angle_ = 0.0;
        capture_angle_ = 0.0;

        image_count_ = 0;


        ROS_WARN("====================================");
        ROS_WARN("REFERENCE RESET");
        ROS_WARN("x   = 0.000");
        ROS_WARN("y   = 0.000");
        ROS_WARN("yaw = 0.000 deg");
        ROS_WARN("====================================");
    }


    // =====================================================
    // IMAGE + ODOM + IMU 저장
    // =====================================================

    void saveData()
    {
        if (
            latest_image_.empty()
        )
        {
            return;
        }


        std::stringstream ss;


        ss
            << save_dir_
            << "/image_"
            << std::setw(4)
            << std::setfill('0')
            << image_count_;


        std::string base =
            ss.str();


        std::string jpg_file =
            base + ".jpg";

        std::string txt_file =
            base + ".txt";


        // =================================================
        // JPG
        // =================================================

        if (
            !cv::imwrite(
                jpg_file,
                latest_image_
            )
        )
        {
            ROS_ERROR(
                "Image save failed: %s",
                jpg_file.c_str()
            );

            return;
        }


        // =================================================
        // TXT
        // =================================================

        std::ofstream file(
            txt_file
        );


        if (!file.is_open())
        {
            ROS_ERROR(
                "TXT save failed: %s",
                txt_file.c_str()
            );

            return;
        }


        file
            << std::fixed
            << std::setprecision(9);


        // =================================================
        // IMAGE
        // =================================================

        file << "[IMAGE]\n";

        file
            << "timestamp: "
            << image_stamp_.toSec()
            << "\n\n";


        // =================================================
        // ODOM
        // =================================================

        file << "[ODOMETRY]\n";

        file
            << "timestamp: "
            << odom_stamp_.toSec()
            << "\n";

        file
            << "x: "
            << relative_x_
            << "\n";

        file
            << "y: "
            << relative_y_
            << "\n";

        file
            << "yaw_rad: "
            << relative_yaw_
            << "\n";

        file
            << "yaw_deg: "
            << relative_yaw_
               * 180.0 / M_PI
            << "\n";

        file
            << "rotated_deg: "
            << rotated_angle_
               * 180.0 / M_PI
            << "\n";

        file
            << "direction: "
            << rotation_direction_
            << "\n\n";


        // =================================================
        // ACCEL
        // =================================================

        file << "[ACCEL]\n";

        file
            << "timestamp: "
            << accel_stamp_.toSec()
            << "\n";

        file
            << "x: "
            << accel_x_
            << "\n";

        file
            << "y: "
            << accel_y_
            << "\n";

        file
            << "z: "
            << accel_z_
            << "\n\n";


        // =================================================
        // GYRO
        // =================================================

        file << "[GYRO]\n";

        file
            << "timestamp: "
            << gyro_stamp_.toSec()
            << "\n";

        file
            << "x: "
            << gyro_x_
            << "\n";

        file
            << "y: "
            << gyro_y_
            << "\n";

        file
            << "z: "
            << gyro_z_
            << "\n";


        file.close();


        ROS_INFO(
            "SAVE %04d | %.1f deg | yaw %.1f | gyro_z %.4f",
            image_count_,
            rotated_angle_ * 180.0 / M_PI,
            relative_yaw_ * 180.0 / M_PI,
            gyro_z_
        );


        image_count_++;
    }


    // =====================================================
    // CMD_VEL
    // =====================================================

    void publishCmd(
        double angular
    )
    {
        geometry_msgs::Twist cmd;


        cmd.linear.x = 0.0;

        cmd.angular.z =
            angular;


        cmd_pub_.publish(
            cmd
        );
    }


    // =====================================================
    // STOP
    // =====================================================

    void stopRobot()
    {
        publishCmd(
            0.0
        );
    }


    // =====================================================
    // ROTATION
    // =====================================================

    void rotateRobot()
    {
        double target_rad =
            rotation_deg_
            * M_PI
            / 180.0;


        double command_speed;


        // right = 음수
        if (
            rotation_direction_ == "right" ||
            rotation_direction_ == "RIGHT"
        )
        {
            command_speed =
                -std::fabs(
                    angular_speed_
                );
        }

        // left = 양수
        else
        {
            command_speed =
                std::fabs(
                    angular_speed_
                );
        }


        ros::Rate rate(
            control_rate_
        );


        ROS_WARN("====================================");
        ROS_WARN("ROTATION START");
        ROS_WARN("Target    : %.1f deg", rotation_deg_);
        ROS_WARN("Direction : %s", rotation_direction_.c_str());
        ROS_WARN("Speed     : %.6f rad/s", command_speed);
        ROS_WARN("====================================");


        while (
            ros::ok() &&
            rotated_angle_ < target_rad
        )
        {
            // 모든 센서 업데이트
            ros::spinOnce();


            // 회전 명령
            publishCmd(
                command_speed
            );


            ROS_INFO_THROTTLE(
                0.5,
                "ROTATING %.1f / %.1f deg | gyro_z %.4f | cmd %.6f",
                rotated_angle_ * 180.0 / M_PI,
                rotation_deg_,
                gyro_z_,
                command_speed
            );


            rate.sleep();
        }


        // =================================================
        // 정지 명령 여러 번 전송
        // =================================================

        for (int i = 0; i < 10; i++)
        {
            stopRobot();

            ros::spinOnce();

            rate.sleep();
        }


        ROS_WARN("====================================");
        ROS_WARN("ROTATION COMPLETE");

        ROS_WARN(
            "Rotated : %.1f deg",
            rotated_angle_
            * 180.0 / M_PI
        );

        ROS_WARN(
            "Saved   : %d",
            image_count_
        );

        ROS_WARN("====================================");
    }


    // =====================================================
    // RUN
    // =====================================================

    void run()
    {
        ros::Rate rate(
            20
        );


        // =================================================
        // 모든 센서 준비될 때까지 대기
        // =================================================

        while (
            ros::ok() &&
            (
                !odom_ready_ ||
                !image_ready_ ||
                !accel_ready_ ||
                !gyro_ready_
            )
        )
        {
            ros::spinOnce();


            ROS_INFO_THROTTLE(
                1.0,
                "Waiting... ODOM:%d CAMERA:%d ACCEL:%d GYRO:%d",
                odom_ready_,
                image_ready_,
                accel_ready_,
                gyro_ready_
            );


            rate.sleep();
        }


        if (!ros::ok())
            return;


        // =================================================
        // 모든 센서 준비 완료
        // =================================================

        ROS_WARN("====================================");
        ROS_WARN("ALL SENSOR READY");
        ROS_WARN("AUTO ROTATION START");
        ROS_WARN("====================================");


        // 최신 센서 반영
        ros::spinOnce();


        // 현재 자세를 0도로 설정
        resetOdomReference();


        running_ = true;


        // 자동 회전 시작
        rotateRobot();


        running_ = false;


        stopRobot();
    }
};


// =========================================================
// MAIN
// =========================================================

int main(
    int argc,
    char** argv
)
{
    ros::init(
        argc,
        argv,
        "odom_rotate_capture"
    );


    OdomRotateCapture node;


    node.run();


    return 0;
}
