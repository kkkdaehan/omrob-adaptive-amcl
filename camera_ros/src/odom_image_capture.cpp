#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>


class OdomImageCapture
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    ros::Subscriber odom_sub_, image_sub_;
    ros::Publisher cmd_pub_;
    ros::ServiceServer start_srv_;


    // =====================================================
    // Launch 파라미터
    // =====================================================

    std::string odom_topic_;
    std::string image_topic_;
    std::string cmd_topic_;
    std::string save_dir_;

    double linear_speed_;          // 직진/후진 속도 [m/s]
    double distance_cm_;           // 편도 이동거리 [cm]
    double capture_interval_cm_;   // 이미지 저장 간격 [cm]
    double control_rate_;          // cmd_vel publish [Hz]
    double pause_sec_;             // 전진 후 후진 전 정지시간 [sec]


    // =====================================================
    // 실제 /odom
    // =====================================================

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    double prev_x_ = 0.0;
    double prev_y_ = 0.0;

    ros::Time odom_stamp_;


    // =====================================================
    // 서비스 호출 순간의 시작 위치
    // =====================================================

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;


    // =====================================================
    // 시작점 기준 상대 odom
    // =====================================================

    double relative_x_ = 0.0;
    double relative_y_ = 0.0;
    double relative_yaw_ = 0.0;


    // =====================================================
    // 거리
    // =====================================================

    // 현재 전진 또는 후진 구간에서 이동한 거리
    double leg_distance_ = 0.0;

    // 전진 + 후진 전체 실제 이동거리
    double total_path_distance_ = 0.0;

    // 마지막 이미지 저장 이후 이동거리
    double capture_distance_ = 0.0;


    // =====================================================
    // 이미지
    // =====================================================

    cv::Mat latest_image_;
    ros::Time image_stamp_;

    int image_count_ = 0;


    // =====================================================
    // 상태
    // =====================================================

    bool odom_ready_ = false;
    bool image_ready_ = false;
    bool start_requested_ = false;
    bool running_ = false;

    std::string direction_ = "STOP";


public:

    OdomImageCapture()
    {
        // =============================
        // Topic
        // =============================

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
            "cmd_vel_topic",
            cmd_topic_,
            "/cmd_vel"
        );


        // =============================
        // 저장 위치
        // =============================

        pnh_.param<std::string>(
            "save_dir",
            save_dir_,
            "/home/omrob/catkin_ws/src/camera_ros/src/연구실"
        );


        // =============================
        // 이동 설정
        // =============================

        pnh_.param(
            "linear_speed",
            linear_speed_,
            0.0926510094425921
        );

        // 편도 이동거리 [cm]
        // 100이면 1m 전진 후 1m 후진
        pnh_.param(
            "distance_cm",
            distance_cm_,
            100.0
        );

        // 몇 cm마다 이미지 + odom 저장
        pnh_.param(
            "capture_interval_cm",
            capture_interval_cm_,
            20.0
        );

        pnh_.param(
            "control_rate",
            control_rate_,
            20.0
        );

        // 전진 완료 후 후진하기 전에 기다리는 시간
        pnh_.param(
            "pause_sec",
            pause_sec_,
            1.0
        );


        // =============================
        // ROS
        // =============================

        odom_sub_ = nh_.subscribe(
            odom_topic_,
            20,
            &OdomImageCapture::odomCallback,
            this
        );

        image_sub_ = nh_.subscribe(
            image_topic_,
            1,
            &OdomImageCapture::imageCallback,
            this
        );

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(
            cmd_topic_,
            10
        );

        // /odom_image_capture/start
        start_srv_ = pnh_.advertiseService(
            "start",
            &OdomImageCapture::startCallback,
            this
        );


        ROS_INFO("====================================");
        ROS_INFO("FORWARD -> BACKWARD CAPTURE");
        ROS_INFO("Distance      : %.1f cm each way", distance_cm_);
        ROS_INFO("Speed         : %.6f m/s", linear_speed_);
        ROS_INFO("Capture       : %.1f cm", capture_interval_cm_);
        ROS_INFO("Pause         : %.1f sec", pause_sec_);
        ROS_INFO("Save dir      : %s", save_dir_.c_str());
        ROS_INFO("====================================");
    }


    ~OdomImageCapture()
    {
        stopRobot();
    }


    // =====================================================
    // Quaternion -> Yaw
    // =====================================================

    double getYaw(const geometry_msgs::Quaternion& q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
    }


    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }


    // =====================================================
    // Camera callback
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

            image_stamp_ = msg->header.stamp;
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
    // Odom callback
    // =====================================================

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        yaw_ =
            getYaw(msg->pose.pose.orientation);

        odom_stamp_ =
            msg->header.stamp;


        // 최초 odom
        if (!odom_ready_)
        {
            prev_x_ = x_;
            prev_y_ = y_;

            odom_ready_ = true;

            ROS_INFO("ODOM READY");

            return;
        }


        // 주행 시작 전
        if (!running_)
        {
            prev_x_ = x_;
            prev_y_ = y_;

            return;
        }


        // =============================
        // 이동거리 계산
        // =============================

        double dx = x_ - prev_x_;
        double dy = y_ - prev_y_;

        double delta_distance =
            std::sqrt(
                dx * dx +
                dy * dy
            );

        prev_x_ = x_;
        prev_y_ = y_;


        // 현재 구간 이동거리
        leg_distance_ += delta_distance;

        // 전체 경로 이동거리
        total_path_distance_ += delta_distance;

        // 이미지 촬영용
        capture_distance_ += delta_distance;


        // =============================
        // 시작점 기준 상대 odom
        // =============================

        double dx_start =
            x_ - start_x_;

        double dy_start =
            y_ - start_y_;

        relative_x_ =
            std::cos(start_yaw_) * dx_start +
            std::sin(start_yaw_) * dy_start;

        relative_y_ =
            -std::sin(start_yaw_) * dx_start +
             std::cos(start_yaw_) * dy_start;

        relative_yaw_ =
            normalizeAngle(
                yaw_ - start_yaw_
            );


        // =============================
        // 일정 거리마다 저장
        // =============================

        double interval_m =
            capture_interval_cm_ / 100.0;


        if (capture_distance_ >= interval_m)
        {
            saveData();

            // 0으로 만들기보다 초과분을 남겨서
            // 촬영 간격 오차 누적을 줄임
            capture_distance_ -= interval_m;
        }
    }


    // =====================================================
    // Start service
    // =====================================================

    bool startCallback(
        std_srvs::Trigger::Request& req,
        std_srvs::Trigger::Response& res
    )
    {
        if (!odom_ready_)
        {
            res.success = false;
            res.message = "ODOM not ready";
            return true;
        }

        if (!image_ready_)
        {
            res.success = false;
            res.message = "CAMERA not ready";
            return true;
        }

        if (running_ || start_requested_)
        {
            res.success = false;
            res.message = "Already running";
            return true;
        }


        start_requested_ = true;

        res.success = true;
        res.message = "START requested";

        ROS_WARN("START SERVICE RECEIVED");

        return true;
    }


    // =====================================================
    // 서비스 호출 순간을 (0,0,0)으로 설정
    // =====================================================

    void resetOdomReference()
    {
        start_x_ = x_;
        start_y_ = y_;
        start_yaw_ = yaw_;

        prev_x_ = x_;
        prev_y_ = y_;

        relative_x_ = 0.0;
        relative_y_ = 0.0;
        relative_yaw_ = 0.0;

        leg_distance_ = 0.0;
        total_path_distance_ = 0.0;
        capture_distance_ = 0.0;

        image_count_ = 0;


        ROS_WARN("====================================");
        ROS_WARN("ODOM REFERENCE RESET");
        ROS_WARN("x   = 0.000");
        ROS_WARN("y   = 0.000");
        ROS_WARN("yaw = 0.000");
        ROS_WARN("====================================");
    }


    // =====================================================
    // 이미지 + odom TXT 저장
    // =====================================================

    void saveData()
    {
        if (!image_ready_ || latest_image_.empty())
            return;


        // image_0000 공통 파일 이름
        std::stringstream ss;

        ss << save_dir_
           << "/image_"
           << std::setw(4)
           << std::setfill('0')
           << image_count_;


        std::string base = ss.str();

        std::string jpg_file =
            base + ".jpg";

        std::string txt_file =
            base + ".txt";


        // =============================
        // 이미지 저장
        // =============================

        if (!cv::imwrite(
                jpg_file,
                latest_image_
            ))
        {
            ROS_ERROR(
                "Image save failed: %s",
                jpg_file.c_str()
            );

            return;
        }


        // =============================
        // Odom TXT 저장
        // =============================

        std::ofstream file(txt_file);

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


        file << "[IMAGE]\n";

        file << "timestamp: "
             << image_stamp_.toSec()
             << "\n\n";


        file << "[ODOMETRY]\n";

        // 현재 전진/후진 구분
        file << "direction: "
             << direction_
             << "\n";

        file << "timestamp: "
             << odom_stamp_.toSec()
             << "\n";

        file << "x: "
             << relative_x_
             << "\n";

        file << "y: "
             << relative_y_
             << "\n";

        file << "yaw_rad: "
             << relative_yaw_
             << "\n";

        file << "yaw_deg: "
             << relative_yaw_ * 180.0 / M_PI
             << "\n";

        // 현재 전진/후진 구간 이동량
        file << "leg_distance_cm: "
             << leg_distance_ * 100.0
             << "\n";

        // 전진 + 후진 전체 이동량
        file << "total_path_distance_cm: "
             << total_path_distance_ * 100.0
             << "\n";


        file.close();


        ROS_INFO(
            "SAVE %04d | %s | leg %.1f cm | x %.3f y %.3f",
            image_count_,
            direction_.c_str(),
            leg_distance_ * 100.0,
            relative_x_,
            relative_y_
        );


        image_count_++;
    }


    // =====================================================
    // cmd_vel
    // =====================================================

    void publishCmd(double speed)
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = speed;
        cmd.angular.z = 0.0;

        cmd_pub_.publish(cmd);
    }


    void stopRobot()
    {
        publishCmd(0.0);
    }


    // =====================================================
    // 지정 거리 이동
    //
    // forward = true  -> 전진
    // forward = false -> 후진
    // =====================================================

    void moveDistance(bool forward)
    {
        double target_m =
            distance_cm_ / 100.0;

        // 각 구간 시작 시 초기화
        leg_distance_ = 0.0;
        capture_distance_ = 0.0;

        direction_ =
            forward ? "FORWARD" : "BACKWARD";

        double speed =
            forward
            ? linear_speed_
            : -linear_speed_;

        ros::Rate rate(control_rate_);


        ROS_WARN("====================================");
        ROS_WARN("%s START", direction_.c_str());
        ROS_WARN("Target : %.1f cm", distance_cm_);
        ROS_WARN("Speed  : %.6f m/s", speed);
        ROS_WARN("====================================");


        while (ros::ok())
        {
            // 최신 odom / camera 처리
            ros::spinOnce();


            // 목표 이동거리 도달
            if (leg_distance_ >= target_m)
                break;


            // 전진 또는 후진 명령
            publishCmd(speed);


            ROS_INFO_THROTTLE(
                0.5,
                "%s | %.1f / %.1f cm | cmd %.6f",
                direction_.c_str(),
                leg_distance_ * 100.0,
                distance_cm_,
                speed
            );


            rate.sleep();
        }


        // 확실하게 정지
        for (int i = 0; i < 10; i++)
        {
            stopRobot();

            ros::spinOnce();

            rate.sleep();
        }


        ROS_WARN(
            "%s COMPLETE | %.1f cm",
            direction_.c_str(),
            leg_distance_ * 100.0
        );
    }


    // =====================================================
    // 전진 후 후진 사이 정지
    // =====================================================

    void pauseRobot()
    {
        direction_ = "STOP";

        ros::Rate rate(control_rate_);

        ros::Time end =
            ros::Time::now() +
            ros::Duration(pause_sec_);


        ROS_WARN(
            "PAUSE %.1f sec",
            pause_sec_
        );


        while (
            ros::ok() &&
            ros::Time::now() < end
        )
        {
            stopRobot();

            ros::spinOnce();

            rate.sleep();
        }
    }


    // =====================================================
    // Main
    // =====================================================

    void run()
    {
        ros::Rate rate(20);


        // Odom / Camera 준비
        while (
            ros::ok() &&
            (!odom_ready_ || !image_ready_)
        )
        {
            ros::spinOnce();

            ROS_INFO_THROTTLE(
                1.0,
                "Waiting ODOM / CAMERA..."
            );

            rate.sleep();
        }


        ROS_INFO("CAMERA READY");
        ROS_INFO("====================================");
        ROS_INFO("WAITING START SERVICE");
        ROS_INFO("rosservice call /odom_image_capture/start");
        ROS_INFO("====================================");


        // 서비스 호출 대기
        while (
            ros::ok() &&
            !start_requested_
        )
        {
            ros::spinOnce();
            rate.sleep();
        }


        if (!ros::ok())
            return;


        // 최신 odom 반영
        ros::spinOnce();


        // 서비스 호출한 위치를 (0,0,0)
        resetOdomReference();


        running_ = true;


        // =============================
        // 1. 전진
        // =============================
        moveDistance(true);


        // =============================
        // 2. 잠깐 정지
        // =============================
        pauseRobot();


        // 후진 전에 현재 위치를 previous로 다시 맞춤
        prev_x_ = x_;
        prev_y_ = y_;


        // =============================
        // 3. 같은 거리 후진
        // =============================
        moveDistance(false);


        running_ = false;

        direction_ = "STOP";

        stopRobot();


        ROS_WARN("====================================");
        ROS_WARN("ALL COMPLETE");
        ROS_WARN("Forward  : %.1f cm", distance_cm_);
        ROS_WARN("Backward : %.1f cm", distance_cm_);
        ROS_WARN("Final relative x : %.3f m", relative_x_);
        ROS_WARN("Final relative y : %.3f m", relative_y_);
        ROS_WARN("Saved data : %d", image_count_);
        ROS_WARN("====================================");
    }
};


int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "odom_image_capture"
    );

    OdomImageCapture node;

    node.run();

    return 0;
}
