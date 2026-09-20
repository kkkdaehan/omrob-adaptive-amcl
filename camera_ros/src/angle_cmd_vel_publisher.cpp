#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

class RotateAngle
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    bool initialized_;

    // yaw 관련
    double prev_yaw_;
    double accumulated_angle_;

    // 파라미터
    double angular_speed_;
    double target_angle_rad_;
    int direction_; // 1 or -1

public:
    RotateAngle() : pnh_("~"), initialized_(false)
    {
        double target_angle_degree;

        pnh_.param("angular_speed", angular_speed_, 0.3);
        pnh_.param("target_angle_degree", target_angle_degree, 90.0);
        pnh_.param("direction", direction_, 1);

        // degree → radian
        target_angle_rad_ = target_angle_degree * M_PI / 180.0;

        odom_sub_ = nh_.subscribe("/odom", 10, &RotateAngle::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        ROS_INFO("angular_speed: %f", angular_speed_);
        ROS_INFO("target_angle(deg): %f", target_angle_degree);
        ROS_INFO("direction (1:CW, -1:CCW): %d", direction_);
    }

    double getYaw(const geometry_msgs::Quaternion& q)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(q, quat);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    // -pi ~ pi 정규화
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double current_yaw = getYaw(msg->pose.pose.orientation);

        if (!initialized_)
        {
            prev_yaw_ = current_yaw;
            accumulated_angle_ = 0.0;
            initialized_ = true;
            ROS_INFO("Rotation start");
            return;
        }

        // yaw 변화량
        double delta = normalizeAngle(current_yaw - prev_yaw_);

        // 누적 회전량
        accumulated_angle_ += fabs(delta);

        prev_yaw_ = current_yaw;

        // 남은 각도 계산
        double remaining_angle = target_angle_rad_ - accumulated_angle_;

        geometry_msgs::Twist cmd;

        if (remaining_angle > 0.0)
        {
            cmd.angular.z = direction_ * fabs(angular_speed_);
        }
        else
        {
            cmd.angular.z = 0.0;
            ROS_INFO("Reached target angle. Stopping.");
        }

        cmd_pub_.publish(cmd);

        // 🔥 로그 출력 (degree로 보기 쉽게)
        ROS_INFO("Rotated: %.2f deg | Remaining: %.2f deg",
                 accumulated_angle_ * 180.0 / M_PI,
                 std::max(0.0, remaining_angle) * 180.0 / M_PI);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_angle_node");
    RotateAngle node;
    ros::spin();
    return 0;
}
