#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h> // 서비스 호출용 (로봇에 따라 다를 수 있음)
#include <cmath>

class MoveDistance {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    double start_x_, start_y_;
    bool initialized_;
    double linear_speed_, angular_speed_, target_distance_;

public:
    MoveDistance() : pnh_("~"), initialized_(false) {
        pnh_.param("linear_speed", linear_speed_, 0.2);
        pnh_.param("angular_speed", angular_speed_, 0.0);
        pnh_.param("target_distance", target_distance_, 1.0);

        // 1. Odom 초기화 서비스 호출 (선택 사항: 서비스 이름 확인 필요)
        // 예: /reset_odometry 또는 /mobile_base/commands/reset_odometry
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/reset_odometry");
        std_srvs::Empty srv;
        if (client.call(srv)) {
            ROS_INFO("Odometry reset successful.");
        } else {
            ROS_WARN("Failed to call reset_odometry service. Using current position as start.");
        }

        odom_sub_ = nh_.subscribe("/odom", 10, &MoveDistance::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double curr_x = msg->pose.pose.position.x;
        double curr_y = msg->pose.pose.position.y;

        if (!initialized_) {
            start_x_ = curr_x;
            start_y_ = curr_y;
            initialized_ = true;
            return;
        }

        // 2. 유클리드 거리 계산 (x, y 모두 고려하여 실제 이동거리 측정)
        double moved_dist = std::sqrt(std::pow(curr_x - start_x_, 2) + std::pow(curr_y - start_y_, 2));
        double remaining = target_distance_ - moved_dist;

        geometry_msgs::Twist cmd;
        if (remaining > 0.01) { // 약간의 오차 허용
            cmd.linear.x = linear_speed_;
            cmd.angular.z = angular_speed_;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            ROS_INFO_ONCE("--- Target Reached! Stopping ---");
        }

        cmd_pub_.publish(cmd);

        ROS_INFO("Moved: %.3f m | Remaining: %.3f m", moved_dist, std::max(0.0, remaining));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_distance_node");
    MoveDistance node;
    ros::spin();
    return 0;
}
