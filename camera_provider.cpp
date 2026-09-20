#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_camera_provider");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub_left  = it.advertise("camera_left/image_raw", 1);
    image_transport::Publisher pub_right = it.advertise("camera_right/image_raw", 1);

    cv::VideoCapture cap_left("/dev/cam_left",   cv::CAP_V4L2);
    cv::VideoCapture cap_right("/dev/cam_right", cv::CAP_V4L2);

    if (!cap_left.isOpened() || !cap_right.isOpened()) {
        std::cerr << "[Error] 카메라 오픈 실패!" << std::endl;
        return -1;
    }

    int width  = 640;
    int height = 480;
    int fps    = 15;

    cap_left.set(cv::CAP_PROP_FOURCC,       cv::VideoWriter::fourcc('M','J','P','G'));
    cap_left.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap_left.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_left.set(cv::CAP_PROP_FPS,          fps);

    cap_right.set(cv::CAP_PROP_FOURCC,      cv::VideoWriter::fourcc('M','J','P','G'));
    cap_right.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap_right.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_right.set(cv::CAP_PROP_FPS,          fps);

    cv::Mat frame_l, frame_r;
    ros::Rate loop_rate(fps);

    ROS_INFO("Camera provider started (headless mode)");

    while (ros::ok()) {
        if (cap_left.read(frame_l) && cap_right.read(frame_r)) {
            if (!frame_l.empty() && !frame_r.empty()) {
                ros::Time now = ros::Time::now();

                std_msgs::Header h_l;
                h_l.stamp    = now;
                h_l.frame_id = "left_camera_link";
                pub_left.publish(cv_bridge::CvImage(h_l, "bgr8", frame_l).toImageMsg());

                std_msgs::Header h_r;
                h_r.stamp    = now;
                h_r.frame_id = "right_camera_link";
                pub_right.publish(cv_bridge::CvImage(h_r, "bgr8", frame_r).toImageMsg());

                ROS_INFO_THROTTLE(1, "Images are being published to ROS topics...");
            }
        } else {
            ROS_WARN_THROTTLE(1, "Failed to read frames from cameras.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}