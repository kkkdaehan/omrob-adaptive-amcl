#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>

class RGBDRecorder {
public:
    RGBDRecorder() : count_(0), info_received_(false) {
        base_dir_ = "/home/omrob/catkin_ws/rgbd_dataset/";
        mkdir(base_dir_.c_str(), 0777);
        mkdir((base_dir_ + "rgb/").c_str(), 0777);
        mkdir((base_dir_ + "depth/").c_str(), 0777);
        mkdir((base_dir_ + "ply/").c_str(), 0777);

        assoc_file_.open(base_dir_ + "associate.txt", std::ios::out);
        assoc_file_ << "# timestamp filename_rgb timestamp filename_depth\n";

        // 카메라 내부 파라미터 토픽 구독 (한 번 받으면 파라미터 갱신됨)
        info_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &RGBDRecorder::infoCallback, this);

        // 메시지 필터 싱크로나이저 설정 (동기화)
        color_sub_.subscribe(nh_, "/camera/color/image_raw", 1);
        depth_sub_.subscribe(nh_, "/camera/aligned_depth_to_color/image_raw", 1);

        sync_.reset(new Sync(SyncPolicy(10), color_sub_, depth_sub_));
        sync_->registerCallback(boost::bind(&RGBDRecorder::callback, this, _1, _2));

        ROS_INFO("RGB-D Recorder Node Initialized. Waiting for CameraInfo and Data...");
    }

    ~RGBDRecorder() {
        if (assoc_file_.is_open()) {
            assoc_file_.close();
        }
        ROS_INFO("RGB-D Recorder Terminated. Total frames saved: %d", count_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber info_sub_;
    message_filters::Subscriber<sensor_msgs::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    std::ofstream assoc_file_;
    std::string base_dir_;
    int count_;
    bool info_received_;

    // 카메라 내부 파라미터 (K 매트릭스에서 동적 추출)
    double fx_, fy_, cx_, cy_;

    void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
        // K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx_ = info_msg->K[0];
        fy_ = info_msg->K[4];
        cx_ = info_msg->K[2];
        cy_ = info_msg->K[5];

        if (!info_received_) {
            ROS_INFO("Camera Intrinsics Received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
            info_received_ = true;
        }
    }

    void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg) {
        if (!info_received_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for camera_info topic...");
            return;
        }

        try {
            double timestamp = color_msg->header.stamp.toSec();
            std::stringstream ss_time;
            ss_time << std::fixed << std::setprecision(6) << timestamp;
            std::string time_str = ss_time.str();

            std::string rgb_filename = "rgb/" + time_str + ".png";
            std::string depth_filename = "depth/" + time_str + ".png";
            std::string ply_filename = base_dir_ + "ply/" + time_str + ".ply";

            cv_bridge::CvImageConstPtr color_ptr = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

            cv::imwrite(base_dir_ + rgb_filename, color_ptr->image);
            cv::imwrite(base_dir_ + depth_filename, depth_ptr->image);

            assoc_file_ << time_str << " " << rgb_filename << " " << time_str << " " << depth_filename << "\n";

            savePointCloudAsPLY(color_ptr->image, depth_ptr->image, ply_filename);

            count_++;
            if (count_ % 30 == 0) {
                ROS_INFO("Saved %d frames...", count_);
            }

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void savePointCloudAsPLY(const cv::Mat& color, const cv::Mat& depth, const std::string& filename) {
        std::ofstream ply_file(filename);
        if (!ply_file.is_open()) return;

        std::vector<std::string> points_buffer;
        int valid_points = 0;

        for (int v = 0; v < depth.rows; v += 2) {
            for (int u = 0; u < depth.cols; u += 2) {
                unsigned short d = depth.at<unsigned short>(v, u);
                if (d == 0 || d > 10000) continue;

                double z = d / 1000.0;
                double x = (u - cx_) * z / fx_;
                double y = (v - cy_) * z / fy_;

                cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);

                std::stringstream ss;
                ss << x << " " << y << " " << z << " "
                   << (int)bgr[2] << " " << (int)bgr[1] << " " << (int)bgr[0] << "\n";
                points_buffer.push_back(ss.str());
                valid_points++;
            }
        }

        ply_file << "ply\n";
        ply_file << "format ascii 1.0\n";
        ply_file << "element vertex " << valid_points << "\n";
        ply_file << "property float x\n";
        ply_file << "property float y\n";
        ply_file << "property float z\n";
        ply_file << "property uchar red\n";
        ply_file << "property uchar green\n";
        ply_file << "property uchar blue\n";
        ply_file << "end_header\n";

        for (const auto& p : points_buffer) {
            ply_file << p;
        }
        ply_file.close();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgbd_recorder_node");
    RGBDRecorder recorder;
    ros::spin();
    return 0;
}
