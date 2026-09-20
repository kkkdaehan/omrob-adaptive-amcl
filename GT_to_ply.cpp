#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <boost/filesystem.hpp>

#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>

class RealSenseToPLY
{
public:
    RealSenseToPLY()
        : nh_(),
          pnh_("~"),
          color_sub_(nh_, "/camera/color/image_raw", 1),
          depth_sub_(nh_, "/camera/aligned_depth_to_color/image_raw", 1),
          sync_(SyncPolicy(10), color_sub_, depth_sub_)
    {
        pnh_.param<std::string>(
            "camera_info_topic",
            camera_info_topic_,
            "/camera/aligned_depth_to_color/camera_info"
        );

        pnh_.param<std::string>(
            "base_dir",
            base_dir_,
            "/home/omrob/catkin_ws/src/camera_ros/src"
        );

        pnh_.param<std::string>(
            "folder_name",
            folder_name_,
            "realsense_ply"
        );

        pnh_.param<std::string>(
            "file_prefix",
            file_prefix_,
            "cloud_"
        );

        pnh_.param<int>("save_interval", save_interval_, 1);
        pnh_.param<int>("stride", stride_, 1);
        pnh_.param<double>("max_depth", max_depth_, 5.0);
        pnh_.param<bool>("binary", binary_, true);

        if (stride_ < 1)
            stride_ = 1;

        if (save_interval_ < 1)
            save_interval_ = 1;

        output_dir_ = base_dir_ + "/" + folder_name_;

        if (!boost::filesystem::exists(output_dir_))
        {
            boost::filesystem::create_directories(output_dir_);
            ROS_INFO("Created output directory: %s", output_dir_.c_str());
        }
        else
        {
            ROS_INFO("Output directory already exists: %s", output_dir_.c_str());
        }

        camera_info_sub_ = nh_.subscribe(
            camera_info_topic_,
            1,
            &RealSenseToPLY::cameraInfoCallback,
            this
        );

        sync_.registerCallback(
            boost::bind(&RealSenseToPLY::imageCallback, this, _1, _2)
        );

        ROS_INFO("======================================");
        ROS_INFO("RealSense RGB-D to PLY node started.");
        ROS_INFO("Color topic      : /camera/color/image_raw");
        ROS_INFO("Depth topic      : /camera/aligned_depth_to_color/image_raw");
        ROS_INFO("CameraInfo topic : %s", camera_info_topic_.c_str());
        ROS_INFO("Base directory   : %s", base_dir_.c_str());
        ROS_INFO("Output directory : %s", output_dir_.c_str());
        ROS_INFO("File prefix      : %s", file_prefix_.c_str());
        ROS_INFO("Save interval    : %d", save_interval_);
        ROS_INFO("Stride           : %d", stride_);
        ROS_INFO("Max depth        : %.2f m", max_depth_);
        ROS_INFO("Binary PLY       : %s", binary_ ? "true" : "false");
        ROS_INFO("======================================");
    }

private:
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        camera_info_ = msg;
    }

    std::string makeOutputFileName()
    {
        std::stringstream ss;

        ss << output_dir_ << "/"
           << file_prefix_
           << std::setw(6) << std::setfill('0') << saved_count_
           << ".ply";

        return ss.str();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& color_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg)
    {
        frame_count_++;

        if (frame_count_ % save_interval_ != 0)
            return;

        if (!camera_info_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for camera_info...");
            return;
        }

        cv_bridge::CvImageConstPtr color_ptr;
        cv_bridge::CvImageConstPtr depth_ptr;

        try
        {
            color_ptr = cv_bridge::toCvShare(
                color_msg,
                sensor_msgs::image_encodings::BGR8
            );

            depth_ptr = cv_bridge::toCvShare(depth_msg);
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        const cv::Mat& color_img = color_ptr->image;
        const cv::Mat& depth_img = depth_ptr->image;

        if (color_img.empty() || depth_img.empty())
        {
            ROS_ERROR("Empty image received.");
            return;
        }

        if (color_img.cols != depth_img.cols ||
            color_img.rows != depth_img.rows)
        {
            ROS_ERROR("Color and depth image size mismatch.");
            ROS_ERROR("Color size: %d x %d", color_img.cols, color_img.rows);
            ROS_ERROR("Depth size: %d x %d", depth_img.cols, depth_img.rows);
            return;
        }

        const double fx = camera_info_->K[0];
        const double fy = camera_info_->K[4];
        const double cx = camera_info_->K[2];
        const double cy = camera_info_->K[5];

        if (fx <= 0.0 || fy <= 0.0)
        {
            ROS_ERROR("Invalid camera intrinsic parameters.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>
        );

        cloud->header.frame_id = color_msg->header.frame_id;
        cloud->is_dense = false;

        for (int v = 0; v < depth_img.rows; v += stride_)
        {
            for (int u = 0; u < depth_img.cols; u += stride_)
            {
                float z = 0.0f;

                if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                    depth_msg->encoding == "16UC1")
                {
                    uint16_t depth_value = depth_img.at<uint16_t>(v, u);

                    if (depth_value == 0)
                        continue;

                    z = static_cast<float>(depth_value) * 0.001f;  // mm -> m
                }
                else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                         depth_msg->encoding == "32FC1")
                {
                    z = depth_img.at<float>(v, u);

                    if (!std::isfinite(z))
                        continue;
                }
                else
                {
                    ROS_ERROR("Unsupported depth encoding: %s",
                              depth_msg->encoding.c_str());
                    return;
                }

                if (z <= 0.0f || z > max_depth_)
                    continue;

                pcl::PointXYZRGB point;

                point.z = z;
                point.x = static_cast<float>((u - cx) * z / fx);
                point.y = static_cast<float>((v - cy) * z / fy);

                const cv::Vec3b& bgr = color_img.at<cv::Vec3b>(v, u);

                point.b = bgr[0];
                point.g = bgr[1];
                point.r = bgr[2];

                cloud->points.push_back(point);
            }
        }

        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;

        if (cloud->points.empty())
        {
            ROS_WARN("No valid points. PLY was not saved.");
            return;
        }

        std::string output_path = makeOutputFileName();

        int result = 0;

        if (binary_)
        {
            result = pcl::io::savePLYFileBinary(output_path, *cloud);
        }
        else
        {
            result = pcl::io::savePLYFileASCII(output_path, *cloud);
        }

        if (result == 0)
        {
            ROS_INFO("Saved PLY: %s", output_path.c_str());
            ROS_INFO("Point count: %zu", cloud->points.size());

            saved_count_++;
        }
        else
        {
            ROS_ERROR("Failed to save PLY: %s", output_path.c_str());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    message_filters::Subscriber<sensor_msgs::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::Image
    > SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync_;

    ros::Subscriber camera_info_sub_;
    sensor_msgs::CameraInfoConstPtr camera_info_;

    std::string camera_info_topic_;

    std::string base_dir_;
    std::string folder_name_;
    std::string output_dir_;
    std::string file_prefix_;

    int save_interval_;
    int stride_;
    int frame_count_ = 0;
    int saved_count_ = 0;

    double max_depth_;
    bool binary_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_to_ply");

    RealSenseToPLY node;

    ros::spin();

    return 0;
}
