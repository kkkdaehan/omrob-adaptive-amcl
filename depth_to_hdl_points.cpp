#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

class DepthToHdlPoints
{
public:
  DepthToHdlPoints()
    : nh_(),
      pnh_("~"),
      has_camera_info_(false),
      fx_(0.0),
      fy_(0.0),
      cx_(0.0),
      cy_(0.0)
  {
    pnh_.param<std::string>("depth_topic",
                            depth_topic_,
                            "/camera/depth/image_rect_raw");

    pnh_.param<std::string>("camera_info_topic",
                            camera_info_topic_,
                            "/camera/depth/camera_info");

    pnh_.param<std::string>("output_topic",
                            output_topic_,
                            "/hdl_input_points");

    pnh_.param<double>("min_depth", min_depth_, 0.20);
    pnh_.param<double>("max_depth", max_depth_, 6.00);

    // RealSense 16UC1 depth image: 기본 단위는 mm이므로 0.001을 곱해 m로 변환
    pnh_.param<double>("depth_scale", depth_scale_, 0.001);

    // 1: 모든 픽셀 사용, 2: 가로/세로 2픽셀마다 사용, 3: 3픽셀마다 사용
    pnh_.param<int>("pixel_step", pixel_step_, 2);
    pixel_step_ = std::max(1, pixel_step_);

    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    camera_info_sub_ = nh_.subscribe(camera_info_topic_,
                                     1,
                                     &DepthToHdlPoints::cameraInfoCallback,
                                     this);

    depth_sub_ = nh_.subscribe(depth_topic_,
                               2,
                               &DepthToHdlPoints::depthCallback,
                               this);

    ROS_INFO_STREAM("depth_to_hdl_points node started");
    ROS_INFO_STREAM("  depth_topic       : " << depth_topic_);
    ROS_INFO_STREAM("  camera_info_topic : " << camera_info_topic_);
    ROS_INFO_STREAM("  output_topic      : " << output_topic_);
    ROS_INFO_STREAM("  depth range       : " << min_depth_ << " ~ " << max_depth_ << " m");
    ROS_INFO_STREAM("  pixel_step        : " << pixel_step_);
    ROS_INFO_STREAM("  depth_scale       : " << depth_scale_);
  }

private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /*
      image_rect_raw는 보정된 영상이므로 P 행렬을 우선 사용합니다.
      P 값이 비정상인 경우 K 행렬을 사용합니다.
    */
    if (msg->P[0] > 0.0 && msg->P[5] > 0.0)
    {
      fx_ = msg->P[0];
      fy_ = msg->P[5];
      cx_ = msg->P[2];
      cy_ = msg->P[6];
    }
    else if (msg->K[0] > 0.0 && msg->K[4] > 0.0)
    {
      fx_ = msg->K[0];
      fy_ = msg->K[4];
      cx_ = msg->K[2];
      cy_ = msg->K[5];
    }
    else
    {
      ROS_WARN_THROTTLE(2.0, "CameraInfo intrinsic parameters are invalid.");
      has_camera_info_ = false;
      return;
    }

    if (!has_camera_info_)
    {
      ROS_INFO_STREAM("CameraInfo received.");
      ROS_INFO_STREAM("  frame_id : " << msg->header.frame_id);
      ROS_INFO_STREAM("  fx fy    : " << fx_ << ", " << fy_);
      ROS_INFO_STREAM("  cx cy    : " << cx_ << ", " << cy_);
    }

    has_camera_info_ = true;
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    if (!has_camera_info_)
    {
      ROS_WARN_THROTTLE(2.0, "Waiting for /camera/depth/camera_info ...");
      return;
    }

    const bool is_16uc1 =
        depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth_msg->encoding == sensor_msgs::image_encodings::MONO16;

    const bool is_32fc1 =
        depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;

    if (!is_16uc1 && !is_32fc1)
    {
      ROS_ERROR_THROTTLE(2.0,
                         "Unsupported depth encoding: %s. Expected 16UC1 or 32FC1.",
                         depth_msg->encoding.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr cv_depth;

    try
    {
      cv_depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR_THROTTLE(2.0, "cv_bridge exception: %s", e.what());
      return;
    }

    const int width = static_cast<int>(depth_msg->width);
    const int height = static_cast<int>(depth_msg->height);

    std::vector<std::array<float, 4>> valid_points;
    valid_points.reserve((width / pixel_step_) * (height / pixel_step_));

    for (int v = 0; v < height; v += pixel_step_)
    {
      for (int u = 0; u < width; u += pixel_step_)
      {
        float z = 0.0f;

        if (is_16uc1)
        {
          const uint16_t raw_depth = cv_depth->image.at<uint16_t>(v, u);

          if (raw_depth == 0)
          {
            continue;
          }

          z = static_cast<float>(raw_depth * depth_scale_);
        }
        else
        {
          z = cv_depth->image.at<float>(v, u);
        }

        if (!std::isfinite(z) ||
            z < static_cast<float>(min_depth_) ||
            z > static_cast<float>(max_depth_))
        {
          continue;
        }

        /*
          Depth optical frame 좌표계:
            X: 영상 오른쪽 방향
            Y: 영상 아래쪽 방향
            Z: 카메라 전방 방향
        */
        const float x =
            static_cast<float>((static_cast<double>(u) - cx_) * z / fx_);

        const float y =
            static_cast<float>((static_cast<double>(v) - cy_) * z / fy_);

        // hdl_graph_slam은 PointXYZI를 사용하므로 intensity 필드를 포함합니다.
        // Depth 카메라에는 LiDAR 반사강도 값이 없으므로 0.0으로 둡니다.
        const float intensity = 0.0f;

        valid_points.push_back({{x, y, z, intensity}});
      }
    }

    if (valid_points.empty())
    {
      ROS_WARN_THROTTLE(2.0, "No valid depth points were generated.");
      return;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = depth_msg->header;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

    modifier.setPointCloud2Fields(
        4,
        "x",         1, sensor_msgs::PointField::FLOAT32,
        "y",         1, sensor_msgs::PointField::FLOAT32,
        "z",         1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);

    modifier.resize(valid_points.size());

    cloud_msg.height = 1;
    cloud_msg.width = valid_points.size();
    cloud_msg.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

    for (const auto& point : valid_points)
    {
      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];
      *iter_intensity = point[3];

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }

    points_pub_.publish(cloud_msg);

    ROS_INFO_THROTTLE(2.0,
                      "Published %zu points to %s, frame_id=%s",
                      valid_points.size(),
                      output_topic_.c_str(),
                      cloud_msg.header.frame_id.c_str());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber depth_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher points_pub_;

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;

  double min_depth_;
  double max_depth_;
  double depth_scale_;

  int pixel_step_;

  bool has_camera_info_;

  double fx_;
  double fy_;
  double cx_;
  double cy_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_to_hdl_points");

  DepthToHdlPoints node;

  ros::spin();

  return 0;
}
