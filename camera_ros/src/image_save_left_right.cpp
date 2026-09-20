#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <sstream>
#include <iomanip>

class RgbImageSaver
{
public:
  RgbImageSaver()
    : nh_("~"),
      saved_count_(0),
      frame_count_(0)
  {
    nh_.param<std::string>(
      "output_dir",
      output_dir_,
      "/home/omrob/catkin_ws/src/camera_ros/src/saved_data/images/rgb"
    );

    // 몇 프레임마다 이미지를 저장할지 설정
    nh_.param<int>("save_every_n", save_every_n_, 1);

    if (save_every_n_ < 1)
    {
      ROS_WARN("save_every_n must be greater than 0. Set to 1.");
      save_every_n_ = 1;
    }

    if (!createDirectoryRecursive(output_dir_))
    {
      ROS_ERROR("Failed to prepare output directory: %s",
                output_dir_.c_str());
    }

    color_sub_ = nh_.subscribe(
      "/camera/color/image_raw",
      1,
      &RgbImageSaver::colorCallback,
      this
    );

    ROS_INFO("RGB image saver started");
    ROS_INFO("Subscribe topic : /camera/color/image_raw");
    ROS_INFO("Save directory  : %s", output_dir_.c_str());
    ROS_INFO("save_every_n    : %d", save_every_n_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber color_sub_;

  std::string output_dir_;

  int saved_count_;
  int frame_count_;
  int save_every_n_;

  bool createDirectoryRecursive(const std::string& path)
  {
    if (path.empty())
    {
      return false;
    }

    std::string current_path;

    if (path[0] == '/')
    {
      current_path = "/";
    }

    std::stringstream ss(path);
    std::string folder;

    while (std::getline(ss, folder, '/'))
    {
      if (folder.empty())
      {
        continue;
      }

      if (current_path.size() > 1)
      {
        current_path += "/";
      }

      current_path += folder;

      struct stat st;

      if (stat(current_path.c_str(), &st) != 0)
      {
        if (mkdir(current_path.c_str(), 0755) != 0)
        {
          ROS_ERROR("Failed to create directory: %s",
                    current_path.c_str());
          return false;
        }
      }
      else if (!S_ISDIR(st.st_mode))
      {
        ROS_ERROR("Path exists but is not a directory: %s",
                  current_path.c_str());
        return false;
      }
    }

    return true;
  }

  std::string stampToString(const ros::Time& stamp)
  {
    std::stringstream ss;

    ss << stamp.sec
       << "."
       << std::setw(9)
       << std::setfill('0')
       << stamp.nsec;

    return ss.str();
  }

  void colorCallback(const sensor_msgs::ImageConstPtr& color_msg)
  {
    // save_every_n 프레임마다 저장
    if (frame_count_ % save_every_n_ != 0)
    {
      frame_count_++;
      return;
    }

    frame_count_++;

    cv_bridge::CvImageConstPtr color_cv;

    try
    {
      color_cv = cv_bridge::toCvShare(
        color_msg,
        sensor_msgs::image_encodings::BGR8
      );
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge error: %s", e.what());
      return;
    }

    const cv::Mat& color_image = color_cv->image;

    if (color_image.empty())
    {
      ROS_WARN("Received an empty RGB image");
      return;
    }

    ros::Time stamp = color_msg->header.stamp;

    // timestamp가 0인 경우 현재 ROS 시간 사용
    if (stamp.isZero())
    {
      stamp = ros::Time::now();
    }

    std::string timestamp = stampToString(stamp);
    std::string filename = timestamp + ".png";
    std::string save_path = output_dir_ + "/" + filename;

    bool save_result = false;

    try
    {
      save_result = cv::imwrite(save_path, color_image);
    }
    catch (const cv::Exception& e)
    {
      ROS_ERROR("OpenCV image save error: %s", e.what());
      return;
    }

    if (!save_result)
    {
      ROS_WARN("Failed to save RGB image: %s", save_path.c_str());
      return;
    }

    saved_count_++;

    ROS_INFO("Saved RGB image %d", saved_count_);
    ROS_INFO("  Timestamp : %s", timestamp.c_str());
    ROS_INFO("  Resolution: %d x %d",
             color_image.cols,
             color_image.rows);
    ROS_INFO("  Path      : %s", save_path.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_rgb_image_node");

  RgbImageSaver saver;

  ros::spin();

  return 0;
}
