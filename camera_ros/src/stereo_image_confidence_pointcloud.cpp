#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <memory>
#include <iomanip>
#include <sstream>
#include <string>


class StereoDataSaver
{
public:

    // =========================================================
    // 동기화:
    // 1. Left Image
    // 2. Confidence Map
    // 3. Organized PointCloud
    // =========================================================
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::Image,
        sensor_msgs::PointCloud2
    > SyncPolicy;

    typedef message_filters::Synchronizer<SyncPolicy>
        Synchronizer;


    StereoDataSaver()
        : nh_("~"),
          frame_count_(0)
    {
        // =====================================================
        // 실제 사용하는 토픽
        // =====================================================

        nh_.param<std::string>(
            "image_topic",
            image_topic_,
            "/zed/zed_node/left/image_rect_color"
        );

        nh_.param<std::string>(
            "confidence_topic",
            confidence_topic_,
            "/zed/zed_node/confidence/confidence_map"
        );

        nh_.param<std::string>(
            "pointcloud_topic",
            pointcloud_topic_,
            "/zed/zed_node/point_cloud/cloud_registered"
        );


        // =====================================================
        // 저장 기본 경로
        // =====================================================

        nh_.param<std::string>(
            "save_dir",
            save_dir_,
            "/home/omrob/catkin_ws/src/camera_ros/src/stereo"
        );


        image_dir_ =
            save_dir_ + "/image";

        confidence_dir_ =
            save_dir_ + "/confidence";

        pointcloud_dir_ =
            save_dir_ + "/pointcloud";


        // =====================================================
        // 폴더 없으면 자동 생성
        // =====================================================

        boost::filesystem::create_directories(
            image_dir_);

        boost::filesystem::create_directories(
            confidence_dir_);

        boost::filesystem::create_directories(
            pointcloud_dir_);


        // =====================================================
        // Subscriber
        // =====================================================

        image_sub_.subscribe(
            nh_,
            image_topic_,
            5
        );

        confidence_sub_.subscribe(
            nh_,
            confidence_topic_,
            5
        );

        pointcloud_sub_.subscribe(
            nh_,
            pointcloud_topic_,
            5
        );


        // =====================================================
        // Approximate Time Synchronizer
        // =====================================================

        sync_.reset(
            new Synchronizer(
                SyncPolicy(30),
                image_sub_,
                confidence_sub_,
                pointcloud_sub_
            )
        );


        sync_->registerCallback(
            boost::bind(
                &StereoDataSaver::callback,
                this,
                _1,
                _2,
                _3
            )
        );


        ROS_INFO("================================================");
        ROS_INFO(" ZED Image / Confidence / PointCloud Saver");
        ROS_INFO("================================================");

        ROS_INFO(
            "IMAGE      : %s",
            image_topic_.c_str()
        );

        ROS_INFO(
            "CONFIDENCE : %s",
            confidence_topic_.c_str()
        );

        ROS_INFO(
            "POINTCLOUD : %s",
            pointcloud_topic_.c_str()
        );

        ROS_INFO("------------------------------------------------");

        ROS_INFO(
            "SAVE ROOT  : %s",
            save_dir_.c_str()
        );

        ROS_INFO(
            "IMAGE      : %s",
            image_dir_.c_str()
        );

        ROS_INFO(
            "CONFIDENCE : %s",
            confidence_dir_.c_str()
        );

        ROS_INFO(
            "POINTCLOUD : %s",
            pointcloud_dir_.c_str()
        );

        ROS_INFO("================================================");
    }


private:

    // =========================================================
    // CALLBACK
    // =========================================================

    void callback(
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::ImageConstPtr& confidence_msg,
        const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
    {
        // =====================================================
        // 1. LEFT IMAGE
        // =====================================================

        cv_bridge::CvImageConstPtr image_ptr;

        try
        {
            image_ptr =
                cv_bridge::toCvShare(
                    image_msg,
                    sensor_msgs::image_encodings::BGR8
                );
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR(
                "LEFT IMAGE cv_bridge error: %s",
                e.what()
            );

            return;
        }


        // =====================================================
        // 2. CONFIDENCE MAP
        //
        // encoding 강제 변환하지 않고
        // ZED 원본 그대로 받음.
        // =====================================================

        cv_bridge::CvImageConstPtr confidence_ptr;

        try
        {
            confidence_ptr =
                cv_bridge::toCvShare(
                    confidence_msg
                );
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR(
                "CONFIDENCE cv_bridge error: %s",
                e.what()
            );

            return;
        }


        const cv::Mat& image =
            image_ptr->image;

        const cv::Mat& confidence =
            confidence_ptr->image;


        const uint32_t width =
            static_cast<uint32_t>(
                image.cols
            );

        const uint32_t height =
            static_cast<uint32_t>(
                image.rows
            );


        // =====================================================
        // Image <-> Confidence 크기 확인
        // =====================================================

        if (
            confidence.cols != static_cast<int>(width) ||
            confidence.rows != static_cast<int>(height)
        )
        {
            ROS_ERROR_THROTTLE(
                2.0,
                "CONFIDENCE SIZE MISMATCH | "
                "image=%ux%u confidence=%dx%d",
                width,
                height,
                confidence.cols,
                confidence.rows
            );

            return;
        }


        // =====================================================
        // Image <-> PointCloud 크기 확인
        //
        // 현재 확인값:
        //
        // Image      = 640 x 360
        // PointCloud = 640 x 360
        //
        // organized cloud이므로 pixel과 1:1 대응 가능
        // =====================================================

        if (
            pointcloud_msg->width != width ||
            pointcloud_msg->height != height
        )
        {
            ROS_ERROR_THROTTLE(
                2.0,
                "POINTCLOUD SIZE MISMATCH | "
                "image=%ux%u cloud=%ux%u",
                width,
                height,
                pointcloud_msg->width,
                pointcloud_msg->height
            );

            return;
        }


        // =====================================================
        // Organized PointCloud 확인
        // =====================================================

        if (pointcloud_msg->height <= 1)
        {
            ROS_ERROR_THROTTLE(
                2.0,
                "POINTCLOUD IS NOT ORGANIZED | "
                "width=%u height=%u",
                pointcloud_msg->width,
                pointcloud_msg->height
            );

            return;
        }


        // =====================================================
        // 동일한 Frame 번호 생성
        //
        // image/000000.png
        // confidence/000000.tiff
        // pointcloud/000000.pcd
        // =====================================================

        std::stringstream ss;

        ss << std::setw(6)
           << std::setfill('0')
           << frame_count_;

        const std::string frame_id =
            ss.str();


        // =====================================================
        // 저장 경로
        // =====================================================

        const std::string image_path =
            image_dir_ +
            "/" +
            frame_id +
            ".png";


        const std::string confidence_path =
            confidence_dir_ +
            "/" +
            frame_id +
            ".tiff";


        const std::string pointcloud_path =
            pointcloud_dir_ +
            "/" +
            frame_id +
            ".pcd";


        // =====================================================
        // LEFT IMAGE 저장
        // =====================================================

        if (!cv::imwrite(
                image_path,
                image
            ))
        {
            ROS_ERROR(
                "Failed to save image: %s",
                image_path.c_str()
            );

            return;
        }


        // =====================================================
        // CONFIDENCE MAP 저장
        //
        // TIFF:
        // confidence 값 손실 최소화
        // float map도 그대로 저장 가능
        // =====================================================

        if (!cv::imwrite(
                confidence_path,
                confidence
            ))
        {
            ROS_ERROR(
                "Failed to save confidence: %s",
                confidence_path.c_str()
            );

            return;
        }


        // =====================================================
        // POINTCLOUD 저장
        //
        // 절대 PointXYZ로 재구성하지 않음.
        //
        // PCLPointCloud2 그대로 저장:
        //
        // - width 유지
        // - height 유지
        // - NaN 유지
        // - point 순서 유지
        // - field 유지
        //
        // 따라서 이미지 pixel 순서 유지
        // =====================================================

        pcl::PCLPointCloud2 pcl_cloud;

        pcl_conversions::toPCL(
            *pointcloud_msg,
            pcl_cloud
        );


        // =====================================================
        // PCL 1.10 대응
        // =====================================================

        pcl::PCDWriter writer;


        if (
            writer.writeBinary(
                pointcloud_path,
                pcl_cloud
            ) < 0
        )
        {
            ROS_ERROR(
                "Failed to save pointcloud: %s",
                pointcloud_path.c_str()
            );

            return;
        }


        // =====================================================
        // 픽셀 대응
        //
        // width = 640
        // height = 360
        //
        // index = v * width + u
        //
        //
        // IMAGE
        //
        // (u,v)
        //
        //        |
        //        v
        //
        // CONFIDENCE
        //
        // confidence(v,u)
        //
        //        |
        //        v
        //
        // POINTCLOUD
        //
        // point[index]
        //
        //
        // index = v * width + u
        //
        // =====================================================

        const size_t total_points =
            static_cast<size_t>(width) *
            static_cast<size_t>(height);


        ROS_INFO(
            "[%s] SAVED | "
            "image=%ux%u | "
            "confidence=%dx%d | "
            "cloud=%ux%u | "
            "total=%zu | "
            "confidence_encoding=%s",
            frame_id.c_str(),
            width,
            height,
            confidence.cols,
            confidence.rows,
            pointcloud_msg->width,
            pointcloud_msg->height,
            total_points,
            confidence_msg->encoding.c_str()
        );


        // 다음 프레임
        frame_count_++;
    }


    // =========================================================
    // ROS
    // =========================================================

    ros::NodeHandle nh_;


    // =========================================================
    // Subscribers
    // =========================================================

    message_filters::Subscriber<
        sensor_msgs::Image
    > image_sub_;


    message_filters::Subscriber<
        sensor_msgs::Image
    > confidence_sub_;


    message_filters::Subscriber<
        sensor_msgs::PointCloud2
    > pointcloud_sub_;


    // =========================================================
    // Synchronizer
    // =========================================================

    std::shared_ptr<Synchronizer>
        sync_;


    // =========================================================
    // Topic 이름
    // =========================================================

    std::string image_topic_;
    std::string confidence_topic_;
    std::string pointcloud_topic_;


    // =========================================================
    // 저장 경로
    // =========================================================

    std::string save_dir_;

    std::string image_dir_;
    std::string confidence_dir_;
    std::string pointcloud_dir_;


    // =========================================================
    // Frame 번호
    // =========================================================

    size_t frame_count_;
};


// =============================================================
// MAIN
// =============================================================

int main(
    int argc,
    char** argv
)
{
    ros::init(
        argc,
        argv,
        "stereo_image_confidence_pointcloud"
    );


    StereoDataSaver saver;


    ros::spin();


    return 0;
}
