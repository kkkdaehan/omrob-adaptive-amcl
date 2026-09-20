#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


struct FrameFile {
    int frame_number;
    boost::filesystem::path path;
};


struct CachedFrame {
    int frame_number;
    std::string filename;
    sensor_msgs::PointCloud2 msg;
};


/**
 * cloud_0.ply   -> 0
 * cloud_1.ply   -> 1
 * cloud_125.ply -> 125
 */
bool extractFrameNumber(const std::string& filename, int& frame_number) {
    const std::regex pattern(R"(cloud_(\d+)\.ply)");
    std::smatch match;

    if (!std::regex_match(filename, match, pattern)) {
        return false;
    }

    frame_number = std::stoi(match[1].str());
    return true;
}


/**
 * 폴더 내부의 cloud_XXX.ply 파일을 모두 찾고
 * cloud 번호 기준으로 정렬합니다.
 */
std::vector<FrameFile> findFrameFiles(const std::string& input_dir) {
    std::vector<FrameFile> files;
    const boost::filesystem::path directory(input_dir);

    if (!boost::filesystem::exists(directory) ||
        !boost::filesystem::is_directory(directory)) {
        ROS_ERROR_STREAM("Input directory does not exist: " << input_dir);
        return files;
    }

    for (boost::filesystem::directory_iterator it(directory);
         it != boost::filesystem::directory_iterator();
         ++it) {

        if (!boost::filesystem::is_regular_file(it->path())) {
            continue;
        }

        const std::string filename = it->path().filename().string();

        int frame_number = -1;

        if (extractFrameNumber(filename, frame_number)) {
            files.push_back({frame_number, it->path()});
        }
    }

    std::sort(
        files.begin(),
        files.end(),
        [](const FrameFile& a, const FrameFile& b) {
            return a.frame_number < b.frame_number;
        }
    );

    return files;
}


/**
 * cloud 번호 중간 누락 여부를 출력합니다.
 */
void printMissingFrames(const std::vector<FrameFile>& files) {
    if (files.empty()) {
        return;
    }

    const int first_frame = files.front().frame_number;
    const int last_frame = files.back().frame_number;

    std::vector<int> missing_frames;
    std::size_t current_index = 0;

    for (int expected = first_frame; expected <= last_frame; ++expected) {
        if (current_index < files.size() &&
            files[current_index].frame_number == expected) {
            ++current_index;
        } else {
            missing_frames.push_back(expected);
        }
    }

    if (missing_frames.empty()) {
        ROS_INFO("Missing frames    : none");
        return;
    }

    std::cout << "[WARN] Missing frames: ";

    for (std::size_t i = 0; i < missing_frames.size(); ++i) {
        std::cout << missing_frames[i];

        if (i + 1 < missing_frames.size()) {
            std::cout << ", ";
        }
    }

    std::cout << std::endl;
}


std::string toLower(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        }
    );

    return value;
}


/**
 * PLY 내부에서 좌표 필드 검색
 *
 * x, y, z 또는 scalar_x, scalar_y, scalar_z 이름을 처리합니다.
 */
const pcl::PCLPointField* findCoordinateField(
    const pcl::PCLPointCloud2& cloud,
    const std::string& coordinate
) {
    for (const auto& field : cloud.fields) {
        const std::string field_name = toLower(field.name);

        if (field_name == coordinate ||
            field_name == "scalar_" + coordinate) {
            return &field;
        }
    }

    return nullptr;
}


template<typename T>
bool copyNumericValue(
    const std::vector<std::uint8_t>& data,
    std::size_t offset,
    double& output
) {
    if (offset + sizeof(T) > data.size()) {
        return false;
    }

    T value;
    std::memcpy(&value, data.data() + offset, sizeof(T));

    output = static_cast<double>(value);
    return true;
}


/**
 * PCLPointCloud2 내부의 수치 필드를 double 형태로 읽습니다.
 *
 * 새 PLY 좌표 필드가 float 또는 double 중 어느 형식이어도
 * x, y, z 값을 읽을 수 있습니다.
 */
bool readFieldAsDouble(
    const pcl::PCLPointCloud2& cloud,
    const pcl::PCLPointField& field,
    std::size_t point_index,
    double& value
) {
    if (cloud.width == 0) {
        return false;
    }

    const std::size_t row = point_index / cloud.width;
    const std::size_t col = point_index % cloud.width;

    const std::size_t offset =
        row * cloud.row_step +
        col * cloud.point_step +
        field.offset;

    switch (field.datatype) {
        case pcl::PCLPointField::FLOAT32:
            return copyNumericValue<float>(cloud.data, offset, value);

        case pcl::PCLPointField::FLOAT64:
            return copyNumericValue<double>(cloud.data, offset, value);

        case pcl::PCLPointField::INT8:
            return copyNumericValue<std::int8_t>(cloud.data, offset, value);

        case pcl::PCLPointField::UINT8:
            return copyNumericValue<std::uint8_t>(cloud.data, offset, value);

        case pcl::PCLPointField::INT16:
            return copyNumericValue<std::int16_t>(cloud.data, offset, value);

        case pcl::PCLPointField::UINT16:
            return copyNumericValue<std::uint16_t>(cloud.data, offset, value);

        case pcl::PCLPointField::INT32:
            return copyNumericValue<std::int32_t>(cloud.data, offset, value);

        case pcl::PCLPointField::UINT32:
            return copyNumericValue<std::uint32_t>(cloud.data, offset, value);

        default:
            return false;
    }
}


/**
 * 첫 번째 PLY의 실제 point 필드 정보를 출력합니다.
 *
 * 이것은 PCL 경고가 아니라 현재 데이터 확인용 ROS_INFO 출력입니다.
 */
void printPlyFields(const pcl::PCLPointCloud2& raw_cloud) {
    ROS_INFO("PLY vertex fields detected:");

    for (const auto& field : raw_cloud.fields) {
        ROS_INFO_STREAM(
            "  name=" << field.name
            << ", datatype=" << static_cast<int>(field.datatype)
            << ", offset=" << field.offset
        );
    }
}


/**
 * PLY 한 장을 읽어 PointXYZI cloud로 변환합니다.
 *
 * optical_to_ros == true:
 *
 * 카메라 optical 좌표계
 *   x = 오른쪽
 *   y = 아래쪽
 *   z = 전방
 *
 * ROS base_link 기준 좌표계
 *   x = 전방
 *   y = 왼쪽
 *   z = 위쪽
 *
 * 변환식:
 *   X_ros =  Z_camera
 *   Y_ros = -X_camera
 *   Z_ros = -Y_camera
 */
bool loadPlyCloud(
    const std::string& ply_path,
    bool optical_to_ros,
    double pre_voxel_leaf,
    bool print_fields,
    pcl::PointCloud<pcl::PointXYZI>& output_cloud
) {
    pcl::PCLPointCloud2 raw_cloud;

    /*
     * PLY 읽기.
     *
     * PLY 안에 camera element가 있더라도,
     * main()에서 PCL verbosity를 L_ERROR로 설정했으므로
     * focal, k1, k2 등이 처리되지 않는다는 반복 경고는 출력되지 않습니다.
     */
    if (pcl::io::loadPLYFile(ply_path, raw_cloud) < 0) {
        ROS_ERROR_STREAM("Failed to read PLY: " << ply_path);
        return false;
    }

    if (print_fields) {
        printPlyFields(raw_cloud);
    }

    const pcl::PCLPointField* field_x =
        findCoordinateField(raw_cloud, "x");

    const pcl::PCLPointField* field_y =
        findCoordinateField(raw_cloud, "y");

    const pcl::PCLPointField* field_z =
        findCoordinateField(raw_cloud, "z");

    if (field_x == nullptr ||
        field_y == nullptr ||
        field_z == nullptr) {

        ROS_ERROR_STREAM(
            "PLY does not contain readable x/y/z fields: " << ply_path
        );

        printPlyFields(raw_cloud);
        return false;
    }

    const std::size_t total_points =
        static_cast<std::size_t>(raw_cloud.width) *
        static_cast<std::size_t>(raw_cloud.height);

    pcl::PointCloud<pcl::PointXYZI> converted_cloud;
    converted_cloud.points.reserve(total_points);

    for (std::size_t i = 0; i < total_points; ++i) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        if (!readFieldAsDouble(raw_cloud, *field_x, i, x) ||
            !readFieldAsDouble(raw_cloud, *field_y, i, y) ||
            !readFieldAsDouble(raw_cloud, *field_z, i, z)) {
            continue;
        }

        if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z)) {
            continue;
        }

        pcl::PointXYZI point;

        if (optical_to_ros) {
            point.x = static_cast<float>(z);
            point.y = static_cast<float>(-x);
            point.z = static_cast<float>(-y);
        } else {
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = static_cast<float>(z);
        }

        /*
         * hdl_graph_slam 입력 형식 대응용 intensity 값.
         * 현재 PLY의 색상이나 intensity를 사용하지 않고 고정값을 넣습니다.
         */
        point.intensity = 1.0f;

        converted_cloud.points.push_back(point);
    }

    converted_cloud.width =
        static_cast<std::uint32_t>(converted_cloud.points.size());

    converted_cloud.height = 1;
    converted_cloud.is_dense = false;

    if (converted_cloud.empty()) {
        ROS_ERROR_STREAM("No valid points after conversion: " << ply_path);
        return false;
    }

    /*
     * publish 전 voxel downsampling.
     * PLY 전체를 메모리에 저장하므로 메모리 사용량을 줄이고,
     * hdl_graph_slam 입력 cloud도 가볍게 만듭니다.
     */
    if (pre_voxel_leaf > 0.0) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

        voxel_filter.setInputCloud(converted_cloud.makeShared());

        const float leaf = static_cast<float>(pre_voxel_leaf);

        voxel_filter.setLeafSize(leaf, leaf, leaf);
        voxel_filter.filter(output_cloud);
    } else {
        output_cloud = converted_cloud;
    }

    output_cloud.width =
        static_cast<std::uint32_t>(output_cloud.points.size());

    output_cloud.height = 1;
    output_cloud.is_dense = false;

    return !output_cloud.empty();
}


/**
 * 모든 PLY 로딩 완료 후,
 * 터미널에서 q + Enter 입력이 들어올 때까지 publish하지 않고 대기합니다.
 */
bool waitForStartCommand() {
    std::string command;

    std::cout << "\n";
    std::cout << "====================================================\n";
    std::cout << "[READY] All PLY frames are loaded into memory.\n";
    std::cout << "        Start hdl_graph_slam before publishing.\n";
    std::cout << "        Type q and press ENTER to start publishing.\n";
    std::cout << "====================================================\n";

    while (ros::ok()) {
        std::cout << "> " << std::flush;

        if (!std::getline(std::cin, command)) {
            ROS_ERROR("Failed to read terminal input.");
            return false;
        }

        if (command == "q" || command == "Q") {
            ROS_INFO("Start command received.");
            return true;
        }

        std::cout << "Type q and press ENTER to start publishing."
                  << std::endl;
    }

    return false;
}


/**
 * q 입력 이후 hdl_graph_slam subscriber 연결을 확인합니다.
 *
 * subscriber가 없으면 첫 point cloud가 유실될 수 있으므로
 * 연결될 때까지 기다립니다.
 */
bool waitForSubscriber(
    ros::Publisher& publisher,
    const std::string& topic
) {
    if (publisher.getNumSubscribers() > 0) {
        ROS_INFO_STREAM(
            "Subscriber already connected on " << topic
            << ". count=" << publisher.getNumSubscribers()
        );

        return true;
    }

    ROS_INFO_STREAM(
        "Waiting for subscriber on " << topic
        << ". Start hdl_graph_slam if necessary."
    );

    ros::WallRate wait_rate(5.0);

    while (ros::ok() && publisher.getNumSubscribers() == 0) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    if (!ros::ok()) {
        return false;
    }

    ROS_INFO_STREAM(
        "Subscriber connected on " << topic
        << ". count=" << publisher.getNumSubscribers()
    );

    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ply_topic_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /*
     * 핵심 추가 부분:
     *
     * 새 cloud_*.ply 파일에는 camera element가 포함되어 있을 수 있습니다.
     * PCL은 focal, centerx, centery, k1, k2 등의 camera 메타데이터를
     * 현재 point cloud 결과에 넣지 못해 warning을 반복 출력합니다.
     *
     * 현재 SLAM 입력에는 vertex의 x, y, z만 필요하므로,
     * PCL warning은 숨기고 실제 error만 출력하도록 설정합니다.
     */
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::string input_dir;
    std::string topic;
    std::string frame_id;

    double hz = 1.0;
    double pre_voxel_leaf = 0.05;

    bool optical_to_ros = true;
    bool loop = false;
    bool wait_for_subscriber = true;

    pnh.param<std::string>(
        "input_dir",
        input_dir,
        "/home/omrob/catkin_ws/src/camera_ros/src/ply"
    );

    pnh.param<std::string>(
        "topic",
        topic,
        "/velodyne_points"
    );

    /*
     * 이전의 base_link <-> velodyne TF 오류를 피하기 위해
     * 현재는 point cloud를 base_link frame으로 publish합니다.
     */
    pnh.param<std::string>(
        "frame_id",
        frame_id,
        "base_link"
    );

    pnh.param<double>("hz", hz, 1.0);
    pnh.param<double>("pre_voxel_leaf", pre_voxel_leaf, 0.05);

    pnh.param<bool>("optical_to_ros", optical_to_ros, true);
    pnh.param<bool>("loop", loop, false);
    pnh.param<bool>("wait_for_subscriber", wait_for_subscriber, true);

    if (hz <= 0.0) {
        ROS_ERROR("hz must be greater than zero.");
        return 1;
    }

    /*
     * 1. cloud_*.ply 파일 검색 및 숫자 정렬
     */
    const std::vector<FrameFile> files = findFrameFiles(input_dir);

    if (files.empty()) {
        ROS_ERROR_STREAM(
            "No cloud_*.ply files found in: " << input_dir
        );

        return 1;
    }

    ROS_INFO("====================================================");
    ROS_INFO("PLY Topic Publisher Configuration");
    ROS_INFO("====================================================");
    ROS_INFO_STREAM("Input directory     : " << input_dir);
    ROS_INFO_STREAM("PLY file count      : " << files.size());
    ROS_INFO_STREAM("First cloud number  : " << files.front().frame_number);
    ROS_INFO_STREAM("Last cloud number   : " << files.back().frame_number);
    ROS_INFO_STREAM("Output topic        : " << topic);
    ROS_INFO_STREAM("Frame ID            : " << frame_id);
    ROS_INFO_STREAM("Publish frequency   : " << hz << " Hz");
    ROS_INFO_STREAM("Optical to ROS      : "
                    << (optical_to_ros ? "true" : "false"));
    ROS_INFO_STREAM("Pre voxel leaf size : " << pre_voxel_leaf << " m");
    ROS_INFO_STREAM("Loop playback       : "
                    << (loop ? "true" : "false"));
    ROS_INFO("PCL camera metadata warnings are suppressed.");
    ROS_INFO("====================================================");

    printMissingFrames(files);

    /*
     * 2. PLY 전체를 미리 읽어서 메모리에 적재
     *
     * publish 도중에는 디스크에서 PLY를 다시 읽지 않습니다.
     */
    std::vector<CachedFrame> cached_frames;
    cached_frames.reserve(files.size());

    ROS_INFO("Preloading all PLY frames into memory...");

    for (std::size_t i = 0; i < files.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI> cloud;

        const bool print_fields = (i == 0);

        if (!loadPlyCloud(
                files[i].path.string(),
                optical_to_ros,
                pre_voxel_leaf,
                print_fields,
                cloud)) {

            ROS_WARN_STREAM(
                "Skipping unreadable cloud: "
                << files[i].path.filename().string()
            );

            continue;
        }

        sensor_msgs::PointCloud2 msg;

        pcl::toROSMsg(cloud, msg);

        msg.header.frame_id = frame_id;

        cached_frames.push_back({
            files[i].frame_number,
            files[i].path.filename().string(),
            msg
        });

        if (i % 20 == 0 || i + 1 == files.size()) {
            ROS_INFO_STREAM(
                "Loaded ["
                << std::setw(4) << (i + 1)
                << "/"
                << std::setw(4) << files.size()
                << "] cloud="
                << std::setw(4) << files[i].frame_number
                << ", points_after_voxel="
                << cloud.points.size()
            );
        }
    }

    if (cached_frames.empty()) {
        ROS_ERROR("All PLY clouds failed to load.");
        return 1;
    }

    ROS_INFO_STREAM(
        "Preload complete. Cached clouds: " << cached_frames.size()
    );

    /*
     * 3. Publisher 등록
     *
     * q 입력 전에도 publisher는 ROS master에 등록됩니다.
     * 따라서 hdl_graph_slam이 subscriber로 연결될 수 있습니다.
     */
    ros::Publisher cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>(topic, 1);

    /*
     * 4. q 입력 전까지 publish 대기
     */
    if (!waitForStartCommand()) {
        return 0;
    }

    /*
     * 5. q 입력 이후 subscriber 확인
     */
    if (wait_for_subscriber) {
        if (!waitForSubscriber(cloud_pub, topic)) {
            return 0;
        }
    }

    /*
     * 6. 메모리에 저장된 point cloud를 설정 주기로 publish
     */
    ROS_INFO_STREAM(
        "Publishing " << cached_frames.size()
        << " clouds at " << hz << " Hz on " << topic
    );

    ros::WallRate publish_rate(hz);

    std::size_t index = 0;
    std::uint32_t sequence = 0;

    while (ros::ok()) {
        sensor_msgs::PointCloud2 msg = cached_frames[index].msg;

        msg.header.seq = sequence++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;

        cloud_pub.publish(msg);
        ros::spinOnce();

        ROS_INFO_STREAM(
            "Published ["
            << std::setw(4) << (index + 1)
            << "/"
            << std::setw(4) << cached_frames.size()
            << "] cloud="
            << std::setw(4) << cached_frames[index].frame_number
            << ", file="
            << cached_frames[index].filename
        );

        ++index;

        if (index >= cached_frames.size()) {
            if (loop) {
                index = 0;
                ROS_INFO("Reached final cloud. Restarting from cloud_0.");
            } else {
                ROS_INFO("Finished publishing all cached PLY clouds.");
                break;
            }
        }

        publish_rate.sleep();
    }

    return 0;
}
