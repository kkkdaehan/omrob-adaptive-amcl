cmake_minimum_required(VERSION 3.0.2)
project(camera_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  std_srvs
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_provider src/camera_provider.cpp)
target_link_libraries(camera_provider
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
target_link_libraries(cmd_vel_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(angle_cmd_vel_publisher src/angle_cmd_vel_publisher.cpp)
target_link_libraries(angle_cmd_vel_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(image_save_left_right src/image_save_left_right.cpp)
target_link_libraries(image_save_left_right
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(ours_pointcloud src/ours_pointcloud.cpp)
target_link_libraries(ours_pointcloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(before_pointcloud src/before_pointcloud.cpp)
target_link_libraries(before_pointcloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(depth_to_hdl_points src/depth_to_hdl_points.cpp)
target_link_libraries(depth_to_hdl_points
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(GT_to_ply src/GT_to_ply.cpp)
target_link_libraries(GT_to_ply
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(odom_image_capture src/odom_image_capture.cpp)
target_link_libraries(odom_image_capture
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(stereo_image_confidence_pointcloud src/stereo_image_confidence_pointcloud.cpp)
target_link_libraries(stereo_image_confidence_pointcloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(
  odom_rotate_capture src/odom_rotate_capture.cpp)
target_link_libraries(
  odom_rotate_capture
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)




