cmake_minimum_required(VERSION 3.10.2)
project(custom_amcl)

include(CheckIncludeFile)
include(CheckSymbolExists)

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/AMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES
    custom_amcl_sensors
    custom_amcl_map
    custom_amcl_pf
)

include_directories(
  include
  src/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

check_include_file(unistd.h HAVE_UNISTD_H)
if(HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif()

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if(HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif()

add_library(custom_amcl_pf
  src/amcl/pf/pf.c
  src/amcl/pf/pf_kdtree.c
  src/amcl/pf/pf_pdf.c
  src/amcl/pf/pf_vector.c
  src/amcl/pf/eig3.c
  src/amcl/pf/pf_draw.c
)

add_library(custom_amcl_map
  src/amcl/map/map.c
  src/amcl/map/map_cspace.cpp
  src/amcl/map/map_range.c
  src/amcl/map/map_store.c
  src/amcl/map/map_draw.c
)

add_library(custom_amcl_sensors
  src/amcl/sensors/amcl_sensor.cpp
  src/amcl/sensors/amcl_odom.cpp
  src/amcl/sensors/amcl_laser.cpp
)

add_dependencies(custom_amcl_sensors
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(custom_amcl_sensors
  custom_amcl_map
  custom_amcl_pf
  ${catkin_LIBRARIES}
)

add_executable(custom_amcl
  src/amcl_node.cpp
)

add_dependencies(custom_amcl
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(custom_amcl
  custom_amcl_sensors
  custom_amcl_map
  custom_amcl_pf
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS
  custom_amcl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  custom_amcl_sensors
  custom_amcl_map
  custom_amcl_pf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)
