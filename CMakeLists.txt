cmake_minimum_required(VERSION 3.0.2)
project(shslam)

find_package(PkgConfig REQUIRED)
pkg_check_modules(
  OpenCV
  yaml-cpp
)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  include
)

#add_executable(ros_app src/ros_app.cpp src/slam_system/raw_data_buffers_manager.cpp)
add_executable(ros_app
  src/ros_app.cpp
  src/shslam/slam_system.cpp
  src/shslam/slam_system/trackers_manager.cpp
  src/shslam/slam_system/trackers_manager/mono_cams_tracker.cpp
  src/shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.cpp
  src/shslam/slam_system/common_info_manager.cpp
  src/shslam/slam_system/raw_data_buffers_manager.cpp
)

add_dependencies(ros_app ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_app ${OpenCV_LIBS} Eigen3::Eigen ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})