cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)

set(CMAKE_PREFIX_PATH "/opt/ros/melodic")
set(qt_build_DIR "/opt/ros/melodic/share/qt_build/cmake")
set(roscpp_DIR "/opt/ros/melodic/share/roscpp/cmake")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  qt_build
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rospy
  tf
  aruco
  aruco_msgs
  sensor_msgs
  visualization_msgs
)
#find_package(Boost REQUIRED system filesystem date_time thread ) #
generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros_utils
  CATKIN_DEPENDS aruco sensor_msgs tf 
)

include_directories(
  include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(aruco_ros_utils src/aruco_ros_utils.cpp)
target_link_libraries(aruco_ros_utils ${catkin_LIBRARIES})

add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp
                      src/follow_marker.cpp)
add_dependencies(single ${PROJECT_NAME}_gencfg)
target_link_libraries(single ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(double src/simple_double.cpp
#                      src/aruco_ros_utils.cpp)
#add_dependencies(double ${PROJECT_NAME}_gencfg)
#target_link_libraries(double ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
add_dependencies(marker_publisher ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# install(TARGETS single double marker_publisher aruco_ros_utils
install(TARGETS single marker_publisher aruco_ros_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

