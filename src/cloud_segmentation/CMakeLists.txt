cmake_minimum_required(VERSION 2.8.3)
project(cloud_segmentation)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")



find_package(catkin REQUIRED COMPONENTS
        roscpp rostime std_msgs sensor_msgs message_filters
        cv_bridge image_transport compressed_image_transport
        compressed_depth_image_transport kinect2_bridge
        yolo_node
        )
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(HDF5 REQUIRED)
find_package(FLANN REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cloud_segmentation
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${HDF5_INCLUDE_DIR}
)

add_executable(cloud_segmentation src/cloud_seg.cpp)
target_link_libraries(cloud_segmentation
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )
