cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
include_directories(/usr/include/vtk-5.8)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

rosbuild_add_executable(test src/test.cpp src/visualization.cpp src/features/normal.cpp src/features/surflet.cpp src/util/util.cpp)
rosbuild_link_boost(test system)
target_link_libraries(test libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(detect_keypoints src/detect_keypoints.cpp src/keypoints/sift_extractor.cpp)

rosbuild_add_executable(correspondence_grouping src/correspondence_grouping.cpp)
rosbuild_link_boost(correspondence_grouping system)
target_link_libraries(correspondence_grouping libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(correspondence_detection src/correspondence_detection.cpp)
rosbuild_link_boost(correspondence_detection system)
target_link_libraries(correspondence_detection libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(extract_narf src/extract_narf.cpp src/keypoints/narf.cpp)
rosbuild_link_boost(extract_narf system)
target_link_libraries(extract_narf libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(save_clouds src/save_clouds.cpp)
rosbuild_link_boost(save_clouds system)