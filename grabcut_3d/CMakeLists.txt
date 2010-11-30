cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(CMAKE_CXX_FLAGS "-p")

#set(CMAKE_VERBOSE_MAKEFILE true)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
rosbuild_add_boost_directories()

rosbuild_add_executable(grabcut_3d src/grabcut_3d.cpp src/grabcut_3d_app.cpp)
rosbuild_add_library(grabcut3d src/grabcut_3d.cpp)
