cmake_minimum_required(VERSION 2.8)
project( CameraFrameServer )

find_package( OpenCV )
include_directories("${OpenCV_INCLUDE_DIRS}")

get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
set(XH264_INCLUDE "${PARENT_DIR}/x264" CACHE STRING "X264 source and library file")

include_directories("${XH264_INCLUDE}")
LINK_DIRECTORIES("${XH264_INCLUDE}")

set(CMAKE_PREFIX_PATH "${PARENT_DIR}/OpenIGTLink-XcodeBuild")
find_package( OpenIGTLink REQUIRED)
include_directories("${OpenIGTLink_INCLUDE_DIRS}")
LINK_DIRECTORIES("${OpenIGTLink_LIBRARY_DIRS}")

add_executable( CameraFrameServer CameraFrameServer.cpp)
add_executable( example example.cpp)
target_link_libraries( CameraFrameServer ${OpenCV_LIBS} OpenIGTLink x264)
target_link_libraries( example ${OpenCV_LIBS} OpenIGTLink x264)