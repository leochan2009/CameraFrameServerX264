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
add_executable( example example.c)
message("${XH264_INCLUDE}")
include_directories("${XH264_INCLUDE}/filters")
include_directories("${XH264_INCLUDE}/common")
include_directories("${XH264_INCLUDE}/encoder")

add_executable( localX264 x264Local.c)

SET(SRCX264 x264.cpp
  "${XH264_INCLUDE}/input/input.c"
  "${XH264_INCLUDE}/input/timecode.c"
  "${XH264_INCLUDE}/input/raw.c"
  "${XH264_INCLUDE}/input/y4m.c"
  "${XH264_INCLUDE}/output/raw.c"
  "${XH264_INCLUDE}/output/matroska.c"
  "${XH264_INCLUDE}/output/matroska_ebml.c"
  "${XH264_INCLUDE}/output/flv.c"
  "${XH264_INCLUDE}/output/flv_bytestream.c"
  "${XH264_INCLUDE}/filters/filters.c"
  "${XH264_INCLUDE}/filters/video/video.c"
  "${XH264_INCLUDE}/filters/video/source.c"
  "${XH264_INCLUDE}/filters/video/internal.c"
  "${XH264_INCLUDE}/filters/video/resize.c"
  "${XH264_INCLUDE}/filters/video/cache.c"
  "${XH264_INCLUDE}/filters/video/fix_vfr_pts.c"
  "${XH264_INCLUDE}/filters/video/select_every.c"
  "${XH264_INCLUDE}/filters/video/crop.c"
  "${XH264_INCLUDE}/filters/video/depth.c"
  "${XH264_INCLUDE}/input/avs.c"
  "${XH264_INCLUDE}/input/thread.c"
#  "${XH264_INCLUDE}/input/ffms.c"
#  "${XH264_INCLUDE}/output/mp4_lsmash.c"
#  "${XH264_INCLUDE}/output/mp4.c"
)

add_executable(localX2642 ${SRCX264})
target_link_libraries( CameraFrameServer ${OpenCV_LIBS} OpenIGTLink x264)
target_link_libraries( example ${OpenCV_LIBS} x264)
target_link_libraries( localX264 ${OpenCV_LIBS} x264)
target_link_libraries( localX2642 ${OpenCV_LIBS} x264)