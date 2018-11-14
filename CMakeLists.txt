cmake_minimum_required(VERSION 2.8)
project(ffmpeg_image_transport)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport ffmpeg_image_transport_msgs sensor_msgs dynamic_reconfigure)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(FFMPEG REQUIRED)

generate_dynamic_reconfigure_options(cfg/EncoderDynConfig.cfg)

catkin_package(CATKIN_DEPENDS ffmpeg_image_transport_msgs sensor_msgs)

find_package(OpenCV)

include_directories(include
  ${FFMPEG_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# add the plugin
add_library(ffmpeg_image_transport src/manifest.cpp src/ffmpeg_publisher.cpp src/ffmpeg_subscriber.cpp src/ffmpeg_encoder.cpp src/ffmpeg_decoder.cpp src/tdiff.cpp)

add_dependencies(ffmpeg_image_transport ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ffmpeg_image_transport ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${FFMPEG_LIBRARIES})

