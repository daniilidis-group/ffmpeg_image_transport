cmake_minimum_required(VERSION 2.8)
project(ffmpeg_image_transport)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport ffmpeg_image_transport_msgs sensor_msgs dynamic_reconfigure)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(FFMPEG REQUIRED)

generate_dynamic_reconfigure_options(cfg/EncoderDyn.cfg)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
DEPENDS FFMPEG
CATKIN_DEPENDS roscpp cv_bridge image_transport
ffmpeg_image_transport_msgs sensor_msgs dynamic_reconfigure)

find_package(OpenCV)

include_directories(include
  ${FFMPEG_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# add the plugin
add_library(${PROJECT_NAME} src/manifest.cpp src/ffmpeg_publisher.cpp
src/ffmpeg_subscriber.cpp src/ffmpeg_encoder.cpp
src/ffmpeg_decoder.cpp src/tdiff.cpp ${FFMPEG_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${FFMPEG_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  ffmpeg_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
