cmake_minimum_required(VERSION 2.8.3)
project(video_stream)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  )


find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} src/video_stream.cpp)

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------
