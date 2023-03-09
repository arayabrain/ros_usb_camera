cmake_minimum_required(VERSION 3.0.2)
project(usb_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV ${OCV_VERSION} REQUIRED)
# find_package(OpenCV)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)