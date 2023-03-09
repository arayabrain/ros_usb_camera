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

install(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
