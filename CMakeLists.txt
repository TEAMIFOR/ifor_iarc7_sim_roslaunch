cmake_minimum_required(VERSION 2.8.3)
project(nquad)


find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_msgs
  )

find_package(gazebo)

find_package( OpenCV REQUIRED )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED sensor_msgs cv_bridge roscpp std_msgs image_transport)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

FIND_PACKAGE( OpenCV REQUIRED )
IF (${OpenCV_VERSION} VERSION_LESS 2.3.0)
    MESSAGE(FATAL_ERROR "OpenCV version is not compatible : ${OpenCV_VERSION}")
ENDIF()

catkin_package(
  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_msgs roscpp std_msgs
  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

set(GROUND_BOT   
  src/groundbot.cpp
)


add_executable(groundbot_node ${GROUND_BOT})

target_link_libraries(groundbot_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_library(contact SHARED src/ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_libraries})
