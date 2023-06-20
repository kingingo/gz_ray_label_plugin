cmake_minimum_required(VERSION 3.0.2)
project(gz_ray_label_plugin)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  gazebo_msgs
  rviz
  visualization_msgs
)

add_message_files(
  FILES 
  LabelPoint.msg 
  LabelPoints.msg 
)
generate_messages(DEPENDENCIES std_msgs visualization_msgs)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  #${CMAKE_CURRENT_BINARY_DIR}/msgs 
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS} 
  #${CMAKE_CURRENT_BINARY_DIR}/msgs
)
#add_subdirectory(msgs)

catkin_package(
  LIBRARIES
    rviz
    ${OGRE_LIBRARIES}
    ${rviz_ADDITIONAL_LIBRARIES}
    ${OPENGL_LIBRARIES}
  CATKIN_DEPENDS roscpp gazebo_ros visualization_msgs
)

add_library(${PROJECT_NAME} src/gz_ray_label_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})