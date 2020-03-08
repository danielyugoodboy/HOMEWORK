cmake_minimum_required(VERSION 2.8.3)
project(week_3)

set(CMAKE_BUILD_TYPE "Debug")

#set C++ version
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  pcl_conversions
  pcl_ros
  cv_bridge
  tf
)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES week_11
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3 PCL Ceres
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
## include forlds named include
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


add_executable(tf_master src/tf_master.cpp)
target_link_libraries(tf_master ${catkin_LIBRARIES} )

add_executable(tf_boy src/tf_boy.cpp)
target_link_libraries(tf_boy ${catkin_LIBRARIES} )

add_executable(tf_girl src/tf_girl.cpp)
target_link_libraries(tf_girl ${catkin_LIBRARIES} )

add_executable(tf_childern src/tf_childern.cpp)
target_link_libraries(tf_childern ${catkin_LIBRARIES} )

add_executable(tf_homework src/tf_homework.cpp)
target_link_libraries(tf_homework ${catkin_LIBRARIES} )

add_executable(final_work src/final_work.cpp)
target_link_libraries(final_work ${catkin_LIBRARIES} )
