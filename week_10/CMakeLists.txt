cmake_minimum_required(VERSION 2.8.3)
project(week_10)

set(CMAKE_BUILD_TYPE "Debug")
#set C++ version
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g") produce 執行檔
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
)


find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  #${PROJECT_SOURCE_DIR}/include	#(project location)
)



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(kf_test 
src/kf_test.cpp
src/kalman.cpp)
target_link_libraries(kf_test ${catkin_LIBRARIES})
