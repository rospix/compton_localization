cmake_minimum_required(VERSION 3.1.2)
project(compton_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  mrs_lib
  mrs_msgs
  nodelet
  rad_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

generate_dynamic_reconfigure_options(
  cfg/compton_localization.cfg
  )

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg FILES
  Swarm.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
   CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
   LIBRARIES ComptonLocalization
   DEPENDS Eigen
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_library(ComptonLocalization src/compton_localization.cpp)

add_dependencies(ComptonLocalization
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(ComptonLocalization
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
