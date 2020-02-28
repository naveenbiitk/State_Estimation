cmake_minimum_required(VERSION 2.8.3)
project(gps_denied)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  std_msgs
  dynamic_reconfigure
  message_generation
  cmake_modules
  dwm1001
  geometry_msgs
)


find_package(Eigen REQUIRED)

catkin_package(
   DEPENDS Eigen
  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp
)
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS})

add_executable(KalmanFilter src/ekf.cpp)
target_link_libraries(KalmanFilter
    ${catkin_LIBRARIES})

add_executable(KalmanFilter_swarm src/ekf_swarm.cpp)
target_link_libraries(KalmanFilter_swarm
    ${catkin_LIBRARIES})

add_executable(IKalmanFilter_swarm src/ikf_swarm.cpp)
target_link_libraries(IKalmanFilter_swarm
    ${catkin_LIBRARIES})

add_executable(yaw src/yaw.cpp)
target_link_libraries(yaw
    ${catkin_LIBRARIES})