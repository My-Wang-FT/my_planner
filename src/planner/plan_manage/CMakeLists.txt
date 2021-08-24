cmake_minimum_required(VERSION 2.8.3)
project(my_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  traj_utils
)


# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS plan_env  traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(position_cmd_test src/position_cmd_test.cpp)
target_link_libraries(position_cmd_test ${catkin_LIBRARIES})
# add_dependencies(position_cmd_test ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(minimum_snap src/min_snap.cpp)
target_link_libraries(minimum_snap ${catkin_LIBRARIES})
# add_dependencies(minimum_snap ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(my_traj_server src/my_traj_server.cpp)
target_link_libraries(my_traj_server ${catkin_LIBRARIES})
# add_dependencies(my_traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(poly_traj_generator src/poly_traj_generator.cpp)
target_link_libraries(poly_traj_generator ${catkin_LIBRARIES})
# add_dependencies(poly_traj_generator ${${PROJECT_NAME}_EXPORTED_TARGETS})
