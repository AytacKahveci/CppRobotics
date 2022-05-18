cmake_minimum_required(VERSION 3.0)
project(mpc_control)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ../../PathPlanning/cubic_spline_planner/include
  qpOASES/include
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  qpOASES/bin
  ../../PathPlanning/cubic_spline_planner/build
)

add_library(mpc_control src/mpc_control.cpp)

add_executable(test_mpc_control test/test_mpc_control.cpp)
target_link_libraries(
  test_mpc_control 
  mpc_control 
  ${GTEST_LIBRARIES} 
  pthread 
  libqpOASES.so
  libcubic_spline_planner.a
)