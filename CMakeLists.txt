cmake_minimum_required(VERSION 3.0)
project(CppRobotics)

add_subdirectory(utils/plotting)
add_subdirectory(PathPlanning/cubic_spline_planner)
add_subdirectory(PathTracking/pure_pursuit)
