cmake_minimum_required(VERSION 3.0)
project(pure_pursuit)

set(${PROJECT_NAME}_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(${PROJECT_NAME}_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}} CACHE STRING "version" FORCE)
set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

list(APPEND CMAKE_PREFIX_PATH 
  "${CMAKE_CURRENT_SOURCE_DIR}/../../utils/plotting/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../PathPlanning/cubic_spline_planner/cmake")

set(BIN_DEST "bin")
set(INCLUDE_DEST "include/${PROJECT_NAME}")
set(LIB_DEST "lib/${PROJECT_NAME}")
set(DOC_DEST "docs/${PROJECT_NAME}")

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(plotting_lib REQUIRED)
find_package(cubic_spline_planner REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS
  Widgets
  Core
  Gui
  PrintSupport)

FILE(GLOB headers include/${PROJECT_NAME}/*)

get_target_property(plotting_lib_includes plotting_lib INTERFACE_INCLUDE_DIRECTORIES)
MESSAGE(STATUS ${plotting_lib_includes})

get_target_property(cubic_spline_planner_includes cubic_spline_planner INTERFACE_INCLUDE_DIRECTORIES)
MESSAGE(STATUS ${cubic_spline_planner_includes})

include_directories(
  include
  ${plotting_lib_includes}
  ${cubic_spline_planner_includes}
)
  
add_library(pure_pursuit src/pure_pursuit.cpp)
target_include_directories(pure_pursuit PUBLIC
                        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
                        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
                        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${INCLUDE_DEST}>
                        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${LIB_DEST}>)
target_link_libraries(
  pure_pursuit PUBLIC
  plotting_lib
  cubic_spline_planner
)

add_executable(test_pure_pursuit test/test_pure_pursuit.cpp)
target_link_libraries(
  test_pure_pursuit 
  pure_pursuit 
  ${GTEST_LIBRARIES} 
  pthread
)

add_subdirectory(docs)

install(TARGETS test_pure_pursuit DESTINATION ${BIN_DEST})
install(TARGETS pure_pursuit EXPORT pure_pursuit DESTINATION ${LIB_DEST})
install(FILES ${headers} DESTINATION ${INCLUDE_DEST})
install(EXPORT pure_pursuit DESTINATION ${LIB_DEST})
install(FILES cmake/pure_pursuitConfig.cmake DESTINATION ${LIB_DEST})

set(pure_pursuit_DIR CACHE INTERNAL "A directory with the in-source config file"
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)