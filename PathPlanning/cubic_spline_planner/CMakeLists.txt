cmake_minimum_required(VERSION 3.0)
project(cubic_spline_planner)

set(${PROJECT_NAME}_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(${PROJECT_NAME}_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}} CACHE STRING "version" FORCE)

add_compile_options(-std=c++17)

set(BIN_DEST "bin")
set(INCLUDE_DEST "include/${PROJECT_NAME}")
set(LIB_DEST "lib/${PROJECT_NAME}")
set(DOC_DEST "docs/${PROJECT_NAME}")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

set(headers ${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_spline_planner
            ${EIGEN3_INCLUDE_DIRS}/Eigen)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(cubic_spline_planner src/cubic_spline_planner.cpp)
target_include_directories(cubic_spline_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${INCLUDE_DEST}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${LIB_DEST}>)

add_executable(test_cubic_spline_planner test/test_cubic_spline_planner.cpp)
target_link_libraries(test_cubic_spline_planner cubic_spline_planner ${GTEST_LIBRARIES} pthread)

add_subdirectory(docs)

install(TARGETS cubic_spline_planner EXPORT cubic_spline_planner DESTINATION ${LIB_DEST})
install(DIRECTORY ${headers} DESTINATION ${INCLUDE_DEST})
install(EXPORT cubic_spline_planner DESTINATION ${LIB_DEST})
install(FILES cmake/cubic_spline_plannerConfig.cmake DESTINATION ${LIB_DEST})

set(cubic_spline_planner_DIR CACHE INTERNAL "A directory with the in-source config file"
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)