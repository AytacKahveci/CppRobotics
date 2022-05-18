cmake_minimum_required(VERSION 3.1.0)
project(plotting)

set(${PROJECT_NAME}_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(${PROJECT_NAME}_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}} CACHE STRING "version" FORCE)

add_compile_options(-std=c++17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(BIN_DEST "bin")
set(INCLUDE_DEST "include/${PROJECT_NAME}")
set(LIB_DEST "lib/${PROJECT_NAME}")
set(DOC_DEST "docs/${PROJECT_NAME}")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(plotting_lib_DIR CACHE INTERNAL "A directory with the in-source config file"
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

find_package(Qt5 REQUIRED COMPONENTS
  Widgets
  Core
  Gui
  PrintSupport)

find_package(Boost REQUIRED COMPONENTS system thread)

set(headers ${CMAKE_CURRENT_SOURCE_DIR}/include/plotting
            ${CMAKE_CURRENT_SOURCE_DIR}/include/qcustomplot
            ${Qt5Core_INCLUDE_DIRS}
            ${Qt5Widgets_INCLUDE_DIRS}
            ${Qt5Gui_INCLUDE_DIRS}
            ${Qt5PrintSupport_INCLUDE_DIRS})
message(STATUS "Headers:" ${headers})
include_directories(include)

SET(library_sources
  include/qcustomplot/qcustomplot.h
  include/plotting/plotting.h
  include/qcustomplot/qcustomplot.cpp
  src/plotting.cpp)

add_library(plotting_lib
  ${library_sources})
target_include_directories(plotting_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${INCLUDE_DEST}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${LIB_DEST}>)

target_link_libraries(plotting_lib
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
  ${Boost_LIBRARIES})

SET(external_sources
  include/qcustomplot/qcustomplot.h
  include/plotting/plotting.h
  include/qcustomplot/qcustomplot.cpp
  src/plotting.cpp
  src/main.cpp)

add_executable(plotting
  ${external_sources})

target_link_libraries(plotting 
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
  ${Boost_LIBRARIES})

add_subdirectory(docs)

install(TARGETS plotting_lib EXPORT plotting_lib DESTINATION ${LIB_DEST})
install(DIRECTORY ${headers} DESTINATION ${INCLUDE_DEST})
install(EXPORT plotting_lib DESTINATION ${LIB_DEST})
install(FILES cmake/plotting_libConfig.cmake DESTINATION ${LIB_DEST})