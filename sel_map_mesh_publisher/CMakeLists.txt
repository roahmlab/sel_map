cmake_minimum_required(VERSION 3.0.2)
project(sel_map_mesh_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add optimizations
# add_compile_options(-O2)

## Multithreading
add_compile_options(-fopenmp -D_GLIBCXX_PARALLEL)

## Vectorization
add_compile_options(-march=native)

## Create Library
add_compile_options(-shared -Wl,-soname,sel_map_mesh_publisher -fPIC)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp sel_map_mesh eigen_340_wrapper)

## Find OpenMP (TODO: https://answers.ros.org/question/306391/how-to-use-parallel-programming-in-catkin/)
find_package(OpenMP REQUIRED)

## Setup python wrapper
catkin_python_setup()

#########################
## Custom Dependencies ##
#########################
#include(ExternalProject)

# THIS NEEDS TO GO
## Add Eigen3.4 using ExternalProject
#set(EIGEN_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/eigen)
#set(EIGEN_INSTALL_DIR ${CMAKE_BINARY_DIR}/libs/eigen3)
#set(EIGEN_INCLUDE_DIR ${EIGEN_INSTALL_DIR})
#ExternalProject_Add(
#  eigen_340
#  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
#  PREFIX ${EIGEN_BUILD_DIR}
#  CONFIGURE_COMMAND ""
#  BUILD_COMMAND ""
#  INSTALL_COMMAND
#    ${CMAKE_COMMAND} -E copy_directory ${EIGEN_BUILD_DIR}/src/eigen/Eigen ${EIGEN_INCLUDE_DIR}/Eigen 
#    && ${CMAKE_COMMAND} -E copy_directory ${EIGEN_BUILD_DIR}/src/eigen/unsupported ${EIGEN_INCLUDE_DIR}/unsupported
#  TEST_COMMAND ""
#)
#include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
#
#set(NANOFLANN_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/nanoflann)
#set(NANOFLANN_INSTALL_DIR ${CMAKE_BINARY_DIR}/libs/nanoflann)
#set(NANOFLANN_INCLUDE_DIR ${NANOFLANN_INSTALL_DIR})
#ExternalProject_Add(
#  nanoflann_132
#  URL https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.3.2.tar.gz
#  PREFIX ${NANOFLANN_BUILD_DIR}
#  CONFIGURE_COMMAND ""
#  BUILD_COMMAND ""
#  INSTALL_COMMAND
#    ${CMAKE_COMMAND} -E copy_directory ${NANOFLANN_BUILD_DIR}/src/nanoflann/include ${NANOFLANN_INCLUDE_DIR}/nanoflann
#  TEST_COMMAND ""
#)
#include_directories(SYSTEM ${NANOFLANN_INCLUDE_DIR})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sel_map_mesh_publisher
  CATKIN_DEPENDS sel_map_mesh roscpp eigen_340_wrapper
  DEPENDS OpenMP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/TriangularMeshDataAdaptor.cpp
  src/TriangularMeshPublisher.cpp
  src/py_wrapper.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME} eigen_340 nanoflann_132)

## Internal location of header files for ease
target_include_directories(${PROJECT_NAME} PRIVATE include/${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} gomp 
)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sel_map_mesh_publisher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
