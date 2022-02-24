cmake_minimum_required(VERSION 3.0.2)
project(fastscnn_wrapper)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############################################
## Clone the segmentation network from git ##
#############################################

include(ExternalProject)
find_package(Git REQUIRED)

## Add the Fast SCNN pytorch code from git
set(FAST_SCNN_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/fast_scnn)
set(FAST_SCNN_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/fast_scnn)
set(FAST_SCNN_INCLUDE_DIR ${FAST_SCNN_INSTALL_DIR})
ExternalProject_Add(
  fast_scnn
  GIT_REPOSITORY https://github.com/Tramac/Fast-SCNN-pytorch.git
  PREFIX ${FAST_SCNN_BUILD_DIR}
  GIT_TAG 0638517d359ae1664a27dfb2cd1780a40a06c465
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND 
    ${CMAKE_COMMAND} -E copy_directory ${FAST_SCNN_BUILD_DIR}/src/fast_scnn ${FAST_SCNN_INSTALL_DIR}/
  TEST_COMMAND ""
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

