cmake_minimum_required(VERSION 3.0.2)
project(mit_semseg_wrapper)

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

## Add the CSAIL Semantic Segmentation Network from Git
set(CSAIL_SEMSEG_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/csail_semseg)
set(CSAIL_SEMSEG_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/mit_semseg)
set(CSAIL_SEMSEG_INCLUDE_DIR ${CSAIL_SEMSEG_INSTALL_DIR})
ExternalProject_Add(
  csail_semseg
  GIT_REPOSITORY https://github.com/CSAILVision/semantic-segmentation-pytorch.git
  PREFIX ${CSAIL_SEMSEG_BUILD_DIR}
  GIT_TAG 8f27c9b97d2ca7c6e05333d5766d144bf7d8c31b
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND 
    ${CMAKE_COMMAND} -E copy_directory ${CSAIL_SEMSEG_BUILD_DIR}/src/csail_semseg/mit_semseg ${CSAIL_SEMSEG_INSTALL_DIR}/
    && ${CMAKE_COMMAND} -E copy_directory ${CSAIL_SEMSEG_BUILD_DIR}/src/csail_semseg/config ${CSAIL_SEMSEG_INSTALL_DIR}/config/
  TEST_COMMAND ""
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

