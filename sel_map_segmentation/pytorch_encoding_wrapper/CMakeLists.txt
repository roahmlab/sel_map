cmake_minimum_required(VERSION 3.0.2)
project(pytorch_encoding_wrapper)

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

find_program(PYTHON "python3")
if(NOT PYTHON)
  message(FATAL_ERROR "Python 3 is required!")
endif()

## Add the Hang Zhang PyTorch-Encoding models from git
set(ENCODING_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/pytorch_encoding)
set(ENCODING_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/encoding)
set(ENCODING_INCLUDE_DIR ${ENCODING_INSTALL_DIR})
ExternalProject_Add(
  pytorch_encoding
  GIT_REPOSITORY https://github.com/zhanghang1989/PyTorch-Encoding.git
  PREFIX ${ENCODING_BUILD_DIR}
  BINARY_DIR ${ENCODING_BUILD_DIR}/src/pytorch_encoding
  GIT_TAG 331ecdd5306104614cb414b16fbcd9d1a8d40e1e
  CONFIGURE_COMMAND ""
  BUILD_COMMAND 
    ${PYTHON} ${ENCODING_BUILD_DIR}/src/pytorch_encoding/setup.py build --build-lib ${ENCODING_BUILD_DIR}/build
  INSTALL_COMMAND 
    ${CMAKE_COMMAND} -E copy_directory ${ENCODING_BUILD_DIR}/build/encoding ${ENCODING_INSTALL_DIR}
  TEST_COMMAND ""
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

