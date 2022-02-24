cmake_minimum_required(VERSION 3.0.2)
project(sel_map_segmentation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############################################
## Clone the segmentation network from git ##
#############################################

#include(ExternalProject)
#find_package(Git REQUIRED)

## Add the CSAIL Semantic Segmentation Network from Git
#set(CSAIL_SEMSEG_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/csail_semseg)
#set(CSAIL_SEMSEG_INSTALL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/mit_semseg)
#set(CSAIL_SEMSEG_INCLUDE_DIR ${CSAIL_SEMSEG_INSTALL_DIR})
#ExternalProject_Add(
#  csail_semseg
#  GIT_REPOSITORY https://github.com/CSAILVision/semantic-segmentation-pytorch.git
#  PREFIX ${CSAIL_SEMSEG_BUILD_DIR}
#  GIT_TAG 8f27c9b97d2ca7c6e05333d5766d144bf7d8c31b
#  CONFIGURE_COMMAND ""
#  BUILD_COMMAND ""
#  INSTALL_COMMAND 
#    ${CMAKE_COMMAND} -E copy_directory ${CSAIL_SEMSEG_BUILD_DIR}/src/csail_semseg/mit_semseg ${CSAIL_SEMSEG_INSTALL_DIR}/
#  TEST_COMMAND ""
#)

## Add the Fast SCNN pytorch code from git
#set(FAST_SCNN_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/fast_scnn)
#set(FAST_SCNN_INSTALL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/fast_scnn)
#set(FAST_SCNN_INCLUDE_DIR ${FAST_SCNN_INSTALL_DIR})
#ExternalProject_Add(
#  fast_scnn
#  GIT_REPOSITORY https://github.com/Tramac/Fast-SCNN-pytorch.git
#  PREFIX ${FAST_SCNN_BUILD_DIR}
#  GIT_TAG 0638517d359ae1664a27dfb2cd1780a40a06c465
#  CONFIGURE_COMMAND ""
#  BUILD_COMMAND ""
#  INSTALL_COMMAND 
#    ${CMAKE_COMMAND} -E copy_directory ${FAST_SCNN_BUILD_DIR}/src/fast_scnn ${FAST_SCNN_INSTALL_DIR}/
#    && ${CMAKE_COMMAND} -E copy_directory ${FAST_SCNN_BUILD_DIR}/src/fast_scnn/weights ${CMAKE_CURRENT_SOURCE_DIR}/ckpt/fast_scnn_git
#  TEST_COMMAND ""
#)

#find_program(PYTHON "python")
## Add the Hang Zhang PyTorch-Encoding models from git
#set(ENCODING_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/pytorch_encoding)
#set(ENCODING_INSTALL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/encoding)
#set(ENCODING_INCLUDE_DIR ${ENCODING_INSTALL_DIR})
#ExternalProject_Add(
#  pytorch_encoding
#  GIT_REPOSITORY https://github.com/zhanghang1989/PyTorch-Encoding.git
#  PREFIX ${ENCODING_BUILD_DIR}
#  BINARY_DIR ${ENCODING_BUILD_DIR}/src/pytorch_encoding
#  GIT_TAG 331ecdd5306104614cb414b16fbcd9d1a8d40e1e
#  CONFIGURE_COMMAND ""
#  BUILD_COMMAND 
#    ${PYTHON} ${ENCODING_BUILD_DIR}/src/pytorch_encoding/setup.py build
#  INSTALL_COMMAND 
#    ${CMAKE_COMMAND} -E copy_directory ${ENCODING_BUILD_DIR}/src/pytorch_encoding/build/lib.linux-x86_64-3.8/encoding ${ENCODING_INSTALL_DIR}
#  TEST_COMMAND ""
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

catkin_install_python(PROGRAMS
  scripts/eval_semseg.py
  scripts/semseg_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


