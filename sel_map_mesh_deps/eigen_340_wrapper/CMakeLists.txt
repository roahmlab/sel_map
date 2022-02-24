cmake_minimum_required(VERSION 3.0.2)
project(eigen_340_wrapper)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

# Find Eigen 3.4, or download if not found
find_package (Eigen3 3.4 NO_MODULE) 
if (TARGET Eigen3::Eigen)
  # Use the imported target
  # Tell the catkin_package to include this.
  catkin_package(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

  # Locations of header files
  include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

else()
  # Download and set it up in this wrapper using ExternalProject
  include(ExternalProject)

  ## Add Eigen3.4 using ExternalProject
  set(EIGEN3_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/eigen)
  set(EIGEN3_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INSTALL_DIR})
  file(MAKE_DIRECTORY ${EIGEN3_INCLUDE_DIR})
  ExternalProject_Add(
    eigen_340
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    PREFIX ${EIGEN3_BUILD_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E copy_directory ${EIGEN3_BUILD_DIR}/src/eigen_340/Eigen ${EIGEN3_INCLUDE_DIR}/Eigen 
      && ${CMAKE_COMMAND} -E copy_directory ${EIGEN3_BUILD_DIR}/src/eigen_340/unsupported ${EIGEN3_INCLUDE_DIR}/unsupported
    TEST_COMMAND ""
  )
  # Tell the catkin_package to include this.
  catkin_package(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

  # Locations of header files
  include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  # Mark for installation
  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

endif (TARGET Eigen3::Eigen)

