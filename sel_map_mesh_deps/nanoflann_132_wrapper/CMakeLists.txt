cmake_minimum_required(VERSION 3.0.2)
project(nanoflann_132_wrapper)

## Find catkin macros and libraries
find_package(catkin REQUIRED eigen_340_wrapper)

# Find nanoflann, or download if not found
find_package (nanoflann NO_MODULE) 
if (TARGET nanoflann::nanoflann)
  # Use the imported target
  # Tell the catkin_package to include this.
  catkin_package(INCLUDE_DIRS ${nanoflann_INCLUDE_DIR})

  # Locations of header files
  include_directories(SYSTEM ${nanoflann_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

else()
  # Download and set it up in this wrapper using ExternalProject
  include(ExternalProject)

  ## Nanoflann 1.3.2 using ExternalProject
  set(NANOFLANN_BUILD_DIR   ${CMAKE_BINARY_DIR}/ext_src/nanoflann)
  set(NANOFLANN_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set(NANOFLANN_INCLUDE_DIR ${NANOFLANN_INSTALL_DIR})
  file(MAKE_DIRECTORY ${NANOFLANN_INCLUDE_DIR})
  ExternalProject_Add(
    nanoflann_132
    URL https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.3.2.tar.gz
    PREFIX ${NANOFLANN_BUILD_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E copy_directory ${NANOFLANN_BUILD_DIR}/src/nanoflann_132/include ${NANOFLANN_INCLUDE_DIR}/nanoflann
    TEST_COMMAND ""
  )
  # Tell the catkin_package to include this.
  catkin_package(INCLUDE_DIRS ${NANOFLANN_INCLUDE_DIR})

  # Locations of header files
  include_directories(${NANOFLANN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  # Mark for installation
  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

endif (TARGET nanoflann::nanoflann)
