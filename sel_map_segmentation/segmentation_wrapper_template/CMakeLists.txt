cmake_minimum_required(VERSION 3.0.2)
project(segmentation_wrapper_template)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

##################################################
## Clone the segmentation network from git here ##
##################################################

# You can check the other 3 included networks for examples of how to have
# catkin clone an external repository and move it to a specific folder.

###################################
## catkin specific configuration ##
###################################
catkin_package()

