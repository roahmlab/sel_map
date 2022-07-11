execute_process(COMMAND "/home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_mesh/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_mesh/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
