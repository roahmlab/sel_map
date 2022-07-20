This version captures and modifies from commit `cab2a59fc8abdcd022748d07d5c8ed6358ae3e5b` from https://github.com/uos/mesh_tools.

If running on a system which captures an out of date version of mesh_tools, you can also clone this specific version into `catkin_ws/src` and add a `CATKIN_IGNORE` file to the resulting `catkin_ws/src/mesh_tools/rviz_map_plugin/` folder to ensure this version of rviz_map_plugin is built instead of the source version.

Step-by-step to do this instructions are as follows:

1. `cd catkin_ws/src`
2. `git clone -n https://github.com/uos/mesh_tools`
3. `git checkout cab2a59fc8abdcd022748d07d5c8ed6358ae3e5b`
4. `touch mesh_tools/rviz_map_plugin/CATKIN_IGNORE`
