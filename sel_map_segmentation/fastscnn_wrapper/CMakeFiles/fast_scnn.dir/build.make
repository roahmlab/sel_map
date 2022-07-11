# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/firefly/ROAHM/sel_map_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/firefly/ROAHM/sel_map_ws/src

# Utility rule file for fast_scnn.

# Include the progress variables for this target.
include sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/progress.make

sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn: sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete


sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-install
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-mkdir
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-update
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-patch
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-build
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-install
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-done

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-install: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E copy_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper/src/fast_scnn/
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-install

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/tmp
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E make_directory /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-mkdir

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-gitinfo.txt
ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src && /usr/bin/cmake -P /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/tmp/fast_scnn-gitclone.cmake
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-update: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn && /usr/bin/cmake -P /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/tmp/fast_scnn-gitupdate.cmake

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-patch: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E echo_append
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-patch

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure: ext_src/fast_scnn/tmp/fast_scnn-cfgcmd.txt
ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-update
ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No configure step for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E echo_append
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-build: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No build step for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E echo_append
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-build

ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-test: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/firefly/ROAHM/sel_map_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "No test step for 'fast_scnn'"
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E echo_append
	cd /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-build && /usr/bin/cmake -E touch /home/firefly/ROAHM/sel_map_ws/src/ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-test

fast_scnn: sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn
fast_scnn: sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn-complete
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-install
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-mkdir
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-download
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-update
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-patch
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-configure
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-build
fast_scnn: ext_src/fast_scnn/src/fast_scnn-stamp/fast_scnn-test
fast_scnn: sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/build.make

.PHONY : fast_scnn

# Rule to build all files generated by this target.
sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/build: fast_scnn

.PHONY : sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/build

sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/clean:
	cd /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper && $(CMAKE_COMMAND) -P CMakeFiles/fast_scnn.dir/cmake_clean.cmake
.PHONY : sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/clean

sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/depend:
	cd /home/firefly/ROAHM/sel_map_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/firefly/ROAHM/sel_map_ws/src /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper /home/firefly/ROAHM/sel_map_ws/src /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper /home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sel_map/sel_map_segmentation/fastscnn_wrapper/CMakeFiles/fast_scnn.dir/depend

