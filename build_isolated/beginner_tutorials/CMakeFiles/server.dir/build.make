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
CMAKE_SOURCE_DIR = /home/parallels/catkin_ws/src/beginner_tutorials

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/catkin_ws/build_isolated/beginner_tutorials

# Include any dependencies generated for this target.
include CMakeFiles/server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/server.dir/flags.make

CMakeFiles/server.dir/src/server.cpp.o: CMakeFiles/server.dir/flags.make
CMakeFiles/server.dir/src/server.cpp.o: /home/parallels/catkin_ws/src/beginner_tutorials/src/server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/catkin_ws/build_isolated/beginner_tutorials/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/server.dir/src/server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/server.dir/src/server.cpp.o -c /home/parallels/catkin_ws/src/beginner_tutorials/src/server.cpp

CMakeFiles/server.dir/src/server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/server.dir/src/server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/catkin_ws/src/beginner_tutorials/src/server.cpp > CMakeFiles/server.dir/src/server.cpp.i

CMakeFiles/server.dir/src/server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/server.dir/src/server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/catkin_ws/src/beginner_tutorials/src/server.cpp -o CMakeFiles/server.dir/src/server.cpp.s

# Object files for target server
server_OBJECTS = \
"CMakeFiles/server.dir/src/server.cpp.o"

# External object files for target server
server_EXTERNAL_OBJECTS =

/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: CMakeFiles/server.dir/src/server.cpp.o
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: CMakeFiles/server.dir/build.make
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/libroscpp.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/librosconsole.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/librostime.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /opt/ros/noetic/lib/libcpp_common.so
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server: CMakeFiles/server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/catkin_ws/build_isolated/beginner_tutorials/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/server.dir/build: /home/parallels/catkin_ws/devel_isolated/beginner_tutorials/lib/beginner_tutorials/server

.PHONY : CMakeFiles/server.dir/build

CMakeFiles/server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/server.dir/clean

CMakeFiles/server.dir/depend:
	cd /home/parallels/catkin_ws/build_isolated/beginner_tutorials && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/catkin_ws/src/beginner_tutorials /home/parallels/catkin_ws/src/beginner_tutorials /home/parallels/catkin_ws/build_isolated/beginner_tutorials /home/parallels/catkin_ws/build_isolated/beginner_tutorials /home/parallels/catkin_ws/build_isolated/beginner_tutorials/CMakeFiles/server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/server.dir/depend
