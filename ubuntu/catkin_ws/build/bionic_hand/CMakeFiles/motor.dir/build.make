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
CMAKE_SOURCE_DIR = /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build

# Include any dependencies generated for this target.
include bionic_hand/CMakeFiles/motor.dir/depend.make

# Include the progress variables for this target.
include bionic_hand/CMakeFiles/motor.dir/progress.make

# Include the compile flags for this target's objects.
include bionic_hand/CMakeFiles/motor.dir/flags.make

bionic_hand/CMakeFiles/motor.dir/src/motor.cpp.o: bionic_hand/CMakeFiles/motor.dir/flags.make
bionic_hand/CMakeFiles/motor.dir/src/motor.cpp.o: /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object bionic_hand/CMakeFiles/motor.dir/src/motor.cpp.o"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motor.dir/src/motor.cpp.o -c /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/motor.cpp

bionic_hand/CMakeFiles/motor.dir/src/motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor.dir/src/motor.cpp.i"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/motor.cpp > CMakeFiles/motor.dir/src/motor.cpp.i

bionic_hand/CMakeFiles/motor.dir/src/motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor.dir/src/motor.cpp.s"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/motor.cpp -o CMakeFiles/motor.dir/src/motor.cpp.s

bionic_hand/CMakeFiles/motor.dir/src/move_motor.cpp.o: bionic_hand/CMakeFiles/motor.dir/flags.make
bionic_hand/CMakeFiles/motor.dir/src/move_motor.cpp.o: /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/move_motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object bionic_hand/CMakeFiles/motor.dir/src/move_motor.cpp.o"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motor.dir/src/move_motor.cpp.o -c /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/move_motor.cpp

bionic_hand/CMakeFiles/motor.dir/src/move_motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor.dir/src/move_motor.cpp.i"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/move_motor.cpp > CMakeFiles/motor.dir/src/move_motor.cpp.i

bionic_hand/CMakeFiles/motor.dir/src/move_motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor.dir/src/move_motor.cpp.s"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/move_motor.cpp -o CMakeFiles/motor.dir/src/move_motor.cpp.s

# Object files for target motor
motor_OBJECTS = \
"CMakeFiles/motor.dir/src/motor.cpp.o" \
"CMakeFiles/motor.dir/src/move_motor.cpp.o"

# External object files for target motor
motor_EXTERNAL_OBJECTS =

/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: bionic_hand/CMakeFiles/motor.dir/src/motor.cpp.o
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: bionic_hand/CMakeFiles/motor.dir/src/move_motor.cpp.o
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: bionic_hand/CMakeFiles/motor.dir/build.make
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/libdynamixel_sdk.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/libroscpp.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/librosconsole.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/librostime.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /opt/ros/noetic/lib/libcpp_common.so
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor: bionic_hand/CMakeFiles/motor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor"
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bionic_hand/CMakeFiles/motor.dir/build: /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/bionic_hand/motor

.PHONY : bionic_hand/CMakeFiles/motor.dir/build

bionic_hand/CMakeFiles/motor.dir/clean:
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand && $(CMAKE_COMMAND) -P CMakeFiles/motor.dir/cmake_clean.cmake
.PHONY : bionic_hand/CMakeFiles/motor.dir/clean

bionic_hand/CMakeFiles/motor.dir/depend:
	cd /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand/CMakeFiles/motor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : bionic_hand/CMakeFiles/motor.dir/depend
