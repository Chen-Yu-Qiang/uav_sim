# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/yuqiang/catkin_ws2/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuqiang/catkin_ws2/build

# Utility rule file for ardronelib.

# Include the progress variables for this target.
include ardrone_autonomy/CMakeFiles/ardronelib.dir/progress.make

ardrone_autonomy/CMakeFiles/ardronelib: ardrone_autonomy/CMakeFiles/ardronelib-complete


ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-install
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-mkdir
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-update
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-patch
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-build
ardrone_autonomy/CMakeFiles/ardronelib-complete: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/build/ardrone_autonomy/CMakeFiles
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/build/ardrone_autonomy/CMakeFiles/ardronelib-complete
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-done

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-install: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && make install INSTALL_PREFIX=/home/yuqiang/catkin_ws2/devel/lib/ardrone
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-install

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/devel/src/ardronelib
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/devel/src/ardronelib
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/devel
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/devel/tmp
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E make_directory /home/yuqiang/catkin_ws2/devel/src
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-mkdir

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-gitinfo.txt
/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/devel/src && /usr/bin/cmake -P /home/yuqiang/catkin_ws2/devel/tmp/ardronelib-gitclone.cmake
	cd /home/yuqiang/catkin_ws2/devel/src && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-update: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && /usr/bin/cmake -P /home/yuqiang/catkin_ws2/devel/tmp/ardronelib-gitupdate.cmake

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-patch: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E echo_append
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-patch

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure: /home/yuqiang/catkin_ws2/devel/tmp/ardronelib-cfgcmd.txt
/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-update
/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && echo "No configure"
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure

/home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-build: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuqiang/catkin_ws2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'ardronelib'"
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && make
	cd /home/yuqiang/catkin_ws2/devel/src/ardronelib && /usr/bin/cmake -E touch /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-build

ardronelib: ardrone_autonomy/CMakeFiles/ardronelib
ardronelib: ardrone_autonomy/CMakeFiles/ardronelib-complete
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-install
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-mkdir
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-download
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-update
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-patch
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-configure
ardronelib: /home/yuqiang/catkin_ws2/devel/src/ardronelib-stamp/ardronelib-build
ardronelib: ardrone_autonomy/CMakeFiles/ardronelib.dir/build.make

.PHONY : ardronelib

# Rule to build all files generated by this target.
ardrone_autonomy/CMakeFiles/ardronelib.dir/build: ardronelib

.PHONY : ardrone_autonomy/CMakeFiles/ardronelib.dir/build

ardrone_autonomy/CMakeFiles/ardronelib.dir/clean:
	cd /home/yuqiang/catkin_ws2/build/ardrone_autonomy && $(CMAKE_COMMAND) -P CMakeFiles/ardronelib.dir/cmake_clean.cmake
.PHONY : ardrone_autonomy/CMakeFiles/ardronelib.dir/clean

ardrone_autonomy/CMakeFiles/ardronelib.dir/depend:
	cd /home/yuqiang/catkin_ws2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuqiang/catkin_ws2/src /home/yuqiang/catkin_ws2/src/ardrone_autonomy /home/yuqiang/catkin_ws2/build /home/yuqiang/catkin_ws2/build/ardrone_autonomy /home/yuqiang/catkin_ws2/build/ardrone_autonomy/CMakeFiles/ardronelib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ardrone_autonomy/CMakeFiles/ardronelib.dir/depend

