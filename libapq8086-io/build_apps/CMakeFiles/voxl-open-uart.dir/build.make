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
CMAKE_SOURCE_DIR = /home/root/lib/apps

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/root/build_apps

# Include any dependencies generated for this target.
include CMakeFiles/voxl-open-uart.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/voxl-open-uart.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/voxl-open-uart.dir/flags.make

CMakeFiles/voxl-open-uart.dir/open-uart.c.o: CMakeFiles/voxl-open-uart.dir/flags.make
CMakeFiles/voxl-open-uart.dir/open-uart.c.o: /home/root/lib/apps/open-uart.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/root/build_apps/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/voxl-open-uart.dir/open-uart.c.o"
	/opt/Qualcomm/ARM_Tools/gcc-4.9-2014.11/bin/arm-linux-gnueabi-gcc --sysroot=/opt/Qualcomm/ARM_Tools/gcc-4.9-2014.11/libc/ $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/voxl-open-uart.dir/open-uart.c.o   -c /home/root/lib/apps/open-uart.c

CMakeFiles/voxl-open-uart.dir/open-uart.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/voxl-open-uart.dir/open-uart.c.i"
	/opt/Qualcomm/ARM_Tools/gcc-4.9-2014.11/bin/arm-linux-gnueabi-gcc --sysroot=/opt/Qualcomm/ARM_Tools/gcc-4.9-2014.11/libc/ $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/root/lib/apps/open-uart.c > CMakeFiles/voxl-open-uart.dir/open-uart.c.i

CMakeFiles/voxl-open-uart.dir/open-uart.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/voxl-open-uart.dir/open-uart.c.s"
	/opt/Qualcomm/ARM_Tools/gcc-4.9-2014.11/bin/arm-linux-gnueabi-gcc --sysroot=/opt/Qualcomm/ARM_Tools/gcc-4.9-2014.11/libc/ $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/root/lib/apps/open-uart.c -o CMakeFiles/voxl-open-uart.dir/open-uart.c.s

CMakeFiles/voxl-open-uart.dir/open-uart.c.o.requires:

.PHONY : CMakeFiles/voxl-open-uart.dir/open-uart.c.o.requires

CMakeFiles/voxl-open-uart.dir/open-uart.c.o.provides: CMakeFiles/voxl-open-uart.dir/open-uart.c.o.requires
	$(MAKE) -f CMakeFiles/voxl-open-uart.dir/build.make CMakeFiles/voxl-open-uart.dir/open-uart.c.o.provides.build
.PHONY : CMakeFiles/voxl-open-uart.dir/open-uart.c.o.provides

CMakeFiles/voxl-open-uart.dir/open-uart.c.o.provides.build: CMakeFiles/voxl-open-uart.dir/open-uart.c.o


# Object files for target voxl-open-uart
voxl__open__uart_OBJECTS = \
"CMakeFiles/voxl-open-uart.dir/open-uart.c.o"

# External object files for target voxl-open-uart
voxl__open__uart_EXTERNAL_OBJECTS =

voxl-open-uart: CMakeFiles/voxl-open-uart.dir/open-uart.c.o
voxl-open-uart: CMakeFiles/voxl-open-uart.dir/build.make
voxl-open-uart: libvoxl_io.so
voxl-open-uart: CMakeFiles/voxl-open-uart.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/root/build_apps/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable voxl-open-uart"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxl-open-uart.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/voxl-open-uart.dir/build: voxl-open-uart

.PHONY : CMakeFiles/voxl-open-uart.dir/build

# Object files for target voxl-open-uart
voxl__open__uart_OBJECTS = \
"CMakeFiles/voxl-open-uart.dir/open-uart.c.o"

# External object files for target voxl-open-uart
voxl__open__uart_EXTERNAL_OBJECTS =

CMakeFiles/CMakeRelink.dir/voxl-open-uart: CMakeFiles/voxl-open-uart.dir/open-uart.c.o
CMakeFiles/CMakeRelink.dir/voxl-open-uart: CMakeFiles/voxl-open-uart.dir/build.make
CMakeFiles/CMakeRelink.dir/voxl-open-uart: libvoxl_io.so
CMakeFiles/CMakeRelink.dir/voxl-open-uart: CMakeFiles/voxl-open-uart.dir/relink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/root/build_apps/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable CMakeFiles/CMakeRelink.dir/voxl-open-uart"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxl-open-uart.dir/relink.txt --verbose=$(VERBOSE)

# Rule to relink during preinstall.
CMakeFiles/voxl-open-uart.dir/preinstall: CMakeFiles/CMakeRelink.dir/voxl-open-uart

.PHONY : CMakeFiles/voxl-open-uart.dir/preinstall

CMakeFiles/voxl-open-uart.dir/requires: CMakeFiles/voxl-open-uart.dir/open-uart.c.o.requires

.PHONY : CMakeFiles/voxl-open-uart.dir/requires

CMakeFiles/voxl-open-uart.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/voxl-open-uart.dir/cmake_clean.cmake
.PHONY : CMakeFiles/voxl-open-uart.dir/clean

CMakeFiles/voxl-open-uart.dir/depend:
	cd /home/root/build_apps && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/root/lib/apps /home/root/lib/apps /home/root/build_apps /home/root/build_apps /home/root/build_apps/CMakeFiles/voxl-open-uart.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/voxl-open-uart.dir/depend

