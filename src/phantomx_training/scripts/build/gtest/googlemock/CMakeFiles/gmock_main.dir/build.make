# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/tania/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/tania/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tania/Codigos/phantomx/src/phantomx_training/scripts

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build

# Include any dependencies generated for this target.
include gtest/googlemock/CMakeFiles/gmock_main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include gtest/googlemock/CMakeFiles/gmock_main.dir/compiler_depend.make

# Include the progress variables for this target.
include gtest/googlemock/CMakeFiles/gmock_main.dir/progress.make

# Include the compile flags for this target's objects.
include gtest/googlemock/CMakeFiles/gmock_main.dir/flags.make

gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: gtest/googlemock/CMakeFiles/gmock_main.dir/flags.make
gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: /usr/src/googletest/googlemock/src/gmock_main.cc
gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: gtest/googlemock/CMakeFiles/gmock_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"
	cd /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -MF CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.d -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -c /usr/src/googletest/googlemock/src/gmock_main.cc

gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock_main.dir/src/gmock_main.cc.i"
	cd /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/src/googletest/googlemock/src/gmock_main.cc > CMakeFiles/gmock_main.dir/src/gmock_main.cc.i

gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock_main.dir/src/gmock_main.cc.s"
	cd /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/src/googletest/googlemock/src/gmock_main.cc -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.s

# Object files for target gmock_main
gmock_main_OBJECTS = \
"CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"

# External object files for target gmock_main
gmock_main_EXTERNAL_OBJECTS =

gtest/lib/libgmock_main.so: gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o
gtest/lib/libgmock_main.so: gtest/googlemock/CMakeFiles/gmock_main.dir/build.make
gtest/lib/libgmock_main.so: gtest/lib/libgmock.so
gtest/lib/libgmock_main.so: gtest/lib/libgtest.so
gtest/lib/libgmock_main.so: gtest/googlemock/CMakeFiles/gmock_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../lib/libgmock_main.so"
	cd /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gmock_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gtest/googlemock/CMakeFiles/gmock_main.dir/build: gtest/lib/libgmock_main.so
.PHONY : gtest/googlemock/CMakeFiles/gmock_main.dir/build

gtest/googlemock/CMakeFiles/gmock_main.dir/clean:
	cd /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock && $(CMAKE_COMMAND) -P CMakeFiles/gmock_main.dir/cmake_clean.cmake
.PHONY : gtest/googlemock/CMakeFiles/gmock_main.dir/clean

gtest/googlemock/CMakeFiles/gmock_main.dir/depend:
	cd /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tania/Codigos/phantomx/src/phantomx_training/scripts /usr/src/googletest/googlemock /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock /home/tania/Codigos/phantomx/src/phantomx_training/scripts/build/gtest/googlemock/CMakeFiles/gmock_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtest/googlemock/CMakeFiles/gmock_main.dir/depend

