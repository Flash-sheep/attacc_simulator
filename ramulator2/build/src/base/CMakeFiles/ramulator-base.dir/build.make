# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/long/attacc_simulator/ramulator2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/long/attacc_simulator/ramulator2/build

# Include any dependencies generated for this target.
include src/base/CMakeFiles/ramulator-base.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/base/CMakeFiles/ramulator-base.dir/compiler_depend.make

# Include the progress variables for this target.
include src/base/CMakeFiles/ramulator-base.dir/progress.make

# Include the compile flags for this target's objects.
include src/base/CMakeFiles/ramulator-base.dir/flags.make

src/base/CMakeFiles/ramulator-base.dir/factory.cpp.o: src/base/CMakeFiles/ramulator-base.dir/flags.make
src/base/CMakeFiles/ramulator-base.dir/factory.cpp.o: ../src/base/factory.cpp
src/base/CMakeFiles/ramulator-base.dir/factory.cpp.o: src/base/CMakeFiles/ramulator-base.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/long/attacc_simulator/ramulator2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/base/CMakeFiles/ramulator-base.dir/factory.cpp.o"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/base/CMakeFiles/ramulator-base.dir/factory.cpp.o -MF CMakeFiles/ramulator-base.dir/factory.cpp.o.d -o CMakeFiles/ramulator-base.dir/factory.cpp.o -c /home/long/attacc_simulator/ramulator2/src/base/factory.cpp

src/base/CMakeFiles/ramulator-base.dir/factory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ramulator-base.dir/factory.cpp.i"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/long/attacc_simulator/ramulator2/src/base/factory.cpp > CMakeFiles/ramulator-base.dir/factory.cpp.i

src/base/CMakeFiles/ramulator-base.dir/factory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ramulator-base.dir/factory.cpp.s"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/long/attacc_simulator/ramulator2/src/base/factory.cpp -o CMakeFiles/ramulator-base.dir/factory.cpp.s

src/base/CMakeFiles/ramulator-base.dir/logging.cpp.o: src/base/CMakeFiles/ramulator-base.dir/flags.make
src/base/CMakeFiles/ramulator-base.dir/logging.cpp.o: ../src/base/logging.cpp
src/base/CMakeFiles/ramulator-base.dir/logging.cpp.o: src/base/CMakeFiles/ramulator-base.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/long/attacc_simulator/ramulator2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/base/CMakeFiles/ramulator-base.dir/logging.cpp.o"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/base/CMakeFiles/ramulator-base.dir/logging.cpp.o -MF CMakeFiles/ramulator-base.dir/logging.cpp.o.d -o CMakeFiles/ramulator-base.dir/logging.cpp.o -c /home/long/attacc_simulator/ramulator2/src/base/logging.cpp

src/base/CMakeFiles/ramulator-base.dir/logging.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ramulator-base.dir/logging.cpp.i"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/long/attacc_simulator/ramulator2/src/base/logging.cpp > CMakeFiles/ramulator-base.dir/logging.cpp.i

src/base/CMakeFiles/ramulator-base.dir/logging.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ramulator-base.dir/logging.cpp.s"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/long/attacc_simulator/ramulator2/src/base/logging.cpp -o CMakeFiles/ramulator-base.dir/logging.cpp.s

src/base/CMakeFiles/ramulator-base.dir/utils.cpp.o: src/base/CMakeFiles/ramulator-base.dir/flags.make
src/base/CMakeFiles/ramulator-base.dir/utils.cpp.o: ../src/base/utils.cpp
src/base/CMakeFiles/ramulator-base.dir/utils.cpp.o: src/base/CMakeFiles/ramulator-base.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/long/attacc_simulator/ramulator2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/base/CMakeFiles/ramulator-base.dir/utils.cpp.o"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/base/CMakeFiles/ramulator-base.dir/utils.cpp.o -MF CMakeFiles/ramulator-base.dir/utils.cpp.o.d -o CMakeFiles/ramulator-base.dir/utils.cpp.o -c /home/long/attacc_simulator/ramulator2/src/base/utils.cpp

src/base/CMakeFiles/ramulator-base.dir/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ramulator-base.dir/utils.cpp.i"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/long/attacc_simulator/ramulator2/src/base/utils.cpp > CMakeFiles/ramulator-base.dir/utils.cpp.i

src/base/CMakeFiles/ramulator-base.dir/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ramulator-base.dir/utils.cpp.s"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/long/attacc_simulator/ramulator2/src/base/utils.cpp -o CMakeFiles/ramulator-base.dir/utils.cpp.s

src/base/CMakeFiles/ramulator-base.dir/config.cpp.o: src/base/CMakeFiles/ramulator-base.dir/flags.make
src/base/CMakeFiles/ramulator-base.dir/config.cpp.o: ../src/base/config.cpp
src/base/CMakeFiles/ramulator-base.dir/config.cpp.o: src/base/CMakeFiles/ramulator-base.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/long/attacc_simulator/ramulator2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/base/CMakeFiles/ramulator-base.dir/config.cpp.o"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/base/CMakeFiles/ramulator-base.dir/config.cpp.o -MF CMakeFiles/ramulator-base.dir/config.cpp.o.d -o CMakeFiles/ramulator-base.dir/config.cpp.o -c /home/long/attacc_simulator/ramulator2/src/base/config.cpp

src/base/CMakeFiles/ramulator-base.dir/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ramulator-base.dir/config.cpp.i"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/long/attacc_simulator/ramulator2/src/base/config.cpp > CMakeFiles/ramulator-base.dir/config.cpp.i

src/base/CMakeFiles/ramulator-base.dir/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ramulator-base.dir/config.cpp.s"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/long/attacc_simulator/ramulator2/src/base/config.cpp -o CMakeFiles/ramulator-base.dir/config.cpp.s

src/base/CMakeFiles/ramulator-base.dir/stats.cpp.o: src/base/CMakeFiles/ramulator-base.dir/flags.make
src/base/CMakeFiles/ramulator-base.dir/stats.cpp.o: ../src/base/stats.cpp
src/base/CMakeFiles/ramulator-base.dir/stats.cpp.o: src/base/CMakeFiles/ramulator-base.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/long/attacc_simulator/ramulator2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/base/CMakeFiles/ramulator-base.dir/stats.cpp.o"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/base/CMakeFiles/ramulator-base.dir/stats.cpp.o -MF CMakeFiles/ramulator-base.dir/stats.cpp.o.d -o CMakeFiles/ramulator-base.dir/stats.cpp.o -c /home/long/attacc_simulator/ramulator2/src/base/stats.cpp

src/base/CMakeFiles/ramulator-base.dir/stats.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ramulator-base.dir/stats.cpp.i"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/long/attacc_simulator/ramulator2/src/base/stats.cpp > CMakeFiles/ramulator-base.dir/stats.cpp.i

src/base/CMakeFiles/ramulator-base.dir/stats.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ramulator-base.dir/stats.cpp.s"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/long/attacc_simulator/ramulator2/src/base/stats.cpp -o CMakeFiles/ramulator-base.dir/stats.cpp.s

src/base/CMakeFiles/ramulator-base.dir/request.cpp.o: src/base/CMakeFiles/ramulator-base.dir/flags.make
src/base/CMakeFiles/ramulator-base.dir/request.cpp.o: ../src/base/request.cpp
src/base/CMakeFiles/ramulator-base.dir/request.cpp.o: src/base/CMakeFiles/ramulator-base.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/long/attacc_simulator/ramulator2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/base/CMakeFiles/ramulator-base.dir/request.cpp.o"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/base/CMakeFiles/ramulator-base.dir/request.cpp.o -MF CMakeFiles/ramulator-base.dir/request.cpp.o.d -o CMakeFiles/ramulator-base.dir/request.cpp.o -c /home/long/attacc_simulator/ramulator2/src/base/request.cpp

src/base/CMakeFiles/ramulator-base.dir/request.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ramulator-base.dir/request.cpp.i"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/long/attacc_simulator/ramulator2/src/base/request.cpp > CMakeFiles/ramulator-base.dir/request.cpp.i

src/base/CMakeFiles/ramulator-base.dir/request.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ramulator-base.dir/request.cpp.s"
	cd /home/long/attacc_simulator/ramulator2/build/src/base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/long/attacc_simulator/ramulator2/src/base/request.cpp -o CMakeFiles/ramulator-base.dir/request.cpp.s

ramulator-base: src/base/CMakeFiles/ramulator-base.dir/factory.cpp.o
ramulator-base: src/base/CMakeFiles/ramulator-base.dir/logging.cpp.o
ramulator-base: src/base/CMakeFiles/ramulator-base.dir/utils.cpp.o
ramulator-base: src/base/CMakeFiles/ramulator-base.dir/config.cpp.o
ramulator-base: src/base/CMakeFiles/ramulator-base.dir/stats.cpp.o
ramulator-base: src/base/CMakeFiles/ramulator-base.dir/request.cpp.o
ramulator-base: src/base/CMakeFiles/ramulator-base.dir/build.make
.PHONY : ramulator-base

# Rule to build all files generated by this target.
src/base/CMakeFiles/ramulator-base.dir/build: ramulator-base
.PHONY : src/base/CMakeFiles/ramulator-base.dir/build

src/base/CMakeFiles/ramulator-base.dir/clean:
	cd /home/long/attacc_simulator/ramulator2/build/src/base && $(CMAKE_COMMAND) -P CMakeFiles/ramulator-base.dir/cmake_clean.cmake
.PHONY : src/base/CMakeFiles/ramulator-base.dir/clean

src/base/CMakeFiles/ramulator-base.dir/depend:
	cd /home/long/attacc_simulator/ramulator2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/long/attacc_simulator/ramulator2 /home/long/attacc_simulator/ramulator2/src/base /home/long/attacc_simulator/ramulator2/build /home/long/attacc_simulator/ramulator2/build/src/base /home/long/attacc_simulator/ramulator2/build/src/base/CMakeFiles/ramulator-base.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/base/CMakeFiles/ramulator-base.dir/depend

