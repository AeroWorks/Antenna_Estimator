# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Maverick/Documents/GeographicLib-1.42

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Maverick/Documents/GeographicLib-1.42/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-TransverseMercatorExact.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example-TransverseMercatorExact.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-TransverseMercatorExact.dir/flags.make

examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o: examples/CMakeFiles/example-TransverseMercatorExact.dir/flags.make
examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o: ../examples/example-TransverseMercatorExact.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/Maverick/Documents/GeographicLib-1.42/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/examples && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o -c /Users/Maverick/Documents/GeographicLib-1.42/examples/example-TransverseMercatorExact.cpp

examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.i"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/examples && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/Maverick/Documents/GeographicLib-1.42/examples/example-TransverseMercatorExact.cpp > CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.i

examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.s"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/examples && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/Maverick/Documents/GeographicLib-1.42/examples/example-TransverseMercatorExact.cpp -o CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.s

examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.requires:
.PHONY : examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.requires

examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.provides: examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/example-TransverseMercatorExact.dir/build.make examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.provides.build
.PHONY : examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.provides

examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.provides.build: examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o

# Object files for target example-TransverseMercatorExact
example__TransverseMercatorExact_OBJECTS = \
"CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o"

# External object files for target example-TransverseMercatorExact
example__TransverseMercatorExact_EXTERNAL_OBJECTS =

examples/example-TransverseMercatorExact: examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o
examples/example-TransverseMercatorExact: examples/CMakeFiles/example-TransverseMercatorExact.dir/build.make
examples/example-TransverseMercatorExact: src/libGeographic.14.0.3.dylib
examples/example-TransverseMercatorExact: examples/CMakeFiles/example-TransverseMercatorExact.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable example-TransverseMercatorExact"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-TransverseMercatorExact.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-TransverseMercatorExact.dir/build: examples/example-TransverseMercatorExact
.PHONY : examples/CMakeFiles/example-TransverseMercatorExact.dir/build

examples/CMakeFiles/example-TransverseMercatorExact.dir/requires: examples/CMakeFiles/example-TransverseMercatorExact.dir/example-TransverseMercatorExact.cpp.o.requires
.PHONY : examples/CMakeFiles/example-TransverseMercatorExact.dir/requires

examples/CMakeFiles/example-TransverseMercatorExact.dir/clean:
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-TransverseMercatorExact.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-TransverseMercatorExact.dir/clean

examples/CMakeFiles/example-TransverseMercatorExact.dir/depend:
	cd /Users/Maverick/Documents/GeographicLib-1.42/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Maverick/Documents/GeographicLib-1.42 /Users/Maverick/Documents/GeographicLib-1.42/examples /Users/Maverick/Documents/GeographicLib-1.42/build /Users/Maverick/Documents/GeographicLib-1.42/build/examples /Users/Maverick/Documents/GeographicLib-1.42/build/examples/CMakeFiles/example-TransverseMercatorExact.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-TransverseMercatorExact.dir/depend
