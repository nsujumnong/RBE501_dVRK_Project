# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davincic2/catkin_ws/build_release/saw_text_to_speech

# Include any dependencies generated for this target.
include CMakeFiles/sawTextToSpeech.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sawTextToSpeech.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sawTextToSpeech.dir/flags.make

CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o: CMakeFiles/sawTextToSpeech.dir/flags.make
CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o: /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components/code/mtsTextToSpeech.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/davincic2/catkin_ws/build_release/saw_text_to_speech/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o -c /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components/code/mtsTextToSpeech.cpp

CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components/code/mtsTextToSpeech.cpp > CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.i

CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components/code/mtsTextToSpeech.cpp -o CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.s

CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.requires:
.PHONY : CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.requires

CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.provides: CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.requires
	$(MAKE) -f CMakeFiles/sawTextToSpeech.dir/build.make CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.provides.build
.PHONY : CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.provides

CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.provides.build: CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o

# Object files for target sawTextToSpeech
sawTextToSpeech_OBJECTS = \
"CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o"

# External object files for target sawTextToSpeech
sawTextToSpeech_EXTERNAL_OBJECTS =

lib/libsawTextToSpeech.so: CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o
lib/libsawTextToSpeech.so: CMakeFiles/sawTextToSpeech.dir/build.make
lib/libsawTextToSpeech.so: /usr/lib/x86_64-linux-gnu/libGLU.so
lib/libsawTextToSpeech.so: /usr/lib/x86_64-linux-gnu/libGL.so
lib/libsawTextToSpeech.so: /usr/lib/x86_64-linux-gnu/libSM.so
lib/libsawTextToSpeech.so: /usr/lib/x86_64-linux-gnu/libICE.so
lib/libsawTextToSpeech.so: /usr/lib/x86_64-linux-gnu/libX11.so
lib/libsawTextToSpeech.so: /usr/lib/x86_64-linux-gnu/libXext.so
lib/libsawTextToSpeech.so: CMakeFiles/sawTextToSpeech.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library lib/libsawTextToSpeech.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sawTextToSpeech.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sawTextToSpeech.dir/build: lib/libsawTextToSpeech.so
.PHONY : CMakeFiles/sawTextToSpeech.dir/build

CMakeFiles/sawTextToSpeech.dir/requires: CMakeFiles/sawTextToSpeech.dir/code/mtsTextToSpeech.cpp.o.requires
.PHONY : CMakeFiles/sawTextToSpeech.dir/requires

CMakeFiles/sawTextToSpeech.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sawTextToSpeech.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sawTextToSpeech.dir/clean

CMakeFiles/sawTextToSpeech.dir/depend:
	cd /home/davincic2/catkin_ws/build_release/saw_text_to_speech && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components /home/davincic2/catkin_ws/src/cisst-saw/sawTextToSpeech/components /home/davincic2/catkin_ws/build_release/saw_text_to_speech /home/davincic2/catkin_ws/build_release/saw_text_to_speech /home/davincic2/catkin_ws/build_release/saw_text_to_speech/CMakeFiles/sawTextToSpeech.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sawTextToSpeech.dir/depend

