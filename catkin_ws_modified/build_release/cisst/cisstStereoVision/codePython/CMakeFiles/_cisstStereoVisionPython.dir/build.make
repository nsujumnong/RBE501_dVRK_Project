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
CMAKE_SOURCE_DIR = /home/davincic2/catkin_ws/src/cisst-saw/cisst

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davincic2/catkin_ws/build_release/cisst

# Include any dependencies generated for this target.
include cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/depend.make

# Include the progress variables for this target.
include cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/progress.make

# Include the compile flags for this target's objects.
include cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/flags.make

cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx: /home/davincic2/catkin_ws/src/cisst-saw/cisst/cisstStereoVision/codePython/../cisstStereoVision.i
	$(CMAKE_COMMAND) -E cmake_progress_report /home/davincic2/catkin_ws/build_release/cisst/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Swig source"
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/cmake -E make_directory /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/swig2.0 -python -v -modern -fcompact -fvirtual -outdir /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython -c++ -I/home/davincic2/catkin_ws/build_release/cisst/include -I/home/davincic2/catkin_ws/src/cisst-saw/cisst -I/home/davincic2/catkin_ws/build_release/cisst/include -I/home/davincic2/catkin_ws/src/cisst-saw/cisst -I/usr/include -I/usr/include/opencv -I/usr/include -I/usr/include -I/usr/include -I/usr/include -I/usr/include -I/usr/include -I/usr/include -I/usr/include -I/usr/include -I/home/davincic2/catkin_ws/build_release/cisst/cisstJSON/include -I/usr/include -I/home/davincic2/catkin_ws/devel_release/include -I/usr/include/python2.7 -I/usr/lib/python2.7/dist-packages/numpy/core/include/numpy -I/usr/include/opencv -I/usr/include -o /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython//../cisstStereoVisionPYTHON_wrap.cxx /home/davincic2/catkin_ws/src/cisst-saw/cisst/cisstStereoVision/codePython/../cisstStereoVision.i

cisstStereoVision/codePython/cisstStereoVision.py: cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/flags.make
cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o: cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report /home/davincic2/catkin_ws/build_release/cisst/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o"
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o -c /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.i"
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx > CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.i

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.s"
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx -o CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.s

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.requires:
.PHONY : cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.requires

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.provides: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.requires
	$(MAKE) -f cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/build.make cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.provides.build
.PHONY : cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.provides

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.provides.build: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o

# Object files for target _cisstStereoVisionPython
_cisstStereoVisionPython_OBJECTS = \
"CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o"

# External object files for target _cisstStereoVisionPython
_cisstStereoVisionPython_EXTERNAL_OBJECTS =

lib/_cisstStereoVisionPython.so: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o
lib/_cisstStereoVisionPython.so: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/build.make
lib/_cisstStereoVisionPython.so: lib/libcisstStereoVision.so
lib/_cisstStereoVisionPython.so: lib/libcisstCommon.so
lib/_cisstStereoVisionPython.so: lib/libcisstVector.so
lib/_cisstStereoVisionPython.so: lib/libcisstOSAbstraction.so
lib/_cisstStereoVisionPython.so: lib/libcisstMultiTask.so
lib/_cisstStereoVisionPython.so: lib/libcisstNumerical.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
lib/_cisstStereoVisionPython.so: lib/libcisstOSAbstraction.so
lib/_cisstStereoVisionPython.so: lib/libcisstVector.so
lib/_cisstStereoVisionPython.so: lib/libcisstCommon.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libz.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libpng.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libavformat.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libavutil.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libswscale.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libdc1394.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libGLU.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libGL.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libSM.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libICE.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libX11.so
lib/_cisstStereoVisionPython.so: /usr/lib/x86_64-linux-gnu/libXext.so
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib.a
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_hanson_haskell.a
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_lawson_hanson.a
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_lapack.a
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_blas.a
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_gfortran.so
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_quadmath.a
lib/_cisstStereoVisionPython.so: /home/davincic2/catkin_ws/devel_release/lib/libcisstNetlib_gcc.a
lib/_cisstStereoVisionPython.so: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared module ../../lib/_cisstStereoVisionPython.so"
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/_cisstStereoVisionPython.dir/link.txt --verbose=$(VERBOSE)
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/cmake -E copy_if_different /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/cisstStereoVisionPython.py /home/davincic2/catkin_ws/build_release/cisst/lib/./cisstStereoVisionPython.py
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && /usr/bin/cmake -E copy_if_different /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/cisstStereoVisionPython.py /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/cisstStereoVision.py

# Rule to build all files generated by this target.
cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/build: lib/_cisstStereoVisionPython.so
.PHONY : cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/build

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/requires: cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/__/cisstStereoVisionPYTHON_wrap.cxx.o.requires
.PHONY : cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/requires

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/clean:
	cd /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython && $(CMAKE_COMMAND) -P CMakeFiles/_cisstStereoVisionPython.dir/cmake_clean.cmake
.PHONY : cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/clean

cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/depend: cisstStereoVision/codePython/../cisstStereoVisionPYTHON_wrap.cxx
cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/depend: cisstStereoVision/codePython/cisstStereoVision.py
	cd /home/davincic2/catkin_ws/build_release/cisst && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davincic2/catkin_ws/src/cisst-saw/cisst /home/davincic2/catkin_ws/src/cisst-saw/cisst/cisstStereoVision/codePython /home/davincic2/catkin_ws/build_release/cisst /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython /home/davincic2/catkin_ws/build_release/cisst/cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cisstStereoVision/codePython/CMakeFiles/_cisstStereoVisionPython.dir/depend
