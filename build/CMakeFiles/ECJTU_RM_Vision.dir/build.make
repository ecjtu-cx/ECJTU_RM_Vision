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
CMAKE_SOURCE_DIR = /home/chengxi/SrcCode/ECJTU_RM_Vision

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chengxi/SrcCode/ECJTU_RM_Vision/build

# Include any dependencies generated for this target.
include CMakeFiles/ECJTU_RM_Vision.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ECJTU_RM_Vision.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ECJTU_RM_Vision.dir/flags.make

CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o: ../main.cpp
CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/main.cpp

CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/main.cpp > CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/main.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o: ../HKCamera/src/Camera.cpp
CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/HKCamera/src/Camera.cpp

CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/HKCamera/src/Camera.cpp > CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/HKCamera/src/Camera.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.s

# Object files for target ECJTU_RM_Vision
ECJTU_RM_Vision_OBJECTS = \
"CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o"

# External object files for target ECJTU_RM_Vision
ECJTU_RM_Vision_EXTERNAL_OBJECTS =

ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o
ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/HKCamera/src/Camera.cpp.o
ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/build.make
ECJTU_RM_Vision: /usr/local/lib/libopencv_gapi.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_highgui.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_ml.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_objdetect.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_photo.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_stitching.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_video.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_videoio.so.4.6.0
ECJTU_RM_Vision: ../HKCamera/libs/linux/libFormatConversion.so
ECJTU_RM_Vision: ../HKCamera/libs/linux/libMediaProcess.so
ECJTU_RM_Vision: ../HKCamera/libs/linux/libMvCameraControl.so
ECJTU_RM_Vision: ../HKCamera/libs/linux/libMVRender.so
ECJTU_RM_Vision: ../HKCamera/libs/linux/libMvUsb3vTL.so
ECJTU_RM_Vision: /usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1
ECJTU_RM_Vision: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_dnn.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_calib3d.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_features2d.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_flann.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_imgproc.so.4.6.0
ECJTU_RM_Vision: /usr/local/lib/libopencv_core.so.4.6.0
ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ECJTU_RM_Vision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ECJTU_RM_Vision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ECJTU_RM_Vision.dir/build: ECJTU_RM_Vision
.PHONY : CMakeFiles/ECJTU_RM_Vision.dir/build

CMakeFiles/ECJTU_RM_Vision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ECJTU_RM_Vision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ECJTU_RM_Vision.dir/clean

CMakeFiles/ECJTU_RM_Vision.dir/depend:
	cd /home/chengxi/SrcCode/ECJTU_RM_Vision/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chengxi/SrcCode/ECJTU_RM_Vision /home/chengxi/SrcCode/ECJTU_RM_Vision /home/chengxi/SrcCode/ECJTU_RM_Vision/build /home/chengxi/SrcCode/ECJTU_RM_Vision/build /home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles/ECJTU_RM_Vision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ECJTU_RM_Vision.dir/depend

