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
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/main.cpp

CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/main.cpp > CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/main.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o: ../src/AngleSolver/anglesolver.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/AngleSolver/anglesolver.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/AngleSolver/anglesolver.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/AngleSolver/anglesolver.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o: ../src/ArmorDetector/detector.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/ArmorDetector/detector.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/ArmorDetector/detector.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/ArmorDetector/detector.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o: ../src/General/general.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/General/general.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/General/general.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/General/general.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o: ../src/HKCamera/Camera.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/HKCamera/Camera.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/HKCamera/Camera.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/HKCamera/Camera.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o: ../src/SerialPort/CRC_Check.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/SerialPort/CRC_Check.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/SerialPort/CRC_Check.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/SerialPort/CRC_Check.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o: ../src/SerialPort/serialport.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/SerialPort/serialport.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/SerialPort/serialport.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/SerialPort/serialport.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.s

CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/flags.make
CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o: ../src/Thread/thread.cpp
CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o: CMakeFiles/ECJTU_RM_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o -MF CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o.d -o CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o -c /home/chengxi/SrcCode/ECJTU_RM_Vision/src/Thread/thread.cpp

CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengxi/SrcCode/ECJTU_RM_Vision/src/Thread/thread.cpp > CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.i

CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengxi/SrcCode/ECJTU_RM_Vision/src/Thread/thread.cpp -o CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.s

# Object files for target ECJTU_RM_Vision
ECJTU_RM_Vision_OBJECTS = \
"CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o" \
"CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o"

# External object files for target ECJTU_RM_Vision
ECJTU_RM_Vision_EXTERNAL_OBJECTS =

../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/main.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/AngleSolver/anglesolver.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/ArmorDetector/detector.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/General/general.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/HKCamera/Camera.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/CRC_Check.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/SerialPort/serialport.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/src/Thread/thread.cpp.o
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/build.make
../ECJTU_RM_Vision: /usr/local/lib/libopencv_gapi.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_highgui.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_ml.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_objdetect.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_photo.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_stitching.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_video.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_videoio.so.4.6.0
../ECJTU_RM_Vision: ../lib/libFormatConversion.so
../ECJTU_RM_Vision: ../lib/libMediaProcess.so
../ECJTU_RM_Vision: ../lib/libMvCameraControl.so
../ECJTU_RM_Vision: ../lib/libMVRender.so
../ECJTU_RM_Vision: ../lib/libMvUsb3vTL.so
../ECJTU_RM_Vision: /usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1
../ECJTU_RM_Vision: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_dnn.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_calib3d.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_features2d.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_flann.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_imgproc.so.4.6.0
../ECJTU_RM_Vision: /usr/local/lib/libopencv_core.so.4.6.0
../ECJTU_RM_Vision: CMakeFiles/ECJTU_RM_Vision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable ../ECJTU_RM_Vision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ECJTU_RM_Vision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ECJTU_RM_Vision.dir/build: ../ECJTU_RM_Vision
.PHONY : CMakeFiles/ECJTU_RM_Vision.dir/build

CMakeFiles/ECJTU_RM_Vision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ECJTU_RM_Vision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ECJTU_RM_Vision.dir/clean

CMakeFiles/ECJTU_RM_Vision.dir/depend:
	cd /home/chengxi/SrcCode/ECJTU_RM_Vision/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chengxi/SrcCode/ECJTU_RM_Vision /home/chengxi/SrcCode/ECJTU_RM_Vision /home/chengxi/SrcCode/ECJTU_RM_Vision/build /home/chengxi/SrcCode/ECJTU_RM_Vision/build /home/chengxi/SrcCode/ECJTU_RM_Vision/build/CMakeFiles/ECJTU_RM_Vision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ECJTU_RM_Vision.dir/depend

