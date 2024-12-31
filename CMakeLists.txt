cmake_minimum_required(VERSION 3.10)
project(ECJTU_RM_Vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${fmt_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

include_directories(
    ./Config
    ./include/HKCamera
    ./include/ArmorDetector
    ./include/AngleSolver
    ./include/SerialPort
    ./include/Thread
    ./include/General
)

file(GLOB_RECURSE source
    ./src/HKCamera/*.*
    ./src/AngleSolver/*.*
    ./src/ArmorDetector/*.*
    ./src/SerialPort/*.*
    ./src/Thread/*.*
    ./src/General/*.*
)

add_executable(ECJTU_RM_Vision ./main.cpp ${source})

target_link_libraries(ECJTU_RM_Vision
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libFormatConversion.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMediaProcess.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMvCameraControl.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMVRender.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMvUsb3vTL.so
    fmt::fmt
    -lpthread -lm
)