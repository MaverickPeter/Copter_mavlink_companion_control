cmake_minimum_required(VERSION 2.8)
project(mavlink_control)

find_package(OpenCV REQUIRED)
find_package(Threads)
#find_package(WiringPi REQUIRED)

aux_source_directory(. DIRSRCS)
include_directories(./ ./detect ./interface ./serial
    ./mavlink/include/mavlink/v1.0
    ${OpenCV_INCLUDE_DIRS}
    ${WIRINGPI_INCLUDE_DIRS}
)

#add_definitions(-lwiringPi -lwiringPiDev)
add_subdirectory(detect)
add_subdirectory(interface)
add_subdirectory(serial)

add_executable(mavlink_control mavlink_control.cpp)
target_link_libraries (mavlink_control
    detect
    interface
    serial
    ${wiringPi} 
    ${wiringPiDev} 
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
)


