cmake_minimum_required(VERSION 2.6)
project(marker_AR)

find_package(OpenCV REQUIRED )
find_package(OpenGL REQUIRED )
find_package(GLUT REQUIRED)

include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${OpenGL_LIBRARY_DIR})
link_directories(${GLUT_LIBRARY_DIRS})

include_directories(${marker_AR_SOURCE_DIR}/src)


add_executable(marker_AR
        main.cpp
        MarkerDetector.cpp
        GeometryTypes.cpp
        Marker.cpp
)

target_link_libraries( marker_AR ${OpenCV_LIBRARIES} )
target_link_libraries( marker_AR ${OPENGL_LIBRARIES} )
target_link_libraries( marker_AR ${GLUT_LIBRARY} )

install (TARGETS marker_AR DESTINATION bin)


