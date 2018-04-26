aux_source_directory(. DIR_DETECT_SRCS)
add_library(detect ${DIR_DETECT_SRCS})
#target_link_libraries(detect fire_recognition.cpp GeometryTypes.cpp Marker.cpp MaerkerDetector.cpp)
