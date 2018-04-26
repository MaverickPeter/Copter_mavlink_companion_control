aux_source_directory(. DIR_INTERFACE_SRCS)
add_library(interface ${DIR_INTERFACE_SRCS})

#include_directories(../serial)
#link_directories(${DIR_SERIAL_SRCS})
#target_link_libraries(interface serial)