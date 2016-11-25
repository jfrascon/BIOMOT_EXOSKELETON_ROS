#Authors: Monica Reggiani, Bassa Marco, Juan Francisco Rascón Crespo

include_directories(include ${Concurrency_INCLUDE_DIRS})

set(CANbus_HEADERS DataFromCANbus.h CANbusInterface.h CANbusReader.h QueueData.h)
set(CANbus_SOURCES DataFromCANbus.cpp CANbusInterface.cpp CANbusReader.cpp)
set(CANbus_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Include directory for CANbus library")
mark_as_advanced(CANbus_INCLUDE_DIRS)

source_group("Header files" FILES ${CANbus_HEADERS})
source_group("Source files" FILES ${CANbus_SOURCES})

add_library(CANbus ${CANbus_HEADERS} ${CANbus_SOURCES})
