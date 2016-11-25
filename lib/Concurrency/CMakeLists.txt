#Author: Elena Ceseracciu, Juan Francisco Rascón Crespo

include_directories(include)

set(Concurrency_HEADERS Latch.h Queue.h)
set(Concurrency_TEMPLATE_IMPLEMENTATIONS Queue.cpp)
set_source_files_properties(${Concurrency_TEMPLATE_IMPLEMENTATIONS} PROPERTIES HEADER_FILE_ONLY TRUE)
set(Concurrency_SOURCES Latch.cpp)
set(Concurrency_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Include directory for Concurrency library")
mark_as_advanced(Concurrency_INCLUDE_DIRS)

source_group("Header files" FILES ${Concurrency_HEADERS})
source_group("Source files" FILES ${Concurrency_TEMPLATE_IMPLEMENTATIONS} ${Concurrency_SOURCES})

add_library(Concurrency ${Concurrency_HEADERS} ${Concurrency_TEMPLATE_IMPLEMENTATIONS} ${Concurrency_SOURCES})
