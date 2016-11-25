## Authors: Monica Reggiani, Juan Francisco Rascón Crespo, Marco Matteo Bassa

if(GLIB_PKG_FOUND)

include_directories(include ${Concurrency_INCLUDE_DIRS} ${CANbus_INCLUDE_DIRS} ${GLIB_PKG_INCLUDE_DIRS})

#set(H2R_ROS_HEADERS include/interruptFunctions.h)
#set(H2R_ROS_RX_SOURCES rx_node.cpp interruptFunctions.cpp ../lib/CANbus/CANbusInterface.cpp)

set(H2R_ROS_RX_SOURCES rx_node.cpp interruptFunctions.cpp)
set(H2R_ROS_TX_SOURCES tx_node.cpp)
set(H2R_ROS_FSRS_SOURCES fsrs_node.cpp interruptFunctions.cpp)

#set(H2R_ROS_PATTERN_TX_SOURCES pattern_node.cpp)

add_executable(rx_node ${H2R_ROS_RX_SOURCES})
add_executable(tx_node ${H2R_ROS_TX_SOURCES} )
add_executable(fsrs_node ${H2R_ROS_FSRS_SOURCES})

#add_executable(pattern_node ${H2R_ROS_PATTERN_TX_SOURCES})

target_link_libraries(rx_node CANbus ${GLIB_PKG_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tx_node CANbus ${GLIB_PKG_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(fsrs_node CANbus ${GLIB_PKG_LIBRARIES} ${catkin_LIBRARIES})

#target_link_libraries(pattern_node CANbus ${catkin_LIBRARIES} ${GLIB_PKG_LIBRARIES})

else()
  message(STATUS "Disabling compilation of executables because glib was not found")
endif()
