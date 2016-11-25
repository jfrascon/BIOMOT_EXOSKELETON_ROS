## Authors: Monica Reggiani, Juan Francisco Rascón Crespo
cmake_minimum_required(VERSION 2.8.3)

project(h2r)

if(UNIX)
  list(APPEND CMAKE_CXX_FLAGS "-pthread -std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

find_package(Qt4)
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GLIB_PKG glib-2.0)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation)

## Generate messages in the 'msg' folder
add_message_files(FILES FeetFSRSThresholdsMessage.msg ForcesMessage.msg FSRSMessage.msg JointAnglesMessage.msg ParallelSpringMessage.msg PatternsMessage.msg PIDMessage.msg SegmentationMessage.msg SensorsMessage.msg SingleCommandMessage.msg TorquesMessage.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp message_runtime DEPENDS Glib Qt4)

# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

if(Qt4_FOUND)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

set(joint_angles_gui_SOURCES src/joint_angles_gui_node.cpp include/JOINT_ANGLES_GUI/joint_angles_gui.cpp include/qcustomplot.cpp)
set(joint_angles_gui_HEADERS include/JOINT_ANGLES_GUI/joint_angles_gui.h include/qcustomplot.h)
set(joint_angles_gui_FORMS include/JOINT_ANGLES_GUI/joint_angles_gui.ui)
QT4_WRAP_CPP(joint_angles_gui_MOC_outfiles ${joint_angles_gui_HEADERS})
QT4_WRAP_UI(joint_angles_gui_FORMS_outfiles ${joint_angles_gui_FORMS})
add_executable(joint_angles_gui_node ${joint_angles_gui_SOURCES} ${joint_angles_gui_MOC_outfiles} ${joint_angles_gui_FORMS_outfiles})
add_dependencies(joint_angles_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(joint_angles_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

set(aux_var_gui_SOURCES src/aux_var_gui_node.cpp include/AUX_VAR_GUI/aux_var_gui.cpp)
set(aux_var_gui_HEADERS include/AUX_VAR_GUI/aux_var_gui.h)
set(aux_var_gui_FORMS include/AUX_VAR_GUI/aux_var_gui.ui)
QT4_WRAP_CPP(aux_var_gui_MOC_outfiles ${aux_var_gui_HEADERS})
QT4_WRAP_UI(aux_var_gui_FORMS_outfiles ${aux_var_gui_FORMS})
add_executable(aux_var_gui_node ${aux_var_gui_SOURCES} ${aux_var_gui_MOC_outfiles} ${aux_var_gui_FORMS_outfiles})
add_dependencies(aux_var_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(aux_var_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

set(commands_gui_SOURCES src/commands_gui_node.cpp include/COMMANDS_GUI/commands_gui.cpp)
set(commands_gui_HEADERS include/COMMANDS_GUI/commands_gui.h)
set(commands_gui_FORMS include/COMMANDS_GUI/commands_gui.ui)
QT4_WRAP_CPP(commands_gui_MOC_outfiles ${commands_gui_HEADERS})
QT4_WRAP_UI(commands_gui_FORMS_outfiles ${commands_gui_FORMS})
add_executable(commands_gui_node ${commands_gui_SOURCES} ${commands_gui_MOC_outfiles} ${commands_gui_FORMS_outfiles})
add_dependencies(commands_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(commands_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

set(joint_information_gui_SOURCES src/joint_information_gui_node.cpp include/JOINT_INFORMATION_GUI/joint_information_gui.cpp)
set(joint_information_gui_HEADERS include/JOINT_INFORMATION_GUI/joint_information_gui.h)
set(joint_information_gui_FORMS include/JOINT_INFORMATION_GUI/joint_information_gui.ui)
QT4_WRAP_CPP(joint_information_gui_MOC_outfiles ${joint_information_gui_HEADERS})
QT4_WRAP_UI(joint_information_gui_FORMS_outfiles ${joint_information_gui_FORMS})
add_executable(joint_information_gui_node ${joint_information_gui_SOURCES} ${joint_information_gui_MOC_outfiles} ${joint_information_gui_FORMS_outfiles})
add_dependencies(joint_information_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(joint_information_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

list(APPEND parallel_spring_information_GUI src/parallel_spring_information_gui_node.cpp include/PARALLEL_SPRING_INFORMATION_GUI/parallel_spring_information_gui.cpp include/qcustomplot.cpp)
list(APPEND parallel_spring_information_gui_HEADERS include/PARALLEL_SPRING_INFORMATION_GUI/parallel_spring_information_gui.h include/qcustomplot.h)
list(APPEND parallel_spring_information_gui_FORMS include/PARALLEL_SPRING_INFORMATION_GUI/parallel_spring_information_gui.ui)

QT4_WRAP_CPP(parallel_spring_information_gui_MOC_outfiles ${parallel_spring_information_gui_HEADERS})
QT4_WRAP_UI(parallel_spring_information_gui_FORMS_outfiles ${parallel_spring_information_gui_FORMS})

add_executable(parallel_spring_information_gui_node ${parallel_spring_information_GUI} ${parallel_spring_information_gui_MOC_outfiles} ${parallel_spring_information_gui_FORMS_outfiles})
add_dependencies(parallel_spring_information_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(parallel_spring_information_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

set(pid_gui_SOURCES src/pid_gui_node.cpp include/PID_GUI/pid_gui.cpp include/qcustomplot.cpp)
set(pid_gui_HEADERS include/PID_GUI/pid_gui.h include/qcustomplot.h)
set(pid_gui_FORMS include/PID_GUI/pid_gui.ui)
QT4_WRAP_CPP(pid_gui_MOC_outfiles ${pid_gui_HEADERS})
QT4_WRAP_UI(pid_gui_FORMS_outfiles ${pid_gui_FORMS})
add_executable(pid_gui_node ${pid_gui_SOURCES} ${pid_gui_MOC_outfiles} ${pid_gui_FORMS_outfiles})
add_dependencies(pid_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pid_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})


set(pid_information_gui_SOURCES src/pid_information_gui_node.cpp include/PID_INFORMATION_GUI/pid_information_gui.cpp)
set(pid_information_gui_HEADERS include/PID_INFORMATION_GUI/pid_information_gui.h)
set(pid_information_gui_FORMS include/PID_INFORMATION_GUI/pid_information_gui.ui)
QT4_WRAP_CPP(pid_information_gui_MOC_outfiles ${pid_information_gui_HEADERS})
QT4_WRAP_UI(pid_information_gui_FORMS_outfiles ${pid_information_gui_FORMS})
add_executable(pid_information_gui_node ${pid_information_gui_SOURCES} ${pid_information_gui_MOC_outfiles} ${pid_information_gui_FORMS_outfiles})
add_dependencies(pid_information_gui_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pid_information_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})


#set(sensors_information_gui_SOURCES src/sensors_information_gui_node.cpp include/SENSORS_INFORMATION_GUI/sensors_information_gui.cpp include/logObject.cpp)
#set(sensors_information_gui_HEADERS include/SENSORS_INFORMATION_GUI/sensors_information_gui.h include/logObject.h)
#set(sensors_information_gui_FORMS include/SENSORS_INFORMATION_GUI/sensors_information_gui.ui)
#QT4_WRAP_CPP(sensors_information_gui_MOC_outfiles ${sensors_information_gui_HEADERS})
#QT4_WRAP_UI(sensors_information_gui_FORMS_outfiles ${sensors_information_gui_FORMS})
#add_executable(sensors_information_gui_node ${sensors_information_gui_SOURCES} ${sensors_information_gui_MOC_outfiles} ${sensors_information_gui_FORMS_outfiles})
#add_dependencies(sensors_information_gui_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(sensors_information_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})


#set(torque_gui_SOURCES src/torque_gui_node.cpp include/TORQUE_GUI/torque_gui.cpp include/qcustomplot.cpp)
#set(torque_gui_HEADERS include/TORQUE_GUI/torque_gui.h include/qcustomplot.h)
#set(torque_gui_FORMS include/TORQUE_GUI/torque_gui.ui)
#QT4_WRAP_CPP(torque_gui_MOC_outfiles ${torque_gui_HEADERS})
#QT4_WRAP_UI(torque_gui_FORMS_outfiles ${torque_gui_FORMS})
#add_executable(torque_gui_node ${torque_gui_SOURCES} ${torque_gui_MOC_outfiles} ${torque_gui_FORMS_outfiles})
#add_dependencies(torque_gui_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(torque_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})


#set(patterns_gui_SOURCES src/patterns_gui_node.cpp include/PATTERNS_GUI/patterns_gui.cpp include/PATTERNS_GUI/worker_thread_patterns_gui.cpp)
#set(patterns_gui_HEADERS include/PATTERNS_GUI/patterns_gui.h include/PATTERNS_GUI/worker_thread_patterns_gui.h)
#set(patterns_gui_FORMS include/PATTERNS_GUI/patterns_gui.ui)
#QT4_WRAP_CPP(patterns_gui_MOC_outfiles ${patterns_gui_HEADERS})
#QT4_WRAP_UI(patterns_gui_FORMS_outfiles ${patterns_gui_FORMS})
#add_executable(patterns_gui_node ${patterns_gui_SOURCES} ${patterns_gui_MOC_outfiles} ${patterns_gui_FORMS_outfiles})
#add_dependencies(patterns_gui_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(patterns_gui_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

#?
set(CMAKE_AUTOMOC ON)

else()

  message(STATUS "Disabling compilation of UI nodes because Qt4 was not found")

endif()

add_subdirectory(lib)
add_subdirectory(src_bbb)
