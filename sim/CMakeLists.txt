find_package(gazebo REQUIRED)
find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    set(PROTOBUF_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${PROTOBUF_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS} -fPIC -DSIM -fext-numeric-literals")

##################################
# build the custom protobuf msgs
##################################
set(msgs
  ./msgs/state.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gzmaze_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gzmaze_msgs ${PROTOBUF_LIBRARY})
target_include_directories(gzmaze_msgs PRIVATE ${PROTOBUF_INCLUDE_DIRS})

##################################
# procompiled gazebo header
##################################
set(pch_out ${CMAKE_CURRENT_BINARY_DIR}/gazebo_pch.hh.gch)
set(pch_in ${CMAKE_CURRENT_SOURCE_DIR}//gazebo_pch.in)
set(FLAGS -g -fPIC -DSIM -std=c++11 -fext-numeric-literals -x c++-header)
foreach(GZ_INCLUDE ${GAZEBO_INCLUDE_DIRS})
  list(APPEND GZ_INCLUDE_FLAGS "-I${GZ_INCLUDE}")
endforeach()

list(APPEND QT_DEFS -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG)
list(APPEND GZ_INCLUDE_FLAGS "-I/usr/include/qt4/")
list(APPEND GZ_INCLUDE_FLAGS "-I/usr/include/qt4/QtCore")
list(APPEND GZ_INCLUDE_FLAGS "-I/usr/include/qt4/QtGui")
add_custom_command(OUTPUT ${pch_out}
  COMMAND ${CMAKE_CXX_COMPILER} ${GZ_INCLUDE_FLAGS} ${QT_DEFS} ${FLAGS} ${pch_in} -o ${pch_out}
  DEPENDS ${pch_in}
  COMMENT "Generating precompiled header: ${pch_out}"
  VERBATIM)

add_custom_target(gazebo_pch DEPENDS ${pch_out})

##################################
# build the gazebo plugins
##################################
add_library(mouse_plugin SHARED MousePlugin.cc)
target_link_libraries(mouse_plugin gzmaze_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(mouse_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(mouse_plugin gazebo_pch)

add_library(maze_plugin SHARED MazeFactory.cc)
target_link_libraries(maze_plugin ${GAZEBO_LIBRARIES})
set_target_properties(maze_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(maze_plugin gazebo_pch)

QT4_WRAP_CPP(headers_MOC RegenerateWidget.hh)
add_library(regenerate_widget SHARED ${headers_MOC} RegenerateWidget.cc)
set_target_properties(regenerate_widget PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
target_link_libraries(regenerate_widget
    ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
add_dependencies(regenerate_widget gazebo_pch)


##################################
# library for simulation sources
##################################
file(GLOB SIM_SRC *.cpp commands/*.cpp)
add_library(sim ${COM_SRC} ${SIM_SRC})
set_target_properties(sim PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
target_include_directories(sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} commands)
target_link_libraries(sim gzmaze_msgs ${GAZEBO_LIBRARIES})
add_dependencies(sim gazebo_pch)

#################################
# actual mouse programs
#################################
set(SIMS SimSolve
  SimStepSolve)

foreach(MAIN ${SIMS})
  add_executable(${MAIN} "main/${MAIN}.cpp")
  set_target_properties(${MAIN} PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
  target_link_libraries(${MAIN} sim ${GAZEBO_LIBRARIES})
  target_include_directories(${MAIN} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} commands)
  add_dependencies(${MAIN} gazebo_pch)
endforeach()

#################################
# command line tools for simulation
#################################
add_executable(teleop ./tools/keyboard_teleop.cpp)
set_target_properties(teleop PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
target_link_libraries(teleop sim ${GAZEBO_LIBRARIES} ncurses)
target_include_directories(teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} commands)
add_dependencies(teleop gazebo_pch)
