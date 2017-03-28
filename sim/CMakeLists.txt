find_package(gazebo REQUIRED)
find_package (Qt5 REQUIRED COMPONENTS Core Gui)
find_package (Qt5Test REQUIRED)
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

add_definitions(${QT_DEFINITIONS})
include_directories(${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
  ${Qt5Test_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS} -fPIC -DSIM -fext-numeric-literals -Wno-deprecated-declarations")

##################################
# build the custom protobuf msgs
##################################
set(msgs
  ./msgs/state.proto
  ./msgs/maze_location.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gzmaze_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gzmaze_msgs ${PROTOBUF_LIBRARY})
target_include_directories(gzmaze_msgs PRIVATE ${PROTOBUF_INCLUDE_DIRS})


##################################
# procompiled gazebo header
##################################
set(pch_out ${CMAKE_CURRENT_BINARY_DIR}/gazebo_pch.hh.gch)
set(pch_in ${CMAKE_CURRENT_SOURCE_DIR}//gazebo_pch.in)
set(FLAGS -g -fPIC -DSIM -std=c++11 -fext-numeric-literals -x c++-header -Wno-deprecated-declarations)
foreach(GZ_INCLUDE ${GAZEBO_INCLUDE_DIRS})
  list(APPEND GZ_INCLUDE_FLAGS "-I${GZ_INCLUDE}")
endforeach()

list(APPEND GZ_INCLUDE_FLAGS "-I/usr/include/x86_64-linux-gnu/qt5/")
add_custom_command(OUTPUT ${pch_out}
  COMMAND ${CMAKE_CXX_COMPILER} ${GZ_INCLUDE_FLAGS} ${FLAGS} ${pch_in} -o ${pch_out}
  DEPENDS ${pch_in}
  COMMENT "Generating precompiled header: ${pch_out}"
  VERBATIM)

add_custom_target(gazebo_pch DEPENDS ${pch_out})


##################################
# build the gazebo plugins
##################################
add_library(mouse_plugin SHARED MousePlugin.cc)
target_link_libraries(mouse_plugin sim gzmaze_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(mouse_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(mouse_plugin gazebo_pch)

add_library(binary_ir_plugin SHARED BinaryIR.cc)
target_link_libraries(binary_ir_plugin ${GAZEBO_LIBRARIES})
set_target_properties(binary_ir_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(binary_ir_plugin gazebo_pch)

add_library(analog_ir_plugin SHARED AnalogIR.cc)
target_link_libraries(analog_ir_plugin ${GAZEBO_LIBRARIES})
set_target_properties(analog_ir_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(analog_ir_plugin gazebo_pch)

add_library(maze_plugin SHARED MazeFactory.cc)
target_link_libraries(maze_plugin ${GAZEBO_LIBRARIES})
set_target_properties(maze_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(maze_plugin gazebo_pch)

add_library(link_plot_3d_plugin SHARED LinkPlot3DPlugin.cc)
target_link_libraries(link_plot_3d_plugin ${GAZEBO_LIBRARIES})
set_target_properties(link_plot_3d_plugin PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
add_dependencies(link_plot_3d_plugin gazebo_pch)

QT5_WRAP_CPP(state_viz_MOC StateViz.hh)
add_library(state_viz SHARED ${state_viz_MOC} StateViz.cc)
set_target_properties(state_viz PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
target_link_libraries(state_viz
    sim
    gzmaze_msgs
    ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
add_dependencies(state_viz gazebo_pch)

QT5_WRAP_CPP(sensor_viz_MOC SensorViz.hh)
add_library(sensor_viz SHARED ${sensor_viz_MOC} SensorViz.cc)
set_target_properties(sensor_viz PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
target_link_libraries(sensor_viz
    ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
add_dependencies(sensor_viz gazebo_pch)

QT5_WRAP_CPP(regenerate_widget_MOC RegenerateWidget.hh)
add_library(regenerate_widget SHARED ${regenerate_widget_MOC} RegenerateWidget.cc)
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
target_include_directories(sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS} commands)
target_link_libraries(sim gzmaze_msgs ${GAZEBO_LIBRARIES})
add_dependencies(sim gazebo_pch)


#################################
# actual sim mouse programs
#################################
set(SIMS SimSolve SimNavTest)

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
target_include_directories(teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS} commands)
add_dependencies(teleop gazebo_pch)

add_executable(joy_teleop ./tools/joystick_teleop.cpp joystick/joystick.cc)
set_target_properties(joy_teleop PROPERTIES COMPILE_FLAGS "-include gazebo_pch.hh")
target_link_libraries(joy_teleop sim ${GAZEBO_LIBRARIES} ncurses)
target_include_directories(joy_teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS} commands)
add_dependencies(joy_teleop gazebo_pch)
