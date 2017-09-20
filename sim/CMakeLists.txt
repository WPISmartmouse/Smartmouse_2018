cmake_policy(SET CMP0071 OLD)

find_package(Qt5 5.9.1 REQUIRED COMPONENTS Core Gui Charts)
find_package(Protobuf REQUIRED)
find_package(ignition-msgs0 QUIET REQUIRED)
find_package(ignition-transport3 QUIET REQUIRED)

set(PROTOBUF_IMPORT_DIRS ${IGNITION-MSGS_PROTO_PATH} "/usr/include/")

add_definitions(${QT_DEFINITIONS})
include_directories(${PROTOBUF_INCLUDE_DIRS} ${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS} -fPIC -DSIM -fext-numeric-literals -Wno-deprecated-declarations")

#################################
# add the simulator
#################################
add_subdirectory(simulator)

##################################
# libraries for common & simulation sources
##################################
add_library(sim_common ${COM_SRC})
set_target_properties(sim_common PROPERTIES COMPILE_FLAGS "-include common/util.h")

add_library(sim_common_commands ${COM_COMMAND_SRC})
target_link_libraries(sim_common_commands msgs)
set_target_properties(sim_common_commands PROPERTIES COMPILE_FLAGS "-include common/util.h")

file(GLOB SIM_SRC lib/*.cpp commands/*.cpp)
add_library(sim ${SIM_SRC})
set_target_properties(sim PROPERTIES COMPILE_FLAGS "-include common/util.h")
target_link_libraries(sim sim_common sim_common_commands ${IGNITION-TRANSPORT_LIBRARIES} msgs)

#################################
# actual sim mouse programs
#################################
set(SIMS SimSolve SimNavTest)

foreach (MAIN ${SIMS})
    add_executable(${MAIN} "main/${MAIN}.cpp")
    target_link_libraries(${MAIN} sim_common sim_common_commands sim ${IGNITION-TRANSPORT_LIBRARIES})
endforeach ()

#################################
# command line tools for simulation
#################################
add_executable(keyboard_teleop ./tools/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop sim_common sim ncurses msgs)

add_executable(joy_teleop ./tools/joystick_teleop.cpp joystick/joystick.cc)
target_link_libraries(joy_teleop sim_common sim ncurses msgs)
