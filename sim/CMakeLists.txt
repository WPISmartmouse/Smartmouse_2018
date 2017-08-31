find_package(Qt5 REQUIRED COMPONENTS Core Gui)
find_package(Qt5Test REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ignition-transport3 QUIET REQUIRED)

add_definitions(${QT_DEFINITIONS})
include_directories(${PROTOBUF_INCLUDE_DIRS} ${Qt5Test_INCLUDE_DIRS} ${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS} -fPIC -DSIM -fext-numeric-literals -Wno-deprecated-declarations")

#################################
# add the simulator
#################################
add_subdirectory(simulator)

##################################
# library for simulation sources
##################################
file(GLOB SIM_SRC lib/*.cpp commands/*.cpp)
add_library(sim ${COM_SRC} ${SIM_SRC})
target_include_directories(sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS} commands)
set_target_properties(sim PROPERTIES COMPILE_FLAGS "-include common/util.h")
target_link_libraries(sim ${IGNITION-TRANSPORT_LIBRARIES} msgs)
add_dependencies(sim msgs)

#################################
# actual sim mouse programs
#################################
set(SIMS SimSolve SimNavTest)

foreach (MAIN ${SIMS})
    add_executable(${MAIN} "main/${MAIN}.cpp")
    target_link_libraries(${MAIN} sim ${IGNITION-TRANSPORT_LIBRARIES})
    target_include_directories(${MAIN} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} commands)
endforeach ()

#################################
# command line tools for simulation
#################################
add_executable(keyboard_teleop ./tools/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop sim ncurses msgs)
target_include_directories(keyboard_teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS} commands)

add_executable(joy_teleop ./tools/joystick_teleop.cpp joystick/joystick.cc)
target_link_libraries(joy_teleop sim ncurses msgs)
target_include_directories(joy_teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS} commands)
