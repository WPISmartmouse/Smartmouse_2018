# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets Charts REQUIRED)

# Find QWT
find_path(QWT_INCLUDE_DIR NAMES qwt.h PATHS
          /usr/local/qwt-6.1.3/include)

find_library(QWT_LIBRARY NAMES qwt-qt5 qwt PATHS
             /usr/local/qwt-6.1.3/lib)

add_subdirectory(msgs)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER}
                            PROPERTIES GENERATED TRUE)

file(GLOB SIMULATOR_COMMON_SRC lib/common/*.cpp)
add_library(simulator_lib ${SIMULATOR_COMMON_SRC})
target_link_libraries(simulator_lib msgs)

add_library(server lib/Server.cpp)
target_link_libraries(server msgs sim simulator_lib ${IGNITION-TRANSPORT_LIBRARIES})

add_library(client lib/Client.cpp)
target_include_directories(client PRIVATE ${QWT_INCLUDE_DIR})
target_link_libraries(client
                      msgs
                      Qt5::Widgets
                      Qt5::Charts
                      simulator_lib
                      sim
                      server
                      ${IGNITION-TRANSPORT_LIBRARIES}
                      ${QWT_LIBRARY})

file(GLOB WIDGET_SRCS lib/widgets/*.cpp)

add_executable(SmartmouseSim SmartmouseSim.cpp ${WIDGET_SRCS} simulator.qrc)
target_include_directories(SmartmouseSim PRIVATE ${QWT_INCLUDE_DIR})
target_link_libraries(SmartmouseSim server client ${QWT_LIBRARY})

install(TARGETS SmartmouseSim DESTINATION bin)

################################
# testing
################################
add_executable(sim_tests test/SimTest.cpp)
target_link_libraries(sim_tests gtest gtest_main server sim msgs)
set_target_properties(sim_tests PROPERTIES COMPILE_FLAGS "-DCONSOLE -include ${UTIL_HEADER}")

install(TARGETS sim_tests DESTINATION bin)
