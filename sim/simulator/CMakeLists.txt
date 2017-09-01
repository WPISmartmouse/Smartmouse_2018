# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

add_subdirectory(msgs)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER}
        PROPERTIES GENERATED TRUE)

add_library(common lib/Time.cpp)
target_link_libraries(common msgs)

add_library(server lib/Server.cpp)
target_link_libraries(server msgs common ${IGNITION-TRANSPORT_LIBRARIES})

add_library(client lib/Client.cpp)
target_link_libraries(client msgs Qt5::Widgets common server ${IGNITION-TRANSPORT_LIBRARIES})

file(GLOB WIDGET_SRCS lib/widgets/*.cpp)

add_executable(SmartmouseSim main.cpp ${WIDGET_SRCS})

target_link_libraries(SmartmouseSim server client)

install(TARGETS SmartmouseSim DESTINATION bin)
