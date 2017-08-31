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
target_link_libraries(server msgs)

add_executable(SmartmouseSim main.cpp)

target_link_libraries(SmartmouseSim Qt5::Widgets ${IGNITION-TRANSPORT_LIBRARIES} msgs server common)

install(TARGETS SmartmouseSim DESTINATION bin)
