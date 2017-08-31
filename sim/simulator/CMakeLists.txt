# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Bring in ignition-transport3
find_package(ignition-transport3 QUIET REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

add_library(common src/Time.cpp)
add_library(server src/Server.cpp)

add_executable(SmartmouseSim src/main.cpp)

target_link_libraries(SmartmouseSim Qt5::Widgets ${IGNITION-TRANSPORT_LIBRARIES} server common)

install(TARGETS SmartmouseSim DESTINATION bin)
