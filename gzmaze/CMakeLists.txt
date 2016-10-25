cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gzmaze)

find_package(gazebo REQUIRED)
find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(MousePlugin SHARED src/MousePlugin.cc)
target_link_libraries(MousePlugin	 ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(maze_plugin SHARED src/MazeFactory.cc)
target_link_libraries(maze_plugin ${GAZEBO_LIBRARIES})

QT4_WRAP_CPP(headers_MOC src/RegenerateWidget.hh)
add_library(regenerate_widget SHARED ${headers_MOC} src/RegenerateWidget.cc)
target_link_libraries(regenerate_widget
    ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})

