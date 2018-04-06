cmake_minimum_required(VERSION 3.1)

option(PROFILE "profile real robot code" OFF)
option(BUILD_SIM "build simulations" OFF)
include_directories(${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR} common/Eigen)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)

if (NOT ${BUILD_SIM})
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/teensy-arm.toolchain.cmake")

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif ()

project(smartmouse)

get_filename_component(UTIL_HEADER common/core/util.h ABSOLUTE)
file(GLOB_RECURSE CORE_COM_SRC common/commanduino/*.cpp common/core/*.cpp)
file(GLOB COM_COMMAND_SRC common/commands/*.cpp)
file(GLOB COM_SRC
     common/*.cpp
     common/math/*.cpp
     common/commanduino/*.cpp
     common/KinematicController/*.cpp
     common/IRSensorModeling/*.cpp
     common/argparse/*.cpp)

if (NOT ${BUILD_SIM})
  add_subdirectory(real)
else ()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unknown-pragmas -Wextra -fsanitize=address")

  ################################
  # gtest
  ################################
  add_subdirectory(gtest)
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  ################################
  # common tests
  ################################
  add_executable(common_tests common/test/CommonTest.cpp ${COM_SRC} ${CORE_COM_SRC})
  target_link_libraries(common_tests gtest gtest_main)
  set_target_properties(common_tests PROPERTIES COMPILE_FLAGS "-include ${UTIL_HEADER}")

  ################################
  # console
  ################################
  add_subdirectory(console)

  ################################
  # all simulation stuff
  ################################
  if (BUILD_SIM)
    add_subdirectory(sim)
  endif ()

  add_subdirectory(tools)
endif ()
