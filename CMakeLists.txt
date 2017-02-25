project(smartmouse)
cmake_minimum_required(VERSION 3.1)

option(BUILD_SIM "build gazebo simulations" OFF)

if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DEBUG /EHsc")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Winvalid-pch")
endif()

file(GLOB_RECURSE COM_SRC lib/common/*.cpp lib/commanduino/*.cpp)
file(GLOB CONSOLE_SRC console/*.cpp console/commands/*.cpp)
file(GLOB REAL_SRC lib/real/*.cpp lib/real/commands/*.cpp)

include_directories(build
  lib/commanduino
  lib/common
  lib/common/commands)

################################
# library for console programs
################################
add_library(console ${COM_SRC} ${CONSOLE_SRC})
target_compile_features(console PRIVATE cxx_strong_enums)
set_target_properties(console PROPERTIES COMPILE_FLAGS "-DCONSOLE")
target_include_directories(console PRIVATE console/ console/commands)

################################
# text-only console programs
################################
set(CONSOLES Solve
  Animate
  ReadAndPrint)

foreach(MAIN ${CONSOLES})
  add_executable(${MAIN} "console/main/${MAIN}.cpp")
  target_link_libraries(${MAIN} console)
  target_compile_features(${MAIN} PRIVATE cxx_strong_enums)
  set_target_properties(${MAIN} PROPERTIES COMPILE_FLAGS "-DCONSOLE")
  target_include_directories(${MAIN} PRIVATE console/ console/commands)
endforeach()

################################
# actual real robot programs
################################
add_custom_target(serial COMMAND cd ../ && platformio serialports monitor -b 115200)
add_custom_target(upload COMMAND cd ../ && platformio run -v --target upload)

add_custom_target(due COMMAND cd ../ && platformio run -v)

################################
# all simulation stuff
################################
if (BUILD_SIM)
  add_subdirectory(sim)
endif()

################################
# testing
################################
if (test)
  add_subdirectory(gtest)

  enable_testing()

  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(runConsoleTests test/Test.cpp ${COM_SRC} ${CONSOLE_SRC})
	target_compile_features(runConsoleTests PRIVATE cxx_strong_enums)
  target_include_directories(runConsoleTests PRIVATE console/ console/commands)

  target_link_libraries(runConsoleTests gtest gtest_main)
  set_target_properties(runConsoleTests PROPERTIES COMPILE_FLAGS "-DCONSOLE")

  add_test(all-tests runConsoleTests)
endif()
