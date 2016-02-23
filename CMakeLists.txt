project(smartmouse)
cmake_minimum_required(VERSION 3.1)

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
file(GLOB_RECURSE COM_SRC src/common/*.cpp)
file(GLOB_RECURSE COM_HRDS src/common/*.h)

set(CMAKE_CXX_FLAGS "-g")

include_directories(src src/common)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

SET(MAINS Solve
  StepSolve
  Animate
  ReadAndPrint)

foreach(MAIN ${MAINS})
  add_executable(${MAIN} "src/${MAIN}.cpp" ${COM_SRC})
  target_compile_features(${MAIN} PRIVATE cxx_strong_enums)
  set_target_properties(${MAIN} PROPERTIES COMPILE_FLAGS "-DCONSOLE")
endforeach()

add_custom_target(arduino COMMAND ano build -s=../src -f="-std=c++11 -g -DEMBED -I ../src/common")

add_executable(SimStepSolve "src/SimSolve.cpp" ${COM_SRC})
target_link_libraries(SimStepSolve ${GAZEBO_LIBRARIES} pthread)
target_compile_features(SimStepSolve PRIVATE cxx_strong_enums)
set_target_properties(SimStepSolve PROPERTIES COMPILE_FLAGS "-DSIM")

################################
# Testing
################################
if (test)
  add_subdirectory(gtest)

  enable_testing()

  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(runUnitTests test/Test.cpp ${COM_SRC})
	target_compile_features(runUnitTests PRIVATE cxx_strong_enums)

  target_link_libraries(runUnitTests gtest gtest_main)
  set_target_properties(runUnitTests PROPERTIES COMPILE_FLAGS "-DCONSOLE")

  add_test(all-tests runUnitTests)
endif()
