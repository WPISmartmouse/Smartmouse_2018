project(smartmouse)
cmake_minimum_required(VERSION 3.1)

file(GLOB_RECURSE COM_SRC src/common/*.cpp src/common/*.h)

set(CMAKE_CXX_FLAGS "-g")

include_directories(src src/common)

add_executable(explore src/Explore.cpp ${COM_SRC})
target_compile_features(explore PRIVATE cxx_strong_enums)

add_executable(read_and_print src/ReadAndPrint.cpp ${COM_SRC})
target_compile_features(read_and_print PRIVATE cxx_strong_enums)

add_custom_target(arduino COMMAND ano build -s=../src -f="-std=c++11 -g -DEMBED -I common")

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

  add_test(all-tests runUnitTests)
endif()
