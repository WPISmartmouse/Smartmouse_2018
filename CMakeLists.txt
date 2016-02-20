project(smartmouse)
cmake_minimum_required(VERSION 3.1)

file(GLOB_RECURSE COM_SRC src/common/*.cpp)
file(GLOB_RECURSE COM_HRDS src/common/*.h)

set(CMAKE_CXX_FLAGS "-g")

include_directories(src src/common)

add_library(common_lib SHARED ${COM_SRC})
target_compile_features(common_lib PRIVATE cxx_strong_enums)

install(TARGETS common_lib LIBRARY DESTINATION lib)
install(FILES ${COM_HRDS} DESTINATION "include/smartmouse")

SET(MAINS Solve
  StepSolve
  Animate
  ReadAndPrint)

foreach(MAIN ${MAINS})
  add_executable(${MAIN} "src/${MAIN}.cpp")
  target_link_libraries(${MAIN} common_lib)
  target_compile_features(${MAIN} PRIVATE cxx_strong_enums)
endforeach()

add_custom_target(arduino COMMAND ano build -s=../src -f="-std=c++11 -g -DEMBED -I ../src/common")

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
