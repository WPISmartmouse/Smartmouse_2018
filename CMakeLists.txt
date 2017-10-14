cmake_minimum_required(VERSION 3.1)

option(BUILD_SIM "build simulations" OFF)
set(CMAKE_CXX_STANDARD 14)

if (NOT ${BUILD_SIM})
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/teensy-arm.toolchain.cmake")

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif ()

project(smartmouse)

file(GLOB_RECURSE CORE_COM_SRC "common/commands/*.cpp" "common/commanduino/*.cpp" "common/core/*.cpp")
file(GLOB COM_SRC common/*.cpp common/math/*.cpp common/commanduino/*.cpp common/KinematicController/*.cpp)
file(GLOB COM_COMMAND_SRC common/commands/*.cpp)

if (NOT ${BUILD_SIM})
    include(Teensy)

    file(GLOB REAL_SRC
            real/*.cpp
            real/commands/*.cpp)

    file(GLOB_RECURSE REAL_LIBS_SRC
            real/lib/*.cpp)

    file(GLOB REAL_LIB_DIRS real/lib/*)

    include_directories(${CMAKE_SOURCE_DIR} ${REAL_LIB_DIRS} real real/commands common/Eigen)
    add_definitions(-DEMBED)

    import_arduino_library(Bounce2)
    import_arduino_library(i2c_t3)
    import_arduino_library(SPI)

    add_teensy_executable(main real/main/main.cpp ${COM_SRC} ${REAL_SRC} ${REAL_LIBS_SRC})
    set_target_properties(main PROPERTIES COMPILE_FLAGS "-include common/core/util.h")

    add_teensy_executable(eigen_test real/main/eigen_test.cpp common/core/util.cpp)
    set_target_properties(eigen_test PROPERTIES COMPILE_FLAGS "-include common/core/util.h")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unknown-pragmas -std=c++1y")

    ################################
    # gtest
    ################################
    add_library(console ${CORE_COM_SRC} ${CONSOLE_SRC})
    target_compile_features(console PRIVATE cxx_strong_enums)
    set_target_properties(console PROPERTIES COMPILE_FLAGS "-DCONSOLE -include common/core/util.h")
    target_include_directories(console PRIVATE console/ console/commands)

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

    ################################
    # testing
    ################################
    if (TESTS)
        add_subdirectory(gtest)

        enable_testing()

        include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

        add_executable(runConsoleTests test/Test.cpp)
        target_compile_features(runConsoleTests PRIVATE cxx_strong_enums)
        target_include_directories(runConsoleTests PRIVATE console/ console/commands)
        target_link_libraries(runConsoleTests console gtest gtest_main)
        set_target_properties(runConsoleTests PROPERTIES COMPILE_FLAGS "-DCONSOLE -include common/core/util.h")

        add_test(all-tests runConsoleTests)
    endif ()
endif ()
