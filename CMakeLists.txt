cmake_minimum_required(VERSION 3.1)

option(BUILD_SIM "build gazebo simulations" OFF)

if (NOT ${BUILD_SIM})
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/teensy-arm.toolchain.cmake")

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif ()

project(smartmouse)

file(GLOB_RECURSE COM_SRC
        common/*.cpp)

if (NOT ${BUILD_SIM})
    include(Teensy)

    file(GLOB REAL_SRC
            real/*.cpp
            real/commands/*.cpp)

    file(GLOB_RECURSE REAL_LIBS_SRC
            real/lib/*.cpp)

    file(GLOB REAL_LIB_DIRS real/lib/*)

    include_directories(${CMAKE_SOURCE_DIR} ${REAL_LIB_DIRS} real real/commands)
    add_definitions(-DEMBED)

    import_arduino_library(Bounce2)
    import_arduino_library(i2c_t3)
    import_arduino_library(SPI)

    add_teensy_executable(main real/main/main.cpp ${COM_SRC} ${REAL_SRC} ${REAL_LIBS_SRC})
    set_target_properties(main PROPERTIES COMPILE_FLAGS "-include common/util.h")
else ()

    include_directories(${CMAKE_SOURCE_DIR})

    file(GLOB CONSOLE_SRC console/*.cpp console/commands/*.cpp)

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

    foreach (MAIN ${CONSOLES})
        add_executable(${MAIN} "console/main/${MAIN}.cpp")
        target_link_libraries(${MAIN} console)
        target_compile_features(${MAIN} PRIVATE cxx_strong_enums)
        set_target_properties(${MAIN} PROPERTIES COMPILE_FLAGS "-DCONSOLE")
        target_include_directories(${MAIN} PRIVATE console/ console/commands)
    endforeach ()

    ################################
    # actual real robot programs
    ################################
    add_custom_target(serial COMMAND cd ../ && platformio serialports monitor -b 9600)
    add_custom_target(upload COMMAND cd ../ && platformio run -v --target upload)

    add_custom_target(robot COMMAND cd ../ && platformio run)

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

        add_executable(runConsoleTests test/Test.cpp ${COM_SRC} ${CONSOLE_SRC})
        target_compile_features(runConsoleTests PRIVATE cxx_strong_enums)
        target_include_directories(runConsoleTests PRIVATE console/ console/commands)

        target_link_libraries(runConsoleTests gtest gtest_main)
        set_target_properties(runConsoleTests PROPERTIES COMPILE_FLAGS "-DCONSOLE")

        add_test(all-tests runConsoleTests)
    endif ()
endif ()
