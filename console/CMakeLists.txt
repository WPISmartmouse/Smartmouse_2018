file(GLOB CONSOLE_SRC *.cpp commands/*.cpp)

################################
# library for console programs
################################
add_library(console ${COM_SRC} ${CONSOLE_SRC})
target_compile_features(console PRIVATE cxx_strong_enums)
target_include_directories(console PRIVATE console)
set_target_properties(console PROPERTIES COMPILE_FLAGS "-DCONSOLE -include common/util.h")

################################
# text-only console programs
################################
set(CONSOLES Solve
        Animate
        ConsoleNavTest
        GenerateMaze
        ReadAndPrint)

foreach (MAIN ${CONSOLES})
    add_executable(${MAIN} main/${MAIN}.cpp)
    target_link_libraries(${MAIN} console)
    target_compile_features(${MAIN} PRIVATE cxx_strong_enums)
    set_target_properties(${MAIN} PROPERTIES COMPILE_FLAGS "-DCONSOLE")

    install(TARGETS ${MAIN} DESTINATION bin)
endforeach ()

################################
# testing
################################
add_executable(console_tests test/Test.cpp ${COM_SRC})
target_compile_features(console_tests PRIVATE cxx_strong_enums)
target_link_libraries(console_tests gtest gtest_main console)
set_target_properties(console_tests PROPERTIES COMPILE_FLAGS "-DCONSOLE -include common/util.h")

install(TARGETS console_tests DESTINATION bin)
