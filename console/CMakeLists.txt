file(GLOB CONSOLE_SRC *.cpp commands/*.cpp)

################################
# library for console programs
################################
add_library(console ${CORE_COM_SRC} ${COM_SRC} ${COM_COMMAND_SRC} ${CONSOLE_SRC})
target_include_directories(console PRIVATE console)
set_target_properties(console PROPERTIES COMPILE_FLAGS "-DCONSOLE -include ${UTIL_HEADER}")

################################
# text-only console programs
################################
set(CONSOLES ConsoleSolve
    Animate
    GenerateMaze
    ReadAndPrint)

foreach (MAIN ${CONSOLES})
  add_executable(${MAIN} main/${MAIN}.cpp)
  target_link_libraries(${MAIN} console)
  set_target_properties(${MAIN} PROPERTIES COMPILE_FLAGS "-DCONSOLE")

  install(TARGETS ${MAIN} DESTINATION bin)
endforeach ()

################################
# testing
################################
add_executable(console_tests test/Test.cpp ${COM_SRC})
target_link_libraries(console_tests gtest gtest_main console)
set_target_properties(console_tests PROPERTIES COMPILE_FLAGS "-DCONSOLE -include ${UTIL_HEADER}")


install(TARGETS console_tests DESTINATION bin)
