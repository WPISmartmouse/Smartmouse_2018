# Copyright (c) 2015, Pierre-Andre Saulais <pasaulais@free.fr>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

find_program(TY_EXECUTABLE NAMES "teensy_loader_cli" DOC "Path to the executable that can upload programs to the Teensy")

file(GLOB_RECURSE TEENSY_C_CORE_FILES ${TEENSY_ROOT}/*.c )

file(GLOB_RECURSE TEENSY_CXX_CORE_FILES ${TEENSY_ROOT}/*.cpp )

macro(add_teensy_core)
  # Build the Teensy 'core' library.
  set(TARGET_FLAGS "-DUSB_SERIAL -DF_CPU=${TEENSY_FREQUENCY}000000 ${TEENSY_FLAGS}")
  set(TARGET_C_FLAGS "${TARGET_FLAGS} ${TEENSY_C_FLAGS}")
  set(TARGET_CXX_FLAGS "${TARGET_FLAGS} ${TEENSY_CXX_FLAGS}")
  set_source_files_properties(${TEENSY_C_CORE_FILES} PROPERTIES COMPILE_FLAGS ${TARGET_C_FLAGS})
  set_source_files_properties(${TEENSY_CXX_CORE_FILES} PROPERTIES COMPILE_FLAGS ${TARGET_CXX_FLAGS})
  add_library(TeensyCore ${TEENSY_C_CORE_FILES} ${TEENSY_CXX_CORE_FILES})
  set_source_files_properties(${TEENSY_C_CORE_FILES} PROPERTIES COMPILE_FLAGS ${TARGET_C_FLAGS})
  set_source_files_properties(${TEENSY_CXX_CORE_FILES} PROPERTIES COMPILE_FLAGS ${TARGET_CXX_FLAGS})
endmacro()

macro(add_teensy_library TARGET_NAME)
  set(TARGET_FLAGS "-DUSB_SERIAL -DF_CPU=${TEENSY_FREQUENCY}000000 ${TEENSY_FLAGS}")
  set(TARGET_C_FLAGS "${TARGET_FLAGS} ${TEENSY_C_FLAGS}")
  set(TARGET_CXX_FLAGS "${TARGET_FLAGS} ${TEENSY_CXX_FLAGS}")

  set(FINAL_SOURCES ${TEENSY_LIB_SOURCES})
  foreach (SOURCE ${ARGN})
    get_filename_component(SOURCE_EXT ${SOURCE} EXT)
    get_filename_component(SOURCE_NAME ${SOURCE} NAME_WE)
    get_filename_component(SOURCE_PATH ${SOURCE} REALPATH)
    if ((${SOURCE_EXT} STREQUAL .ino) OR (${SOURCE_EXT} STREQUAL .pde))
      # Generate a stub C++ file from the Arduino sketch file.
      set(GEN_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_NAME}.cpp")
      set(TEMPLATE_FILE "${SOURCE}.in")
      if (NOT EXISTS "${TEMPLATE_FILE}")
        set(TEMPLATE_FILE "${CMAKE_SOURCE_DIR}/cmake/Arduino.inc.in")
      endif ()
      configure_file("${TEMPLATE_FILE}" "${GEN_SOURCE}")
      set(FINAL_SOURCES ${FINAL_SOURCES} ${GEN_SOURCE})
    else ()
      set(FINAL_SOURCES ${FINAL_SOURCES} ${SOURCE})
    endif ()
  endforeach (SOURCE ${SOURCES})

  # Add the Arduino library directory to the include path if found.
  if (EXISTS ${ARDUINO_LIB_ROOT})
    include_directories(${ARDUINO_LIB_ROOT})
  endif (EXISTS ${ARDUINO_LIB_ROOT})

  # Build the ELF executable.
  add_library(${TARGET_NAME} ${FINAL_SOURCES})
  set_source_files_properties(${FINAL_SOURCES} PROPERTIES COMPILE_FLAGS ${TARGET_CXX_FLAGS})
  target_link_libraries(${TARGET_NAME} TeensyCore)
endmacro(add_teensy_library)

macro(add_teensy_executable TARGET_NAME)
  set(TARGET_FLAGS "-DUSB_SERIAL -DF_CPU=${TEENSY_FREQUENCY}000000 ${TEENSY_FLAGS}")
  set(TARGET_C_FLAGS "${TARGET_FLAGS} ${TEENSY_C_FLAGS}")
  set(TARGET_CXX_FLAGS "${TARGET_FLAGS} ${TEENSY_CXX_FLAGS}")

  set(FINAL_SOURCES)
  foreach (SOURCE ${ARGN})
    get_filename_component(SOURCE_EXT ${SOURCE} EXT)
    get_filename_component(SOURCE_NAME ${SOURCE} NAME_WE)
    get_filename_component(SOURCE_PATH ${SOURCE} REALPATH)
    if ((${SOURCE_EXT} STREQUAL .ino) OR (${SOURCE_EXT} STREQUAL .pde))
      # Generate a stub C++ file from the Arduino sketch file.
      set(GEN_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_NAME}.cpp")
      set(TEMPLATE_FILE "${SOURCE}.in")
      if (NOT EXISTS "${TEMPLATE_FILE}")
        set(TEMPLATE_FILE "${CMAKE_SOURCE_DIR}/cmake/Arduino.inc.in")
      endif ()
      configure_file("${TEMPLATE_FILE}" "${GEN_SOURCE}")
      set(FINAL_SOURCES ${FINAL_SOURCES} ${GEN_SOURCE})
    else ()
      set(FINAL_SOURCES ${FINAL_SOURCES} ${SOURCE})
    endif ()
  endforeach (SOURCE ${SOURCES})

  # Add the Arduino library directory to the include path if found.
  if (EXISTS ${ARDUINO_LIB_ROOT})
    include_directories(${ARDUINO_LIB_ROOT})
  endif (EXISTS ${ARDUINO_LIB_ROOT})

  # Build the ELF executable.
  add_executable(${TARGET_NAME} ${FINAL_SOURCES})
  set_source_files_properties(${FINAL_SOURCES} PROPERTIES COMPILE_FLAGS ${TARGET_CXX_FLAGS})
  target_link_libraries(${TARGET_NAME} TeensyCore)
  set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME ${TARGET_NAME} SUFFIX ".elf")
  set(TARGET_ELF "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.elf")

  # Generate the hex firmware files that can be flashed to the MCU.
  set(EEPROM_OPTS -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0)
  set(HEX_OPTS -O ihex -R .eeprom)
  add_custom_command(OUTPUT ${TARGET_ELF}.eep
                     COMMAND ${CMAKE_OBJCOPY} ${EEPROM_OPTS} ${TARGET_ELF} ${TARGET_ELF}.eep
                     DEPENDS ${TARGET_ELF})
  add_custom_command(OUTPUT ${TARGET_ELF}.hex
                     COMMAND ${CMAKE_OBJCOPY} ${HEX_OPTS} ${TARGET_ELF} ${TARGET_ELF}.hex
                     DEPENDS ${TARGET_ELF})
  add_custom_target(${TARGET_NAME}_Firmware ALL
                    DEPENDS ${TARGET_ELF}.eep ${TARGET_ELF}.hex)
  add_dependencies(${TARGET_NAME}_Firmware ${TARGET_NAME})

  if (NOT TY_EXECUTABLE)
    message("teensy_loader_cli does not exist! Will not generate ${TARGET_NAME}_Upload target")
  else ()
    add_custom_target(${TARGET_NAME}_Upload
                      DEPENDS ${TY_EXECUTABLE} ${TARGET_ELF}.hex
                      COMMAND "${TY_EXECUTABLE}" --mcu=TEENSY35 -w -s ${TARGET_ELF}.hex)
    add_dependencies(${TARGET_NAME}_Upload ${TARGET_NAME}_Firmware)
  endif ()
endmacro(add_teensy_executable)

macro(import_arduino_library LIB_NAME)
  # Check if we can find the library.
  if (NOT EXISTS ${ARDUINO_LIB_ROOT})
    message(FATAL_ERROR "Could not find the Arduino library directory")
  endif (NOT EXISTS ${ARDUINO_LIB_ROOT})
  set(LIB_DIR "${ARDUINO_LIB_ROOT}/${LIB_NAME}")
  if (NOT EXISTS "${LIB_DIR}")
    message(FATAL_ERROR "Could not find the directory for library '${LIB_NAME}'")
  endif (NOT EXISTS "${LIB_DIR}")

  # Add it to the include path.
  include_directories("${LIB_DIR}")

  # Mark source files to be built along with the sketch code.
  file(GLOB SOURCES_CPP ABSOLUTE "${LIB_DIR}" "${LIB_DIR}/*.cpp")
  foreach (SOURCE_CPP ${SOURCES_CPP})
    set(TEENSY_LIB_SOURCES ${TEENSY_LIB_SOURCES} ${SOURCE_CPP})
  endforeach (SOURCE_CPP ${SOURCES_CPP})
  file(GLOB SOURCES_C ABSOLUTE "${LIB_DIR}" "${LIB_DIR}/*.c")
  foreach (SOURCE_C ${SOURCES_C})
    set(TEENSY_LIB_SOURCES ${TEENSY_LIB_SOURCES} ${SOURCE_C})
  endforeach (SOURCE_C ${SOURCES_C})
endmacro(import_arduino_library)
