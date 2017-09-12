#pragma once

#include <common/commanduino/TimerInterface.h>
void print(const char *fmt, ...);

extern struct global_program_settings_t {
  bool quiet;
} GlobalProgramSettings;

