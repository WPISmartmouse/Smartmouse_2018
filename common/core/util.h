#pragma once

#include <common/commanduino/TimerInterface.h>
#include <vector>

void print(const char *fmt, ...);
void csv_print(std::vector<double> values);

extern struct global_program_settings_t {
  bool quiet;
} GlobalProgramSettings;

