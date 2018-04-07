#pragma once

#include <vector>

void print(const char *fmt, ...);
void csv_print(std::vector<double> values);

#define print_slow(fmt, ...) {static unsigned long __idx=0;__idx++; if (__idx%25==0) {print(fmt, __VA_ARGS__);}}

extern struct global_program_settings_t {
  bool quiet;
  bool dead_reckoning;
} GlobalProgramSettings;

