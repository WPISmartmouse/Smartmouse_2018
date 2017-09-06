#pragma once

void print(const char *fmt, ...);

extern struct global_program_settings_t {
  bool quiet;
} GlobalProgramSettings;
