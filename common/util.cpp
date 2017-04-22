#include <cstdarg>
#include <cstdio>

#ifdef ARDUINO
#include <Arduino.h>
#endif

void print(const char *fmt, ... ){
  char buf[1024]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
  vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
  va_end(args);
#ifdef ARDUINO
  Serial.print(buf);
  Serial1.print(buf);
#else
  printf("%s", buf);
#endif
}

global_program_settings_t GlobalProgramSettings;

