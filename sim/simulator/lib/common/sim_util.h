#pragma once

#include <string>
#include <time.h>
#include <chrono>

namespace smartmouse {
namespace simulator {

inline std::string date_str() {
  char datestr[50]{0};
  auto now = std::time(nullptr);
  std::strftime(datestr, sizeof(datestr), "%H:%M:%S_%d_%m_%Y", std::localtime(&now));
  return std::string(datestr);
}

}
}
