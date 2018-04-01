#include <common/IRSensorModeling/Model.h>
#include <cmath>
#include <common/KinematicController/RobotConfig.h>

namespace smartmouse {

namespace ir {

double ModelParams::toMeters(int adc) {
  double d = a - std::pow(adc, b);
  if (d > smartmouse::kc::ANALOG_MAX_DIST_M) {
    return smartmouse::kc::ANALOG_MAX_DIST_M;
  }
  else if (d < smartmouse::kc::ANALOG_MIN_DIST_M) {
    return smartmouse::kc::ANALOG_MIN_DIST_M;
  }
  else {
    return d;
  }
}

int ModelParams::toADC(double distance_m) {
  return static_cast<int>(pow(a - distance_m, 1/b));
}

}
}
