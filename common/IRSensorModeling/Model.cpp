#include <common/IRSensorModeling/Model.h>
#include <cmath>

namespace smartmouse {

namespace ir {

double ModelParams::toMeters(int adc) {
  return a * std::pow(adc, b) + 180;
}

int ModelParams::toADC(double distance_m) {
  return static_cast<int>(std::pow((distance_m - 180) / a, 1 / b));
}

}
}
