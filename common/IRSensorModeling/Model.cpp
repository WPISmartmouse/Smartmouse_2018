#include <common/IRSensorModeling/Model.h>
#include <cmath>
#include <common/KinematicController/RobotConfig.h>
#include <EEPROM.h>

namespace smartmouse {

namespace ir {

double ModelParams::toMeters(int adc) const {
  double d = a - std::pow(adc - c - adc_offset, b);
  if (std::isnan(d)) {

    return smartmouse::kc::ANALOG_MAX_DIST_M;
  }
  else if (d > smartmouse::kc::ANALOG_MAX_DIST_M) {
    return smartmouse::kc::ANALOG_MAX_DIST_M;
  }
  else if (d < smartmouse::kc::ANALOG_MIN_DIST_M) {
    return smartmouse::kc::ANALOG_MIN_DIST_M;
  }
  else {
    return d;
  }
}

int ModelParams::toADC(double distance_m) const {
  return static_cast<int>(pow(a - distance_m, 1/b) + c + adc_offset);
}

void ModelParams::calibrate(const int adc_reading) {
  adc_offset = static_cast<int8_t>(adc_reading - toADC(CALIBRATION_DISTANCE));
}
void ModelParams::calibrate(const int adc_reading, uint8_t EEPROMPos) {
  adc_offset = static_cast<int8_t>(adc_reading - toADC(CALIBRATION_DISTANCE));
  EEPROM.write(EEPROMPos, static_cast<uint8_t>(adc_offset));
}

void ModelParams::loadCalibrate(int8_t EEPROMval) {
  adc_offset = EEPROMval;
}

}
}
