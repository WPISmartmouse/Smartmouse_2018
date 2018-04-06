#include <cmath>
#include <EEPROM.h>

#include <common/KinematicController/RobotConfig.h>
#include <real/EEPROMModel.h>

namespace smartmouse {

namespace ir {

EEPROMModel::EEPROMModel(double a, double b, double c, double d) : Model(a, b, c, d) {}

void EEPROMModel::calibrate(const int adc_reading, uint8_t EEPROMPos) {
  adc_offset = static_cast<int8_t>(adc_reading - toADC(CALIBRATION_DISTANCE));
  EEPROM.write(EEPROMPos, static_cast<uint8_t>(adc_offset));
}

void EEPROMModel::loadCalibrate(int8_t EEPROMval) {
  adc_offset = EEPROMval;
}

}
}
