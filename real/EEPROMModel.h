#pragma once
#include <cstdint>

#include <common/IRSensorModeling/Model.h>

namespace smartmouse {

namespace ir {

struct EEPROMModel : public Model {
 public:
  EEPROMModel(double a, double b, double c, double d);

  void calibrate(int adc_reading, uint8_t EEPROMpos);

  void loadCalibrate(int8_t EEPROMval);
};

}
}
