#pragma once
#include <cstdint>
namespace smartmouse {

namespace ir {

struct ModelParams {
  /**
   * We model our sensors as follows:
   *
   * distance in meters = d*(reflectivity)*c*(angle to wall)*(a+(ADC value)^b+180)
   *
   * The 180 value was determined experimentally. It could be another variable but it is not necessary
   */

  /** scales based on ADC value **/
  double a;

  /** power based on ADC value **/
  double b;

  /** a constant offset **/
  double c;

  const double CALIBRATION_DISTANCE;

  /** calibration offset **/
  int adc_offset = 0;

  int toADC(double distance_m) const;

  double toMeters(int adc) const;

  void calibrate(int adc_reading);
  void calibrate(int adc_reading, uint8_t EEPROMpos);

  void loadCalibrate(int EEPROMval);
};

}
}
