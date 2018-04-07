#pragma once
#include <cstdint>

namespace smartmouse {

namespace ir {

/**
 * We model our sensors as follows:
 *
 * distance in meters = a - (adc - c - adc_offset)^b;
 *
 * The 180 value was determined experimentally. It could be another variable but it is not necessary
 */
struct Model {
  Model(double a, double b, double c, double d);

  /** scales based on ADC value **/
  double a;

  /** power based on ADC value **/
  double b;

  /** a constant offset **/
  double c;

  const double CALIBRATION_DISTANCE;

  /** calibration offset **/
  int8_t adc_offset = 0;

  int toADC(double distance_m) const;

  double toMeters(int adc) const;

  void calibrate(int adc_reading);
};

}
}
