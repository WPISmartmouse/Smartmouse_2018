#pragma once

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

  /** scales based on angle to the wall **/
  double c;

  /** scales based on reflectivity of the material **/
  double d;

  int toADC(double distance_m);

  double toMeters(int adc);
};

}
}
