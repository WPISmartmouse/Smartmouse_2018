#pragma once

#include <common/math/Functor.h>
#include <common/Eigen/Eigen.h>
#include "KinematicController.h"

namespace smartmouse {
namespace kc {

constexpr size_t kParamsPerSensor = 3;
constexpr size_t kSensors = 6;
constexpr size_t kWidths = 9;
constexpr size_t kPositions = 2;
constexpr size_t kSamples = kPositions * 36; // kWidths choose 2

typedef Eigen::Matrix<double, kSensors, 1> SensorVector;
typedef Eigen::Matrix<double, kPositions, kSensors> SampleMatrix;

class CalibrationFunctor : public Functor<double> {
 private:
  SampleMatrix samples;

 public:

  // Function<n, m>, n is number of parameters. m is number of errors (comparisons of widths)
  CalibrationFunctor(SampleMatrix samples) : Functor<double>(kParamsPerSensor * kSensors, kSamples), samples(samples) {}

  /**
   * Implementation of the objective function
   * @param x A array of parameters. n by 1
   * @param fvec An output array of the errors, which is 36
   * @return
   */
  int operator()(Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {

    unsigned int fvec_idx = 0;

    Eigen::Matrix<SensorPose, kSensors, 2> sensors;
    sensors(0, 0) = BACK_LEFT;
    sensors(0, 1) = FRONT_LEFT;
    sensors(1, 0) = BACK_LEFT;
    sensors(1, 1) = GERALD_LEFT;
    sensors(2, 0) = FRONT_LEFT;
    sensors(2, 1) = GERALD_LEFT;
    sensors(3, 0) = BACK_RIGHT;
    sensors(3, 1) = FRONT_RIGHT;
    sensors(4, 0) = BACK_RIGHT;
    sensors(4, 1) = GERALD_RIGHT;
    sensors(5, 0) = FRONT_RIGHT;
    sensors(5, 1) = GERALD_RIGHT;

    for (unsigned int sample_idx = 0; sample_idx < kPositions; ++sample_idx) {
      // samples contains 6 ADC readings from the sensors, starting from back left, going clockwise around the robot
      // reshape into a 2 by 6 matrix where each column is a pair of sensors we want to compute a distance to wall with
      Eigen::Matrix<double, kSensors, 2> adc;
      adc(0, 0) = samples(sample_idx, 0);
      adc(0, 1) = samples(sample_idx, 1);
      adc(1, 0) = samples(sample_idx, 0);
      adc(1, 1) = samples(sample_idx, 2);
      adc(2, 0) = samples(sample_idx, 1);
      adc(2, 1) = samples(sample_idx, 2);
      adc(3, 0) = samples(sample_idx, 5);
      adc(3, 1) = samples(sample_idx, 4);
      adc(4, 0) = samples(sample_idx, 5);
      adc(4, 1) = samples(sample_idx, 3);
      adc(5, 0) = samples(sample_idx, 4);
      adc(5, 1) = samples(sample_idx, 3);

      Eigen::Matrix<double, kSensors, 1> distances;

      // compute distance for each pair of sensor
      for (size_t i = 0; i < kSensors; ++i) {
        const double a = x(i * 3 + 0);
        const double b = x(i * 3 + 1);
        const double c = x(i * 3 + 2);
        const double s1_adc = adc(i, 0);
        const double s2_adc = adc(i, 1);
        // we want to convet ADC -> meters
        // this applies the inverse of that, to go from meters -> adc
        const double s1_dist_m = a * std::pow(s1_adc, -b) + c;
        const double s2_dist_m = a * std::pow(s2_adc, -b) + c;
        double distance;
        std::tie(distance, std::ignore) = from_sensors_to_wall(sensors(i, 0), sensors(i, 1), s1_dist_m, s2_dist_m);
        distances(i) = distance;
        std::cout << i << ", " << distances(i) << "\n";
      }

      // add up the all combinations of sensors from left and right side
      Eigen::Matrix<double, 9, 1> widths;
      size_t k = 0;
      for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 3; j < 6; ++j) {
          widths(k) = distances(i) + distances(j);
          ++k;
        }
      }

      // the LM error terms are all the pairwise differences between each of the 9 calculated widths
      std::cout << "======\n";
      for (size_t i = 0; i < 9; i++) {
        for (size_t j = i + 1; j < 9; j++) {
          fvec(fvec_idx) = std::fabs(widths(i) - widths(j));
          ++fvec_idx;
        }
      }
    }
    std::cout << "\n";

    return 0;
  }
};

}
}
