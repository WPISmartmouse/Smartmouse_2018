#include <tuple>
#include <common/core/Mouse.h>
#include "RealMouse.h"

RealMouse *RealMouse::instance = nullptr;

IRConverter::IRConverter() : ir_lookup{
        755, // .01
        648, // .02
        492, // .03
        409, // .04
        315, // .05
        268, // .06
        224, // .07
        192, // .08
        165, // .09
        147, // .10
        134, // .11
        115, // .12
        105, // .13
        93,  // .14
        86,  // .15
        75,  // .16
        68,  // .17
        58,  // .18
} {}

void IRConverter::calibrate(int avg_adc_value_on_center) {
  double actual_dist = (0.08 - config.FRONT_SIDE_ANALOG_Y) / sin(config.FRONT_ANALOG_ANGLE);
  int centered_idx = 4;
  double expected_distance = (avg_adc_value_on_center - ir_lookup[centered_idx]) * 0.01 /
                             (ir_lookup[centered_idx] - ir_lookup[centered_idx - 1]) + (centered_idx + 1) * .01;
  calibration_offset = actual_dist - expected_distance;
}

double IRConverter::adcToMeters(int adc) {
  if (adc > 751) {
    return 0.01;
  } else if (adc <= 53) {
    return 0.18;
  } else {
    for (int i = 1; i < 18; i++) {
      if (adc >= ir_lookup[i]) {
        // linear map between i and i+1, .01 meters spacing
        double o = (adc - ir_lookup[i]) * 0.01 / (ir_lookup[i] - ir_lookup[i - 1]) + (i + 1) * .01 + calibration_offset;
        return o;
      }
    }
    return 0.18;
  }
}

double RealMouse::tick_to_rad(int ticks) {
  // if in quadrant I or II, it's positive
  double rad = ticks * RAD_PER_TICK;
  return rad;
}

RealMouse *RealMouse::inst() {
  if (instance == NULL) {
    instance = new RealMouse();
  }
  return instance;
}

RealMouse::RealMouse() : kinematic_controller(this), range_data({0.18, 0.18, 0.18, 0.18, 0.18}) {}

RangeData RealMouse::getRangeData() {
  return range_data;
}

SensorReading RealMouse::checkWalls() {
  SensorReading sr(row, col);

  sr.walls[static_cast<int>(dir)] = range_data.front < 0.17;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data.gerald_left < 0.15;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data.gerald_right < 0.15;
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

double RealMouse::getColOffsetToEdge() {
  return kinematic_controller.col_offset_to_edge;
}

double RealMouse::getRowOffsetToEdge() {
  return kinematic_controller.row_offset_to_edge;
}

GlobalPose RealMouse::getGlobalPose() {
  return kinematic_controller.getGlobalPose();
}

LocalPose RealMouse::getLocalPose() {
  return kinematic_controller.getLocalPose();
}

std::pair<double, double> RealMouse::getWheelVelocities() {
  return kinematic_controller.getWheelVelocities();
};

void RealMouse::run(double dt_s) {
  double abstract_left_force, abstract_right_force;
  left_angle_rad = tick_to_rad(left_encoder.read());
  right_angle_rad = tick_to_rad(right_encoder.read());

  range_data.gerald_left = ir_converter.adcToMeters(analogRead(GERALD_LEFT_ANALOG_PIN));
  range_data.gerald_right = ir_converter.adcToMeters(analogRead(GERALD_RIGHT_ANALOG_PIN));
  range_data.front_left = ir_converter.adcToMeters(analogRead(FRONT_LEFT_ANALOG_PIN));
  range_data.back_left = ir_converter.adcToMeters(analogRead(BACK_LEFT_ANALOG_PIN));
  range_data.front_right = ir_converter.adcToMeters(analogRead(FRONT_RIGHT_ANALOG_PIN));
  range_data.back_right = ir_converter.adcToMeters(analogRead(BACK_RIGHT_ANALOG_PIN));
  range_data.front = ir_converter.adcToMeters(analogRead(FRONT_ANALOG_PIN));

  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s, left_angle_rad,
                                                                                 right_angle_rad, 0, 0, range_data);

  // THIS IS SUPER IMPORTANT!
  // update row/col information
  row = kinematic_controller.row;
  col = kinematic_controller.col;

  if (abstract_left_force < 0) {
    analogWrite(MOTOR_LEFT_A, (int) -abstract_left_force);
    analogWrite(MOTOR_LEFT_B, 0);
  } else {
    analogWrite(MOTOR_LEFT_A, 0);
    analogWrite(MOTOR_LEFT_B, (int) abstract_left_force);
  }

  if (abstract_right_force < 0) {
    analogWrite(MOTOR_RIGHT_B, 0);
    analogWrite(MOTOR_RIGHT_A, (int) -abstract_right_force);
  } else {
    analogWrite(MOTOR_RIGHT_B, (int) abstract_right_force);
    analogWrite(MOTOR_RIGHT_A, 0);
  }
}

void RealMouse::setup() {
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
  pinMode(LED_7, OUTPUT);
  pinMode(LED_8, OUTPUT);
  pinMode(SYS_LED, OUTPUT);
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

//  analogWriteFrequency(MOTOR_LEFT_A, 1831.055);
//  analogWriteFrequency(MOTOR_LEFT_B, 1831.055);
//  analogWriteFrequency(MOTOR_RIGHT_A, 1831.055);
//  analogWriteFrequency(MOTOR_RIGHT_B, 1831.055);
//  analogReadResolution(13); TODO: get more bits baby

  left_encoder.init(ENCODER_LEFT_A, ENCODER_LEFT_B);
  right_encoder.init(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

  kinematic_controller.setAccelerationMpss(2);

  resetToStartPose();

  // Teensy does USB in software, so serial rate doesn't do anything
  Serial.begin(0);
  // But it does here because it's not the USB serial
  Serial1.begin(115200);
}

void RealMouse::resetToStartPose() {
  reset(); // resets row, col, and dir
  left_angle_rad = tick_to_rad(left_encoder.read());
  right_angle_rad = tick_to_rad(right_encoder.read());
  kinematic_controller.left_motor.reset_enc_rad(left_angle_rad);
  kinematic_controller.right_motor.reset_enc_rad(right_angle_rad);
  kinematic_controller.reset_x_to(0.09);
  kinematic_controller.reset_y_to(0.09);
  kinematic_controller.reset_yaw_to(0.0);
}

void RealMouse::setSpeed(double l_mps, double r_mps) {
  kinematic_controller.setSpeedMps(l_mps, r_mps);
}
