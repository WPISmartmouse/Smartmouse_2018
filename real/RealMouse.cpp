#include <tuple>
#include <common/core/Mouse.h>
#include "RealMouse.h"

RealMouse *RealMouse::instance = nullptr;

RealMouse *RealMouse::inst() {
  if (instance == NULL) {
    instance = new RealMouse();
  }
  return instance;
}

RealMouse::RealMouse()
    : kinematic_controller(this),
      range_data_adc({}),
      range_data_m({}),
      left_encoder(LEFT_ENCODER_CS),
      right_encoder(RIGHT_ENCODER_CS) {}

SensorReading RealMouse::checkWalls() {
  SensorReading sr(row, col);

  sr.walls[static_cast<int>(dir)] = range_data_m.front < 0.17;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data_m.gerald_left < 0.15;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data_m.gerald_right < 0.15;
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

GlobalPose RealMouse::getGlobalPose() {
  return kinematic_controller.getGlobalPose();
}

LocalPose RealMouse::getLocalPose() {
  return kinematic_controller.getLocalPose();
}

std::pair<double, double> RealMouse::getWheelVelocities() {
  return kinematic_controller.getWheelVelocitiesCPS();
};

void RealMouse::run(double dt_s) {
  double abstract_left_force, abstract_right_force;
  left_angle_rad = tick_to_rad(left_encoder.getRotation());
  right_angle_rad = tick_to_rad(right_encoder.getRotation());

#ifdef PROFILE
  unsigned long t0 = micros();
#endif

  range_data_adc.back_left = analogRead(BACK_LEFT_ANALOG_PIN);
  range_data_adc.front_left = analogRead(FRONT_LEFT_ANALOG_PIN);
  range_data_adc.gerald_left = analogRead(GERALD_LEFT_ANALOG_PIN);
  range_data_adc.front = analogRead(FRONT_ANALOG_PIN);
  range_data_adc.gerald_right = analogRead(GERALD_RIGHT_ANALOG_PIN);
  range_data_adc.front_right = analogRead(FRONT_RIGHT_ANALOG_PIN);
  range_data_adc.back_right = analogRead(BACK_RIGHT_ANALOG_PIN);
  range_data_m.back_left = back_left_model.toMeters(range_data_adc.back_left);
  range_data_m.front_left = front_left_model.toMeters(range_data_adc.front_left);
  range_data_m.gerald_left = gerald_left_model.toMeters(range_data_adc.gerald_left);
  range_data_m.front = front_model.toMeters(range_data_adc.front);
  range_data_m.gerald_right = gerald_right_model.toMeters(range_data_adc.gerald_right);
  range_data_m.front_right = front_right_model.toMeters(range_data_adc.front_right);
  range_data_m.back_right = back_right_model.toMeters(range_data_adc.back_right);

#ifdef PROFILE
  Serial.print("Sensors, ");
  Serial.println(micros() - t0);
#endif

#ifdef PROFILE
  unsigned long t1 = micros();
#endif
  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s, left_angle_rad,
                                                                                 right_angle_rad, range_data_m);
#ifdef PROFILE
  Serial.print("KC, ");
  Serial.println(micros() - t1);
#endif

  // THIS IS SUPER IMPORTANT!
  // update row/col information
  row = kinematic_controller.row;
  col = kinematic_controller.col;

#ifdef PROFILE
  unsigned long t2 = micros();
#endif
  if (abstract_left_force < 0) {
    analogWrite(MOTOR_LEFT_A1, 0);
    analogWrite(MOTOR_LEFT_A2, (int) -abstract_left_force);
  } else {
    analogWrite(MOTOR_LEFT_A1, (int) abstract_left_force);
    analogWrite(MOTOR_LEFT_A2, 0);
  }

  if (abstract_right_force < 0) {
    analogWrite(MOTOR_RIGHT_B2, 0);
    analogWrite(MOTOR_RIGHT_B1, (int) -abstract_right_force);
  } else {
    analogWrite(MOTOR_RIGHT_B2, (int) abstract_right_force);
    analogWrite(MOTOR_RIGHT_B1, 0);
  }
#ifdef PROFILE
  Serial.print("Motors, ");
  Serial.println(micros() - t2);
#endif
}

void RealMouse::setup() {
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
  pinMode(LED_7, OUTPUT);
  pinMode(SYS_LED, OUTPUT);
  pinMode(MOTOR_LEFT_A1, OUTPUT);
  pinMode(MOTOR_LEFT_A2, OUTPUT);
  pinMode(MOTOR_RIGHT_B1, OUTPUT);
  pinMode(MOTOR_RIGHT_B2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  left_encoder.init();
  right_encoder.init();
  left_encoder.invert();

  // pull MOSI high to run encoders in 3 wire mode
  pinMode(MOSI, OUTPUT);
  digitalWrite(MOSI, HIGH);

  analogWriteFrequency(MOTOR_LEFT_A1, 375000);
  analogWriteFrequency(MOTOR_LEFT_A2, 375000);
  analogWriteFrequency(MOTOR_RIGHT_B1, 375000);
  analogWriteFrequency(MOTOR_RIGHT_B2, 375000);
  analogReadResolution(13);

  kinematic_controller.setAccelerationCpss(10);

  resetToStartPose();

  // Teensy does USB in software, so serial rate doesn't do anything
  Serial.begin(0);
  // But it does here because it's not the USB serial
  Serial1.begin(115200);
  delay(1000);

  // IR sensor calibration
  // calibrate();
}

void RealMouse::resetToStartPose() {
  reset(); // resets row, col, and dir
  left_encoder.ResetPosition();
  right_encoder.ResetPosition();
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(0.0);
}

void RealMouse::setSpeedCps(double l_cps, double r_cps) {
  kinematic_controller.setSpeedCps(l_cps, r_cps);
}

double RealMouse::checkVoltage() {
  // 3.2v is max and 2.7v is min
  int a = analogRead(BATTERY_ANALOG_PIN);
  double voltage = a / std::pow(2, 13) * 3.3;

  if (2 < voltage && voltage < 2.7) {
    print("VOLTAGE IS TOO LOW. CHARGE THE BATTERY!!!\r\n");
  } else if (voltage > 3.3) {
    print("VOLTAGE [%f] IS TOO HIGH. SHE'S GONNA BLOW!!!\r\n", voltage);
  }

  return voltage;
}

void RealMouse::calibrate() {
  digitalWrite(LED_5, HIGH);

  // read the latest values
  range_data_adc.back_left = analogRead(BACK_LEFT_ANALOG_PIN);
  range_data_adc.front_left = analogRead(FRONT_LEFT_ANALOG_PIN);
  range_data_adc.gerald_left = analogRead(GERALD_LEFT_ANALOG_PIN);
  range_data_adc.front = analogRead(FRONT_ANALOG_PIN);
  range_data_adc.gerald_right = analogRead(GERALD_RIGHT_ANALOG_PIN);
  range_data_adc.front_right = analogRead(FRONT_RIGHT_ANALOG_PIN);
  range_data_adc.back_right = analogRead(BACK_RIGHT_ANALOG_PIN);

  // compute offsets between measured and expected ADC value
  back_left_model.calibrate(range_data_adc.back_left);
  front_left_model.calibrate(range_data_adc.front_left);
  gerald_left_model.calibrate(range_data_adc.gerald_left);
  front_model.calibrate(range_data_adc.front);
  gerald_right_model.calibrate(range_data_adc.gerald_right);
  front_right_model.calibrate(range_data_adc.front_right);
  back_right_model.calibrate(range_data_adc.back_right);

  digitalWrite(LED_5, LOW);
}
