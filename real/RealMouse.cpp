#include <tuple>
#include <common/core/Mouse.h>
#include "RealMouse.h"
#include <EEPROM.h>

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

  sr.walls[static_cast<int>(dir)] = range_data_m.front < smartmouse::kc::ANALOG_MAX_DIST_M;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data_m.gerald_left < smartmouse::kc::ANALOG_MAX_DIST_M;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data_m.gerald_right < smartmouse::kc::ANALOG_MAX_DIST_M;
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

GlobalPose RealMouse::getGlobalPose() {
  return kinematic_controller.getGlobalPose();
}

LocalPose RealMouse::getLocalPose() {
  return kinematic_controller.getLocalPose();
}

std::pair<double, double> RealMouse::getWheelVelocitiesCPS() {
  return kinematic_controller.getWheelVelocitiesCPS();
};

void RealMouse::run(double dt_s) {
  double abstract_left_force, abstract_right_force;
  left_angle_rad = tick_to_rad(left_encoder.getRotation());
  right_angle_rad = tick_to_rad(right_encoder.getRotation());

#ifdef PROFILE
  unsigned long t0 = micros();
#endif

  readSensors();

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
  pinMode(BACK_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(FRONT_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(GERALD_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(FRONT_ENABLE_PIN, OUTPUT);
  pinMode(GERALD_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(BACK_RIGHT_ENABLE_PIN, OUTPUT);

  digitalWrite(BACK_LEFT_ENABLE_PIN, LOW);
  digitalWrite(FRONT_LEFT_ENABLE_PIN, LOW);
  digitalWrite(GERALD_LEFT_ENABLE_PIN, LOW);
  digitalWrite(FRONT_ENABLE_PIN, LOW);
  digitalWrite(GERALD_RIGHT_ENABLE_PIN, LOW);
  digitalWrite(FRONT_RIGHT_ENABLE_PIN, LOW);
  digitalWrite(BACK_RIGHT_ENABLE_PIN, LOW);

  left_encoder.init();
  right_encoder.init();
  left_encoder.invert();

  analogWriteResolution(10);

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
  // But it does matter here because it's not the USB serial
  Serial1.begin(115200);

  if (!digitalRead(RealMouse::BUTTON_PIN)) {
    digitalWrite(RealMouse::LED_7, 1);
    delay(1000);
    while (digitalRead(RealMouse::BUTTON_PIN)) {}
    calibrate();
    delay(1000);
  } else {
    delay(1000);
    loadCalibrate();
  }
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

void RealMouse::readSensors() {
  digitalWriteFast(BACK_LEFT_ENABLE_PIN, HIGH);
  range_data_adc.back_left = analogRead(BACK_LEFT_ANALOG_PIN);
  digitalWriteFast(BACK_LEFT_ENABLE_PIN, LOW);

  digitalWriteFast(FRONT_LEFT_ENABLE_PIN, HIGH);
  range_data_adc.front_left = analogRead(FRONT_LEFT_ANALOG_PIN);
  digitalWriteFast(FRONT_LEFT_ENABLE_PIN, LOW);

  digitalWriteFast(GERALD_LEFT_ENABLE_PIN, HIGH);
  range_data_adc.gerald_left = analogRead(GERALD_LEFT_ANALOG_PIN);
  digitalWriteFast(GERALD_LEFT_ENABLE_PIN, LOW);

  digitalWriteFast(FRONT_ENABLE_PIN, HIGH);
  range_data_adc.front = analogRead(FRONT_ANALOG_PIN);
  digitalWriteFast(FRONT_ENABLE_PIN, LOW);

  digitalWriteFast(GERALD_RIGHT_ENABLE_PIN, HIGH);
  range_data_adc.gerald_right = analogRead(GERALD_RIGHT_ANALOG_PIN);
  digitalWriteFast(GERALD_RIGHT_ENABLE_PIN, LOW);

  digitalWriteFast(FRONT_RIGHT_ENABLE_PIN, HIGH);
  range_data_adc.front_right = analogRead(FRONT_RIGHT_ANALOG_PIN);
  digitalWriteFast(FRONT_RIGHT_ENABLE_PIN, LOW);

  digitalWriteFast(BACK_RIGHT_ENABLE_PIN, HIGH);
  range_data_adc.back_right = analogRead(BACK_RIGHT_ANALOG_PIN);
  digitalWriteFast(BACK_RIGHT_ENABLE_PIN, LOW);

  range_data_m.back_left = back_left_model.toMeters(range_data_adc.back_left);
  range_data_m.front_left = front_left_model.toMeters(range_data_adc.front_left);
  range_data_m.gerald_left = gerald_left_model.toMeters(range_data_adc.gerald_left);
  range_data_m.front = front_model.toMeters(range_data_adc.front);
  range_data_m.gerald_right = gerald_right_model.toMeters(range_data_adc.gerald_right);
  range_data_m.front_right = front_right_model.toMeters(range_data_adc.front_right);
  range_data_m.back_right = back_right_model.toMeters(range_data_adc.back_right);
}

void RealMouse::Teleop() {
  if (Serial.available()) {
    double left = 0, right = 0;
    char c = static_cast<char>(Serial.read());
    if (c == 'w') {
      left = 0.3;
      right = 0.3;
    } else if (c == 'a') {
      left = 0.30;
      right = 0.0;
    } else if (c == 'd') {
      left = 0.0;
      right = 0.3;
    } else if (c == 's') {
      left = 0;
      right = 0;

    }
    setSpeedCps(left, right);
  }
}

double RealMouse::checkVoltage() {
  // 3.2v is max and 2.7v is min
  int a = analogRead(BATTERY_ANALOG_PIN);
  double voltage = a / std::pow(2, 13) * 3.3;

  if ( voltage < 2.7) {
    print("VOLTAGE [%f] IS TOO LOW. CHARGE THE BATTERY!!!\r\n", voltage);
  } else if (voltage > 3.3) {
    print("VOLTAGE [%f] IS TOO HIGH. SHE'S GONNA BLOW!!!\r\n", voltage);
  }

  return voltage;
}

void RealMouse::calibrate() {



  // read the latest values
  digitalWrite(LED_6, HIGH);
  range_data_adc.back_left = analogRead(BACK_LEFT_ANALOG_PIN);
  range_data_adc.front_left = analogRead(FRONT_LEFT_ANALOG_PIN);
  range_data_adc.gerald_left = analogRead(GERALD_LEFT_ANALOG_PIN);
  range_data_adc.front = analogRead(FRONT_ANALOG_PIN);
  range_data_adc.gerald_right = analogRead(GERALD_RIGHT_ANALOG_PIN);
  range_data_adc.front_right = analogRead(FRONT_RIGHT_ANALOG_PIN);
  range_data_adc.back_right = analogRead(BACK_RIGHT_ANALOG_PIN);

  // compute offsets between measured and expected ADC value
  back_left_model.calibrate(range_data_adc.back_left, 1);
  front_left_model.calibrate(range_data_adc.front_left, 2);
  gerald_left_model.calibrate(range_data_adc.gerald_left, 3);
  front_model.calibrate(range_data_adc.front, 4);
  gerald_right_model.calibrate(range_data_adc.gerald_right, 5);
  front_right_model.calibrate(range_data_adc.front_right, 6);
  back_right_model.calibrate(range_data_adc.back_right, 7);
  digitalWrite(LED_6, LOW);
}

void RealMouse::loadCalibrate() {
  back_left_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(1)));
  front_left_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(2)));
  gerald_left_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(3)));
  front_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(4)));
  gerald_right_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(5)));
  front_right_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(6)));
  back_right_model.loadCalibrate(static_cast<int8_t>(EEPROM.read(7)));
}
