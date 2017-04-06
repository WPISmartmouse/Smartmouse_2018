#include <tuple>
#include "RealMouse.h"

RealMouse *RealMouse::instance = nullptr;

double RealMouse::tick_to_rad(int ticks) {
  // if in quadrent I or II, it's positive
  double rad = ticks * RAD_PER_TICK;
  return rad;
}

RealMouse *RealMouse::inst() {
  if (instance == NULL) {
    instance = new RealMouse();
  }
  return instance;
}

RealMouse::RealMouse() {}

SensorReading RealMouse::checkWalls() {
  SensorReading sr(row, col);
  sr.walls[static_cast<int>(dir)] = digitalRead(FRONT_BINARY_PIN);
  sr.walls[static_cast<int>(left_of_dir(dir))] = digitalRead(LEFT_BINARY_PIN);
  sr.walls[static_cast<int>(right_of_dir(dir))] = digitalRead(RIGHT_BINARY_PIN);
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

double RealMouse::getColOffsetToEdge() {
  return col_offset_to_edge;
}

double RealMouse::getRowOffsetToEdge() {
  return row_offset_to_edge;
}

Pose RealMouse::getPose() {
  return kinematic_controller.getPose();
}

void RealMouse::run(double dt_s) {
  double abstract_left_force, abstract_right_force;
  double left_angle_rad = tick_to_rad(left_encoder.read());
  double right_angle_rad = tick_to_rad(right_encoder.read());
  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s, left_angle_rad, right_angle_rad,
                                                                                 0, 0);

  abstract_left_force = 30;
  abstract_right_force = 0;

  // update row/col information
  Pose estimated_pose = kinematic_controller.getPose();
  row = (int) (estimated_pose.y / AbstractMaze::UNIT_DIST);
  col = (int) (estimated_pose.x / AbstractMaze::UNIT_DIST);

  row_offset_to_edge = fmod(estimated_pose.y, AbstractMaze::UNIT_DIST);
  col_offset_to_edge = fmod(estimated_pose.x, AbstractMaze::UNIT_DIST);

  if (abstract_left_force < 0) {
    analogWrite(MOTOR1A, (int) abstract_left_force);
    analogWrite(MOTOR1B, 0);
  } else {
    analogWrite(MOTOR1A, 0);
    analogWrite(MOTOR1B, (int) abstract_left_force);
  }

  if (abstract_right_force < 0) {
    analogWrite(MOTOR2B, 0);
    analogWrite(MOTOR2A, (int) abstract_right_force);
  } else {
    analogWrite(MOTOR2B, (int) abstract_right_force);
    analogWrite(MOTOR2A, 0);
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
  pinMode(SYS_LED, OUTPUT);
  pinMode(MOTOR1A, OUTPUT);
  pinMode(MOTOR1B, OUTPUT);
  pinMode(MOTOR2A, OUTPUT);
  pinMode(MOTOR2B, OUTPUT);
  pinMode(ENCODER1A, INPUT_PULLUP);
  pinMode(ENCODER1B, INPUT_PULLUP);
  pinMode(ENCODER2A, INPUT_PULLUP);
  pinMode(ENCODER2B, INPUT_PULLUP);

  left_encoder.init(ENCODER1A, ENCODER1B);
  right_encoder.init(ENCODER2A, ENCODER2B);

  kinematic_controller.setAcceleration(1, 1);
  kinematic_controller.setSpeedMps(0.05, 0.05);

  // Teensy does USB in software, so serial rate doesn't do anything
  Serial.begin(0);
}
