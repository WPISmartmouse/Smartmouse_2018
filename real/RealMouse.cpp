#include <tuple>
#include <common/Mouse.h>
#include "RealMouse.h"

RealMouse *RealMouse::instance = nullptr;
const RobotConfig RealMouse::CONFIG = {
        1.35255, // FRONT_ANALOG_ANGLE
        1.22,  // BACK_ANALOG_ANGLE TODO: CHANGE THIS TO MATCH REAL ROBOT
        0.04,    // FRONT_SIDE_ANALOG_X
        0.024,    // FRONT_SIDE_ANALOG_Y
        -0.024,  // BACK_SIDE_ANALOG_X
        0.024,  // BACK_SIDE_ANALOG_Y
        0.045,   // FRONT_ANALOG_X
        0.09,    // MAX_SPEED
        0.021,    // MIN_SPEED
        0.15,    // WALL_THRESHOLD
};
const int RealMouse::ir_lookup[18] = {
        751, // .01
        653, // .02
        522, // .03
        411, // .04
        313, // .05
        277, // .06
        227, // .07
        206, // .08
        160, // .09
        148, // .10
        125, // .11
        106, // .12
        98,  // .13
        86,  // .14
        77,  // .15
        71,  // .16
        60,  // .17
        53,  // .18
};

double RealMouse::tick_to_rad(int ticks) {
  // if in quadrent I or II, it's positive
  double rad = ticks * RAD_PER_TICK;
  return rad;
}

double RealMouse::adcToMeters(int adc) {
  if (adc > 751) {
    return 0.01;
  } else if (adc <= 53) {
    return 0.18;
  } else {
    for (int i = 1; i < 18; i++) {
      if (adc >= ir_lookup[i]) {
        // linear map between i and i+1, .01 meters spacing
        double o = (adc - ir_lookup[i]) * 0.01 / (ir_lookup[i] - ir_lookup[i - 1]) + (i + 1) * .01;
        return o;
      }
    }
    return 0.18;
  }
}

RangeData RealMouse::getRangeData() {
    RangeData sr;
    sr.front_left_analog = adcToMeters(analogRead(FRONT_LEFT_ANALOG_PIN));
    sr.back_left_analog = adcToMeters(analogRead(BACK_LEFT_ANALOG_PIN));
//    sr.front_right_analog = adcToMeters(analogRead(FRONT_RIGHT_ANALOG_PIN));
    sr.front_right_analog = .18;
//    sr.back_right_analog = adcToMeters(analogRead(BACK_RIGHT_ANALOG_PIN));
    sr.back_right_analog = .18;
//    sr.front_analog = adcToMeters(analogRead(FRONT_ANALOG_PIN));
    sr.front_analog = .18;
    return sr;
  }

  RealMouse *RealMouse::inst() {
    if (instance == NULL) {
      instance = new RealMouse();
    }
    return instance;
  }

  RealMouse::RealMouse() : ignore_sensor_pose_estimate(false) {}

  SensorReading RealMouse::checkWalls() {
    SensorReading sr(row, col);
    sr.walls[static_cast<int>(dir)] = analogRead(FRONT_ANALOG_PIN) < 0.15;
    sr.walls[static_cast<int>(left_of_dir(dir))] = analogRead(FRONT_LEFT_ANALOG_PIN) < 0.15;
    sr.walls[static_cast<int>(right_of_dir(dir))] = analogRead(FRONT_RIGHT_ANALOG_PIN) < 0.15;
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
    return estimated_pose;
  }

  std::pair<double, double> RealMouse::getWheelVelocities() {
    return kinematic_controller.getWheelVelocities();
  };

  void RealMouse::run(double dt_s) {
    double abstract_left_force, abstract_right_force;
    double left_angle_rad = tick_to_rad(left_encoder.read());
    double right_angle_rad = tick_to_rad(right_encoder.read());
    std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s, left_angle_rad,
                                                                                   right_angle_rad, 0, 0);

    // update row/col information
    row = (int) (estimated_pose.y / AbstractMaze::UNIT_DIST);
    col = (int) (estimated_pose.x / AbstractMaze::UNIT_DIST);
    row_offset_to_edge = fmod(estimated_pose.y, AbstractMaze::UNIT_DIST);
    col_offset_to_edge = fmod(estimated_pose.x, AbstractMaze::UNIT_DIST);

    if (abstract_left_force < 0) {
      analogWrite(MOTOR1A, (int) -abstract_left_force);
      analogWrite(MOTOR1B, 0);
    } else {
      analogWrite(MOTOR1A, 0);
      analogWrite(MOTOR1B, (int) abstract_left_force);
    }

    if (abstract_right_force < 0) {
      analogWrite(MOTOR2B, 0);
      analogWrite(MOTOR2A, (int) -abstract_right_force);
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

    kinematic_controller.reset_x_to(0.06);
    kinematic_controller.reset_y_to(0.09);
    kinematic_controller.reset_yaw_to(0.0);
    kinematic_controller.setAcceleration(0.2, 1.0);

    // Teensy does USB in software, so serial rate doesn't do anything
    Serial.begin(0);
  }

  void RealMouse::setSpeed(double l_mps, double r_mps) {
    kinematic_controller.setSpeedMps(l_mps, r_mps);
  }
