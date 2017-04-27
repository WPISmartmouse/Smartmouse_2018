#pragma once

typedef struct robot_config_t {
  double FRONT_ANALOG_ANGLE;
  double BACK_ANALOG_ANGLE;
  double FRONT_SIDE_ANALOG_X;
  double FRONT_SIDE_ANALOG_Y;
  double BACK_SIDE_ANALOG_X;
  double BACK_SIDE_ANALOG_Y;
  double FRONT_ANALOG_X;
  double GERALD_X;
  double GERALD_Y;
  double GERALD_ANGLE;
  double MAX_SPEED;
  double MAX_HARDWARE_SPEED;
  double MIN_SPEED;
  double FRONT_WALL_THRESHOLD;
  double SIDE_WALL_THRESHOLD;
  double GERALD_WALL_THRESHOLD;
  double WALL_CHANGED_THRESHOLD;
  double ROT_TOLERANCE;
  double TRACK_WIDTH;
  double ANALOG_MAX_DIST;
  double MAX_FORCE;
} RobotConfig;

extern RobotConfig config;
