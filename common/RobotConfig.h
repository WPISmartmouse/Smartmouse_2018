#pragma once

typedef struct {
  double FRONT_ANALOG_ANGLE;
  double BACK_ANALOG_ANGLE;
  double FRONT_SIDE_ANALOG_X;
  double FRONT_SIDE_ANALOG_Y;
  double BACK_SIDE_ANALOG_X;
  double BACK_SIDE_ANALOG_Y;
  double FRONT_ANALOG_X;
  double MAX_SPEED;
  double MIN_SPEED;
  double WALL_THRESHOLD;
  double ROT_TOLERANCE;
} RobotConfig;
