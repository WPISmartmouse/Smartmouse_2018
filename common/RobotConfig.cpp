#include "RobotConfig.h"

#if defined(SIM)
RobotConfig config = {
        FRONT_ANALOG_ANGLE : 1.35255,
        BACK_ANALOG_ANGLE : 1.35255,
        FRONT_SIDE_ANALOG_X : 0.045,
        FRONT_SIDE_ANALOG_Y : 0.030,
        BACK_SIDE_ANALOG_X : -0.024,
        BACK_SIDE_ANALOG_Y : 0.030,
        FRONT_ANALOG_X : 0.055,
        GERALD_X : 0.050,
        GERALD_Y : 0.018,
        GERALD_ANGLE : 1.0472,
        MAX_SPEED : 0.15,
        MIN_SPEED : 0.013,
        FRONT_WALL_THRESHOLD : 0.15,
        SIDE_WALL_THRESHOLD : 0.08,
        GERALD_WALL_THRESHOLD : 0.015,
        WALL_CHANGED_THRESHOLD : 0.01,
        ROT_TOLERANCE : 0.14,
        TRACK_WIDTH : 0.0633,
        ANALOG_MAX_DIST : 0.18,
        MAX_FORCE : 0.014,
};
#elif defined(CONSOLE)

#else // REAL
RobotConfig config = {
        FRONT_ANALOG_ANGLE : 1.35255,
        BACK_ANALOG_ANGLE : 1.35255,
        FRONT_SIDE_ANALOG_X : 0.042,
        FRONT_SIDE_ANALOG_Y : 0.0350,
        BACK_SIDE_ANALOG_X : -0.0206,
        BACK_SIDE_ANALOG_Y : 0.0335,
        FRONT_ANALOG_X : 0.056,
        GERALD_X : 0.054,
        GERALD_Y : 0.018,
        GERALD_ANGLE : 0.6544985,
        MAX_SPEED : 0.60,
        MIN_SPEED : 0.013,
        FRONT_WALL_THRESHOLD : 0.15,
        SIDE_WALL_THRESHOLD : 0.08,
        GERALD_WALL_THRESHOLD : 0.17,
        WALL_CHANGED_THRESHOLD : 0.02,
        ROT_TOLERANCE : 0.05,
        TRACK_WIDTH : 0.0633,
        ANALOG_MAX_DIST : 0.18,
        MAX_FORCE : 0.016,
};
#endif
