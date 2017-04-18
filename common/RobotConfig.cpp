#include "RobotConfig.h"

#if defined(SIM)
const RobotConfig config = {
        FRONT_ANALOG_ANGLE : 1.35255,
        BACK_ANALOG_ANGLE : 1.35255,
        FRONT_SIDE_ANALOG_X : 0.045,
        FRONT_SIDE_ANALOG_Y : 0.030,
        BACK_SIDE_ANALOG_X : -0.024,
        BACK_SIDE_ANALOG_Y : 0.030,
        FRONT_ANALOG_X : 0.055,
        MAX_SPEED : 0.12,
        MIN_SPEED : 0.02,
        WALL_THRESHOLD : 0.15,
        ROT_TOLERANCE : 0.08,
        TRACK_WIDTH : 0.0633,
};
#elif defined(CONSOLE)

#else // REAL
const RobotConfig config = {
        FRONT_ANALOG_ANGLE : 1.35255,
        BACK_ANALOG_ANGLE : 1.22,
        FRONT_SIDE_ANALOG_X : 0.0416,
        FRONT_SIDE_ANALOG_Y : 0.0371,
        BACK_SIDE_ANALOG_X : -0.0361,
        BACK_SIDE_ANALOG_Y : 0.0361,
        FRONT_ANALOG_X : 0.055,
        MAX_SPEED : 0.09,
        MIN_SPEED : 0.021,
        WALL_THRESHOLD : 0.15,
        ROT_TOLERANCE : 0.14,
        TRACK_WIDTH : 0.0633,
};
#endif