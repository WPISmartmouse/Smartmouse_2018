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
#else
const RobotConfig config = {
        FRONT_ANALOG_ANGLE : 1.35255,
        BACK_ANALOG_ANGLE : 1.22,
        FRONT_SIDE_ANALOG_X : 0.04,
        FRONT_SIDE_ANALOG_Y : 0.024,
        BACK_SIDE_ANALOG_X : -0.024,
        BACK_SIDE_ANALOG_Y : 0.024,
        FRONT_ANALOG_X : 0.045,
        MAX_SPEED : 0.09,
        MIN_SPEED : 0.021,
        WALL_THRESHOLD : 0.15,
        ROT_TOLERANCE : 0.08,
        TRACK_WIDTH : 0.0633,
};
#endif