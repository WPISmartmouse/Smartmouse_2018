#include <common/RobotConfig.h>

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
        MAX_HARDWARE_SPEED : 0.60,
        MIN_SPEED : 0.014,
        FRONT_WALL_THRESHOLD : 0.15,
        SIDE_WALL_THRESHOLD : 0.08,
        GERALD_WALL_THRESHOLD : 0.17,
        WALL_CHANGED_THRESHOLD : 0.02,
        ROT_TOLERANCE : 0.05,
        TRACK_WIDTH : 0.0633,
        ANALOG_MAX_DIST : 0.18,
        MAX_FORCE : 0.016,
        MIN_ABSTRACT_FORCE : 3.5,
        ARC_TURN : false,
        END_SPEED: 0.3, // this can be lowered to 0.15 to demonstrate ForwardN
};
