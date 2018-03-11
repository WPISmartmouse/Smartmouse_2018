#include <common/KinematicController/RobotConfig.h>

namespace smartmouse {
namespace kc {

double MAX_SPEED_MPS = 0.72;
double MAX_SPEED_CUPS = smartmouse::maze::toCellUnits(MAX_SPEED_MPS);
bool ARC_TURN = false;

}
}
