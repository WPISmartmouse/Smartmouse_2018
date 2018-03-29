#include <real/commands/BatteryMonitor.h>

BatteryMonitor::BatteryMonitor() : Command("battery monitor"), mouse(RealMouse::inst()){}

void BatteryMonitor::execute() {
  // this will print error messages if anything is wrong;
  mouse->checkVoltage();
}

bool BatteryMonitor::isFinished() {
  return false;
}
