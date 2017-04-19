#include <common/Mouse.h>
#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait_calibrate"), mouse(RealMouse::inst()) {
}

void WaitForStart::initialize() {
  setTimeout(3000);
}

void WaitForStart::execute() {
//  data.push_back(mouse->getRangeData());
}

bool WaitForStart::isFinished() {
  if (isTimedOut()) {
//    double front_left_avg = 0;
//    double front_right_avg = 0;
//    double back_left_avg = 0;
//    double back_right_avg = 0;
//    double front_avg = 0;

//    for (RangeData datum : data) {
//      front_left_avg = 0.05 * datum.front_left_analog + 0.95 * front_left_avg;
//      front_right_avg = 0.05 * datum.front_right_analog + 0.95 * front_right_avg;
//      back_left_avg = 0.05 * datum.back_left_analog + 0.95 * back_left_avg;
//      back_right_avg = 0.05 * datum.back_right_analog + 0.95 * back_right_avg;
//      front_avg = 0.05 * datum.front_analog + 0.95 * front_avg;
//    }

    return true;
  }
  return false;
}
