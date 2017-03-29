#include "Stop.h"

Stop::Stop() : Command("end") {}

Stop::Stop(unsigned long stop_time) : Command("end") {}

bool Stop::isFinished() {
}

void Stop::end() {
}
