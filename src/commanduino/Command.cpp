#include "Command.h"

#include "Scheduler.h"

TimerInterface *Command::timer;

void Command::setTimerImplementation(TimerInterface *timer){
  Command::timer = timer;
}

Command::Command() : initialized(false) {}

Command::Command(const char *name) : initialized(false),
  name(name),
  startTime(timer->programTimeMs()) {}

bool Command::cycle() {
  bool finished = false;

  if (!initialized) {
    initialize();
    _initialize();
    initialized = true;
  }
  else if (isFinished()) {
    finished = true;
    end();
    _end();
  }
  else {
    execute();
    _execute();
  }


  return finished;
}

void Command::setTimeout(unsigned long timeout) {
  this->timeout = timeout;
}

unsigned long Command::getTime() {
  return timer->programTimeMs() - startTime;
}

bool Command::isTimedOut() {
  return getTime() > timeout;
}

bool Command::isRunning() {
  return running;
}

void Command::initialize() {}
void Command::_initialize() {
  running = true;
  startTime = timer->programTimeMs();
}

void Command::execute() {}
void Command::_execute() {}

bool Command::isFinished() {}

void Command::end() {}
void Command::_end() {
  running = false;
}

bool Command::operator!=(const Command& other){
    return this->name != other.name;
}
