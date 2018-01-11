#include "Scheduler.h"

Scheduler::Scheduler(Command *masterCommand) {
  addCommand(masterCommand);
}


void Scheduler::addCommand(Command *command) {
  commands.add(command);
}

bool Scheduler::run() {
  //loop through commands and either init, execute, or end
  for (int i = 0; i < commands.size(); i++) {
    Command *command = commands.get(i);
    bool finished = command->cycle();
    if (finished) {
      Command *removed = commands.remove(i);
      delete removed;
    }
  }

  return commands.size() == 0;

}
