#include "CommandGroup.h"

CommandGroup::CommandGroup(const char *name) : Command(name) {}

void CommandGroup::addSequential(Command *command) {
  command->inParallel = false;
  commands.add(command);
}

void CommandGroup::addParallel(Command *command) {
  command->inParallel = true;
  commands.add(command);
}

void CommandGroup::initialize() {}

void CommandGroup::_initialize() {
  currentCommandIndex = -1;
  Command::_initialize();
}

void CommandGroup::execute() {}

void CommandGroup::_execute() {
  Command *executingCommand = 0;
  bool done = false;

  currentCommandIndex = 0;

  while (!done && (currentCommandIndex < commands.size())) {
    executingCommand = commands.get(currentCommandIndex);

    bool isFinished = executingCommand->cycle();
    if (isFinished) {
      Command *removed = commands.remove(currentCommandIndex);
      delete removed;
      currentCommandIndex--;
      break;
    }

    // FIXME: address sanatizer found a bug here
    if (!executingCommand->inParallel) {
      done = true;
    } else {
      currentCommandIndex++;
    }
  }
}

void CommandGroup::end() {}

void CommandGroup::_end() {
  currentCommandIndex = 0;
}


bool CommandGroup::isFinished() {
  return commands.size() <= 0;
}
