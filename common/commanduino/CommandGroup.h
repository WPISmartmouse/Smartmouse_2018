#pragma once

#include "Command.h"
#include "LinkedList.h"

/** \brief grouping commands is a useful abstraction.
 * Commands groups execute commands in parallel or series
 */
class CommandGroup : public Command {
public:
  CommandGroup(const char *name);

  CommandGroup() = default;

  void addSequential(Command *command);

  void addParallel(Command *command);

  virtual void _initialize();

  virtual void _execute();

  virtual void _end();

  virtual void initialize();

  virtual void execute();

  virtual void end();

  virtual bool isFinished();

  LinkedList<Command *> commands;

private:

  int currentCommandIndex;
};
