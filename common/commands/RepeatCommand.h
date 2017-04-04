#pragma once

#include <common/commanduino/CommandGroup.h>

template<typename CommandType, typename... CommandArgs>
class RepeatCommand : public CommandGroup {
public:
  RepeatCommand (int count, CommandArgs... args) : CommandGroup("RepeatCommand") {
    for (int i = 0; i < count; i++) {
      addSequential(new CommandType(args...));
    }
  }
};
