#include "SpeedRun.h"
#include "Turn.h"
#include "Forward.h"

SpeedRun::SpeedRun(Mouse *mouse) : CommandGroup("speed"), mouse(mouse) {}

void SpeedRun::initialize(){
  index = 0;
  path = mouse->maze->fastest_route;
}

bool SpeedRun::isFinished(){
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool returned = mouse->getRow() == AbstractMaze::MAZE_SIZE/2
      && mouse->getCol() == AbstractMaze::MAZE_SIZE/2;

    if (!returned){
      char nextDirection = path[index++];
      addSequential(new Turn(char_to_dir(nextDirection)));
      addSequential(new Forward());
    }
    else {
      return true;
    }

    return false;
  }

  return false;
}
