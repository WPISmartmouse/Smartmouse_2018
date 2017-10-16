#include <common/commands/SpeedRun.h>
#include <commands/WaitForStart.h>
#include <commands/Turn.h>
#include <commands/Forward.h>

SpeedRun::SpeedRun(Mouse *mouse) : CommandGroup("speed"), mouse(mouse) {}

void SpeedRun::initialize() {
  index = 0;
  path = &mouse->maze->fastest_route;
}

bool SpeedRun::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool returned = mouse->getRow() == smartmouse::maze::SIZE / 2
                    && mouse->getCol() == smartmouse::maze::SIZE / 2;

    if (!returned) {
      motion_primitive_t prim = path->at(index++);
      addSequential(new Turn(prim.d));
      addSequential(new Forward());
#ifdef CONSOLE
      addSequential(new WaitForStart());
      mouse->print_maze_mouse();
#endif
    } else {
      return true;
    }

    return false;
  }

  return false;
}
