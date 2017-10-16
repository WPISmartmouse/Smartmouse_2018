#include <commands/WaitForStart.h>
#include <common/commands/ReturnToStart.h>
#include <commands/Forward.h>
#include <commands/Turn.h>

ReturnToStart::ReturnToStart(Mouse *mouse) : CommandGroup("return"), mouse(mouse) {
}

void ReturnToStart::initialize() {
  //plan path from center to origin
  mouse->maze->flood_fill_from_point(&pathToStart, smartmouse::maze::SIZE / 2, smartmouse::maze::SIZE / 2, 0, 0);
  index = 0;
}

bool ReturnToStart::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool returned = mouse->getRow() == 0 && mouse->getCol() == 0;

    if (!returned) {
      motion_primitive_t prim = pathToStart[index++];
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
